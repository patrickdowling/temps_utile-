// Copyright 2021 Patrick Dowling
// Author: Patrick Dowling (pld@gurkenkiste.com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
// See http://creativecommons.org/licenses/MIT/ for more information.
//

#include "APP_SCOPE.h"

#include <algorithm>
#include <functional>

#include "TU_ADC.h"
#include "TU_debug.h"
#include "TU_digital_inputs.h"
#include "TU_menus.h"
#include "TU_strings.h"
#include "TU_ui.h"
#include "UI/ui_event_dispatcher.h"
#include "arm_math.h"
#include "util/util_circular_sample_buffer.h"
#include "util/util_edge_detector.h"
#include "util/util_popup.h"
#include "util/util_sample_decimator.h"
#include "util/util_settings.h"

// NOTES
// - SIMD processing of buffers?
// - Raw values from ADC are inverted, but calibration offset is applied
// - For linked channels, the assumption is that everything will /2 and "just work"

// TODO buffer alignment for external triggers
// TODO If external trigger mode, infinite hodloff?
// TODO Shared vs. common settings (linked vs. advanced mode)
// TODO Instead of decimating incoming samples, do it when reading the buffer (or rendering?)

namespace scope {

namespace menu = TU::menu;

static constexpr weegfx::coord_t kDisplayFrameSize = weegfx::Graphics::kWidth;
static constexpr size_t kADCChunkSize = TU::ADC::kDMAChunkSize;
static constexpr uint32_t kTriggerLostIndicatorTimeoutTicks = TU_CORE_ISR_FREQ / 3;

static constexpr weegfx::coord_t kScreenHeight = weegfx::Graphics::kHeight;
static constexpr weegfx::coord_t kScreenCenterY = kScreenHeight / 2;

static constexpr weegfx::coord_t kStatusBarY = 64 - weegfx::kFixedFontH;

static constexpr int kMaxGraticuleDensity = 5;

static constexpr int kRangeVolts = 5;
static constexpr int kTriggerLevelStepsPerV = 10;
static constexpr int kTriggerLevelIncrement =
    2048.f / float(kRangeVolts * kTriggerLevelStepsPerV) + 0.5f;
static constexpr int kTriggerLevelRange = kTriggerLevelIncrement * (5 * kTriggerLevelStepsPerV - 1);

static debug::AveragedCycles process_cycles;

enum InputRange {
  INPUT_RANGE_BI,
  INPUT_RANGE_UNI,
  INPUT_RANGE_LAST,
  INPUT_RANGE_FIRST = INPUT_RANGE_BI
};

// Helper class to process buffers and find triggers
//
class TriggerProcessor {
public:
  enum TriggerType {
    TRIGGER_TYPE_NONE,
    TRIGGER_TYPE_RISING,
    TRIGGER_TYPE_FALLING,
    TRIGGER_TYPE_EXT1,
    TRIGGER_TYPE_EXT2,
    TRIGGER_TYPE_LAST
  };

  using TriggerOffset = size_t;

  template <size_t buffer_length, int stride>
  const TriggerOffset Process(TriggerType trigger_type, int16_t threshold, const int16_t *buffer)
  {
    using Impl = TriggerOffset (TriggerProcessor::*)(int16_t, const int16_t *);
    static constexpr Impl processors[TRIGGER_TYPE_LAST] = {
        &TriggerProcessor::Nop<buffer_length>,
        &TriggerProcessor::FindEdges<buffer_length, stride, std::greater<int16_t>>,  // rising
        &TriggerProcessor::FindEdges<buffer_length, stride, std::less<int16_t>>,     // falling
        &TriggerProcessor::FindEdges<buffer_length, stride>,
        &TriggerProcessor::FindEdges<buffer_length, stride>,
    };

    auto trigger_offset = (this->*processors[trigger_type])(threshold, buffer);
    stats_.sample_count += buffer_length;
    return trigger_offset;
  }

  struct Stats {
    uint32_t sample_count{0};
    uint32_t trigger_count{0};
    uint32_t edge_count{0};
  };

  const Stats &stats() const { return stats_; }

  void ResetEdgeCounter()
  {
    stats_.edge_count = 0;
    stats_.sample_count = 0;
  }

private:
  Stats stats_;

  using EdgeDetector = util::EdgeDetector<int16_t, 2>;

  EdgeDetector::State edge_detector_state_ = 0;

  template <size_t buffer_length>
  TriggerOffset Nop(int16_t, const int16_t *buffer)
  {
    edge_detector_state_ = 0;
    return buffer_length;
  }

  template <size_t buffer_length, int stride, typename cmp>
  TriggerOffset FindEdges(int16_t threshold, const int16_t *buffer)
  {
    auto buf = buffer;
    auto end = buffer + buffer_length;
    uint32_t edge_count = 0;
    const int16_t *first_edge = nullptr;

    EdgeDetector edge_detector{threshold, edge_detector_state_};
    do {
      edge_detector.Update<cmp>(buf[0]);
      if (edge_detector.rising_edge()) {
        if (!first_edge) first_edge = buf;
        ++edge_count;
      }

      buf += stride;
    } while (buf < end);

    stats_.edge_count += edge_count;
    edge_detector_state_ = edge_detector.state();

    if (first_edge) {
      ++stats_.trigger_count;
      return first_edge - buffer;
    } else {
      return buffer_length;
    }
  }

  template <size_t buffer_length, int stride>
  TriggerOffset FindEdges(int16_t threshold, const int16_t *buffer)
  {
    auto buf = buffer;
    auto end = buffer + buffer_length;
    uint32_t edge_count = 0;

    EdgeDetector edge_detector{threshold, edge_detector_state_};
    do {
      edge_detector.Update<std::greater<int16_t>>(buf[0]);
      if (edge_detector.rising_edge()) ++edge_count;

      buf += stride;
    } while (buf < end);

    stats_.edge_count += edge_count;
    return buffer_length;
  }
};

static constexpr const char *kTriggerTypeStrings[TriggerProcessor::TRIGGER_TYPE_LAST] = {
    "none", "rising", "falling", "ext1", "ext2",
};

struct TimebaseParameters {
  const char *const label;
  uint32_t adc_frequency;
  uint32_t decimate;
  // auto gain?
  // retrigger delay
};

// BEGIN generated via resources/scope_divs.py
// clang-format off
enum Timebase {
  TIMEBASE_500u,
  TIMEBASE_1m,
  TIMEBASE_2m,
  TIMEBASE_5m,
  TIMEBASE_10m,
  TIMEBASE_20m,
  TIMEBASE_50m,
  TIMEBASE_100m,
  TIMEBASE_200m,
  TIMEBASE_500m,
  TIMEBASE_1s,
  TIMEBASE_2s,
  TIMEBASE_5s,
  TIMEBASE_LAST,
};

static constexpr TimebaseParameters kTimebaseParameters[TIMEBASE_LAST] = {
{ .label = "500u", .adc_frequency = 256000, .decimate = 1 },
{ .label = "  1m", .adc_frequency = 128000, .decimate = 1 },
{ .label = "  2m", .adc_frequency = 64000, .decimate = 1 },
{ .label = "  5m", .adc_frequency = 25600, .decimate = 1 },
{ .label = " 10m", .adc_frequency = 12800, .decimate = 1 },
{ .label = " 20m", .adc_frequency = 6400, .decimate = 1 },
{ .label = " 50m", .adc_frequency = 2560, .decimate = 1 },
{ .label = "100m", .adc_frequency = 2560, .decimate = 2 },
{ .label = "200m", .adc_frequency = 2560, .decimate = 4 },
{ .label = "500m", .adc_frequency = 2048, .decimate = 8 },
{ .label = "  1s", .adc_frequency = 2048, .decimate = 16 },
{ .label = "  2s", .adc_frequency = 1024, .decimate = 16 },
{ .label = "  5s", .adc_frequency = 819, .decimate = 32 },
};
// clang-format on
// END generated

enum Scaling {
  DIV_0V5,
  DIV_1V,
  DIV_2V,
  DIV_5V,
  DIV_10V,
  DIV_LAST,
};

static constexpr int32_t kScalingShift = 10;  // \sa to_pixel
struct ScalingParameters {
  const char *label;
  int32_t multiplier;
  // grid spacing?
};

static constexpr int32_t scaling_multiplier(float division)
{
  return 5.f / division * (float)(1 << kScalingShift);
}

static constexpr ScalingParameters kScalingParameters[DIV_LAST] = {
    {"0.5V", scaling_multiplier(0.5f)}, {"  1V", scaling_multiplier(1.f)},
    {"  2V", scaling_multiplier(2.f)},  {"  5V", scaling_multiplier(5.f)},
    {" 10V", scaling_multiplier(10.f)},
};

// Scope channel class; maintains settings and can process buffers
//
enum ScopeChannelSetting {
  SCOPE_CHANNEL_SETTING_RANGE,
  SCOPE_CHANNEL_SETTING_XOFF,  // screen space
  SCOPE_CHANNEL_SETTING_YOFF,  // screen space
  SCOPE_CHANNEL_SETTING_XDIV,
  SCOPE_CHANNEL_SETTING_YDIV,
  SCOPE_CHANNEL_SETTING_TRIG_TYPE,
  SCOPE_CHANNEL_SETTING_TRIG_LEVEL,
  SCOPE_CHANNEL_SETTING_TRIG_HOLDOFF,
  SCOPE_CHANNEL_SETTING_LAST,
  SCOPE_CHANNEL_SETTING_FIRST = SCOPE_CHANNEL_SETTING_XOFF,
};

class ScopeChannel : public settings::SettingsBase<ScopeChannel, SCOPE_CHANNEL_SETTING_LAST>,
                     public settings::DynamicSettings<ScopeChannel, SCOPE_CHANNEL_SETTING_LAST> {
public:
  void Init();

  template <size_t buffer_length, int stride>
  TriggerProcessor::TriggerOffset Process(const TU::ADC::Chunk *chunk)
  {
    auto trigger = trigger_processor_.Process<buffer_length, stride>(
        trigger_type(), trigger_level(), chunk->buffer);

    auto &stats = trigger_processor_.stats();
    if (stats.sample_count >= timebase().adc_frequency) {
      freq_.push(stats.edge_count);
      trigger_processor_.ResetEdgeCounter();
    }

    if (TriggerProcessor::TRIGGER_TYPE_EXT1 == trigger_type() ||
        TriggerProcessor::TRIGGER_TYPE_EXT2 == trigger_type()) {
      if (0xffff != chunk->info.ext_trigger_offset) {
        return chunk->info.ext_trigger_offset;
      } else {
        return buffer_length;
      }
    } else {
      return trigger;
    }
  }

  const TriggerProcessor::Stats stats() const { return trigger_processor_.stats(); }
  uint32_t frequency() const { return freq_.value(); }

  // Settings getters
  int input_range() const { return get_value(SCOPE_CHANNEL_SETTING_RANGE); }
  int xdiv() const { return get_value(SCOPE_CHANNEL_SETTING_XDIV); }
  int ydiv() const { return get_value(SCOPE_CHANNEL_SETTING_YDIV); }

  int screen_xoffset() const { return get_value(SCOPE_CHANNEL_SETTING_XOFF); }
  int screen_yoffset() const
  {
    return INPUT_RANGE_BI == input_range() ? kScreenCenterY : kScreenHeight;
  }

  TriggerProcessor::TriggerType trigger_type() const
  {
    return static_cast<TriggerProcessor::TriggerType>(get_value(SCOPE_CHANNEL_SETTING_TRIG_TYPE));
  }

  int trigger_level_displayable() const { return (trigger_level() * kRangeVolts * 1000) >> 11; }

  int16_t trigger_level() const
  {
    auto level = static_cast<int16_t>(get_value(SCOPE_CHANNEL_SETTING_TRIG_LEVEL));
    return INPUT_RANGE_BI == input_range() ? level : level + 2048;
  }

  int trigger_holdoff() const { return get_value(SCOPE_CHANNEL_SETTING_TRIG_HOLDOFF); }

  const TimebaseParameters &timebase() const { return kTimebaseParameters[xdiv()]; }
  const ScalingParameters &scaling() const { return kScalingParameters[ydiv()]; }

private:
  TriggerProcessor trigger_processor_;

  RunningAverage<uint32_t, 4> freq_;
};

SETTINGS_DECLARE(scope::ScopeChannel, scope::SCOPE_CHANNEL_SETTING_LAST){
    // default, min, max, name, value_names, storage_type, parent_index, parent_value
    {scope::INPUT_RANGE_BI, scope::INPUT_RANGE_FIRST, scope::INPUT_RANGE_LAST, "Uni/bi", nullptr,
     settings::STORAGE_TYPE_U4},
    {0, 0, 127, "XOFF", nullptr, settings::STORAGE_TYPE_I16},
    {0, -32, 32, "YOFF", nullptr, settings::STORAGE_TYPE_I16},
    {1, 0, scope::TIMEBASE_LAST - 1, "XDIV", nullptr, settings::STORAGE_TYPE_U8},
    {scope::DIV_5V, scope::DIV_0V5, scope::DIV_LAST - 1, "YDIV", nullptr,
     settings::STORAGE_TYPE_U8},
    {scope::TriggerProcessor::TRIGGER_TYPE_RISING, scope::TriggerProcessor::TRIGGER_TYPE_NONE,
     scope::TriggerProcessor::TRIGGER_TYPE_LAST - 1, "TRIG TYPE", scope::kTriggerTypeStrings,
     settings::STORAGE_TYPE_U8},
    {0, -scope::kTriggerLevelRange, scope::kTriggerLevelRange, "TRIG LVL", nullptr,
     settings::STORAGE_TYPE_I32},
    {8, 0, 64, "TRIG HOLD", nullptr, settings::STORAGE_TYPE_U8},
};

void ScopeChannel::Init()
{
  InitDefaults();
}

enum ScopeAppSetting {
  SCOPE_APP_SETTING_CHANNEL,
  SCOPE_APP_SETTING_FREQ_COUNTER,
  SCOPE_APP_SETTING_STATS_OVERLAY,
  SCOPE_APP_SETTING_GRATICULE,
  SCOPE_APP_SETTING_LINK12,
  SCOPE_APP_SETTING_LINK34,
  SCOPE_APP_SETTING_RESET,
  SCOPE_APP_SETTING_LAST,
};

class ScopeApp : public settings::SettingsBase<ScopeApp, SCOPE_APP_SETTING_LAST>,
                 public UI::EventDispatcher<ScopeApp> {
public:
  static constexpr int kNumChannels = 4;

  void Init();
  void Process();
  void UpdateUI();

  size_t SaveState(util::StreamBufferWriter &stream_buffer) const;
  size_t RestoreState(util::StreamBufferReader &stream_buffer);

  void Render();             // const;
  void RenderScreensaver();  // const;

  void EventScreensaverOff();
  void Activate();

  bool display_frequency_counter() const { return get_value(SCOPE_APP_SETTING_FREQ_COUNTER); }
  int display_stats_overlay() const { return get_value(SCOPE_APP_SETTING_STATS_OVERLAY); }
  int graticule_density() const { return get_value(SCOPE_APP_SETTING_GRATICULE); }

private:
  struct {
    bool menu_active = false;

    ScopeChannelSetting edit_setting_l = SCOPE_CHANNEL_SETTING_XDIV;
    ScopeChannelSetting edit_setting_r = SCOPE_CHANNEL_SETTING_YDIV;

    volatile uint32_t trigger_lost = 0;

    util::PopupElement edit_setting;
    util::PopupElement status_bar;

    menu::ScreenCursor<menu::kScreenLines> cursor;
  } ui_;

  struct {
    bool triggered = false;
    int holdoff = 0;
  } trigger_state_;

  struct {
    uint32_t skipped_frames_ = 0;
  } debug_;

  // The channel configuration is packed into an int so access within ISR is atomic.
  // We're relying on the fact that channel index = ADC channel
  // Still tempted to just use a union, even if technically UB.
  struct ChannelConfig {
    constexpr ChannelConfig() : packed_value{Pack(ADC_CHANNEL_1, ADC_CHANNEL_LAST)} {}
    constexpr ChannelConfig(ADC_CHANNEL main, ADC_CHANNEL aux) : packed_value{Pack(main, aux)} {}

    int main() const { return packed_value & 0xffff; }
    int aux() const { return packed_value >> 16; }

    ADC_CHANNEL main_adc_channel() const { return static_cast<ADC_CHANNEL>(main()); }
    ADC_CHANNEL aux_adc_channel() const { return static_cast<ADC_CHANNEL>(aux()); }

    bool linked() const { return ADC_CHANNEL_LAST != aux_adc_channel(); }

    int packed_value = Pack(ADC_CHANNEL_1, ADC_CHANNEL_LAST);

    static constexpr int Pack(int main, int aux) { return main | (aux << 16); }
  };

  ChannelConfig channel_config_ = {};

  struct FrameInfo {
    int num_channels = 1;
    size_t length = 0;
  };

  using CircularSampleBuffer = util::CircularSampleBuffer<int16_t, kADCChunkSize * 4>;
  using DisplayFrameBuffer = util::FrameBuffer<kDisplayFrameSize, 2, int16_t, FrameInfo>;
  using DisplayFrame = DisplayFrameBuffer::Frame;
  using SampleDecimator = util::SampleDecimator<kADCChunkSize>;

  ScopeChannel channels_[kNumChannels];

  static CircularSampleBuffer sample_buffer_;
  static DisplayFrameBuffer display_frame_buffer_;

  static int16_t display_buffer_[kDisplayFrameSize];
  static DisplayFrame current_display_frame_;

  const ScopeChannel &main_channel() const { return channels_[channel_config_.main()]; }
  ScopeChannel &main_channel() { return channels_[channel_config_.main()]; }

  const ScopeChannel &aux_channel() const { return channels_[channel_config_.aux()]; }

  int selected_channel_index() const { return get_value(SCOPE_APP_SETTING_CHANNEL); }
  ScopeChannel &selected_channel() { return channels_[get_value(SCOPE_APP_SETTING_CHANNEL)]; }
  const ScopeChannel &selected_channel() const
  {
    return channels_[get_value(SCOPE_APP_SETTING_CHANNEL)];
  }

  void UpdateChannelConfig();
  void ConfigureADC();
  void ConfigureTR();

  static void DrawGraticule(int density);
  static void DrawWaveform(const int16_t *buffer, size_t length, size_t stride, int32_t multiplier,
                           const weegfx::coord_t y);
  void RenderDisplayBuffer() const;
  void RenderMenu() const;
  void RenderScopeUI() const;

  void DrawStatusBar() const;
  void DisplayFrequencyCounter(weegfx::coord_t x, weegfx::coord_t y, uint32_t frequency) const;

  void UpdateDisplayBuffer();

  // Event handlers
  friend class EventDispatcher;
  const EventHandler *get_event_handlers() const
  {
    return ui_.menu_active ? menu_event_handlers : scope_button_handlers;
  }

  static const EventHandler menu_event_handlers[];
  static const EventHandler scope_button_handlers[];

  EVENT_DISPATCH_DECLARE_HANDLER(toggleMenu);
  EVENT_DISPATCH_DECLARE_HANDLER(toggleInputRange);
  EVENT_DISPATCH_DECLARE_HANDLER(scopeButtonDown);
  EVENT_DISPATCH_DECLARE_HANDLER(scopeButtonL);
  EVENT_DISPATCH_DECLARE_HANDLER(scopeButtonR);
  EVENT_DISPATCH_DECLARE_HANDLER(scopeEncoderL);
  EVENT_DISPATCH_DECLARE_HANDLER(scopeEncoderR);
  EVENT_DISPATCH_DECLARE_HANDLER(menuButtonL);
  EVENT_DISPATCH_DECLARE_HANDLER(menuButtonR);
  EVENT_DISPATCH_DECLARE_HANDLER(menuEncoderL);
  EVENT_DISPATCH_DECLARE_HANDLER(menuEncoderR);
};

/*static*/ ScopeApp::CircularSampleBuffer ScopeApp::sample_buffer_ __attribute__((aligned(4)));
/*static*/ ScopeApp::DisplayFrameBuffer ScopeApp::display_frame_buffer_ __attribute__((aligned(4)));
/*static*/ int16_t ScopeApp::display_buffer_[kDisplayFrameSize] __attribute__((aligned(4)));
/*static*/ ScopeApp::DisplayFrame ScopeApp::current_display_frame_{display_buffer_, {1, 0}};

static const char *const stats_overlay_strings[] = {"off", "proc", "draw"};

static const char *const graticule_density_strings[kMaxGraticuleDensity] = {
    "off", "25%", "50%", "75%", "max",
};

SETTINGS_DECLARE(scope::ScopeApp, scope::SCOPE_APP_SETTING_LAST){
    // default, min, max, name, value_names, storage_type, parent_index, parent_value
    {0, 0, scope::ScopeApp::kNumChannels - 1, "CHANNEL", nullptr, settings::STORAGE_TYPE_U8},
    {1, 0, 1, "Disp freq", TU::Strings::no_yes, settings::STORAGE_TYPE_U4},
    {0, 0, 2, "Disp stats", scope::stats_overlay_strings, settings::STORAGE_TYPE_U4},
    {scope::kMaxGraticuleDensity - 1, 0, scope::kMaxGraticuleDensity - 1, "Graticule",
     scope::graticule_density_strings, settings::STORAGE_TYPE_U4},
    {0, 0, 1, "Link 1+2", TU::Strings::no_yes, settings::STORAGE_TYPE_U4},
    {0, 0, 1, "Link 3+4", TU::Strings::no_yes, settings::STORAGE_TYPE_U4},
    {0, 0, 0, "Reset", nullptr, settings::STORAGE_TYPE_NOP}};

void ScopeApp::Init()
{
  InitDefaults();
  for (auto &channel : channels_) channel.Init();

  display_frame_buffer_.Init();
  ui_.cursor.Init(SCOPE_APP_SETTING_FREQ_COUNTER, SCOPE_APP_SETTING_LINK34);

  UpdateChannelConfig();
}

void ScopeApp::Process()
{
  uint32_t trigger_lost = ui_.trigger_lost;
  if (trigger_lost) --trigger_lost;

  const auto channel_config = channel_config_;
  auto &channel = channels_[channel_config.main()];
  auto &timebase = channel.timebase();

  auto &adc_chunks = TU::ADC::chunk_buffers();
  while (adc_chunks.readable()) {
    debug::ScopedCycleMeasurement cycles{process_cycles};

    // Process input buffer; if already triggered we don't really need to find a new one yet but
    // the process function also handles other stats (e.g. frequency counter)
    auto adc_chunk = adc_chunks.readable_frame();
    auto trigger = channel_config.linked() ? channel.Process<kADCChunkSize, 2>(adc_chunk)
                                           : channel.Process<kADCChunkSize, 1>(adc_chunk);

    auto sample_writer = sample_buffer_.writer();
    // TODO account for linked channels?
    sample_writer =
        timebase.decimate > 1
            ? SampleDecimator::Process(sample_writer, adc_chunk->buffer, timebase.decimate)
            : std::copy(adc_chunk->buffer, adc_chunk->buffer + kADCChunkSize, sample_writer);
    adc_chunks.read();
    sample_writer.Commit();

    // Trigger/display buffer handling
    size_t available = sample_buffer_.available();
    size_t read_length = 0;
    if (trigger_state_.triggered) {
      if (available < kDisplayFrameSize) {
        // still accumulating. This should generally only happen if decimating, otherwise the new
        // data in this pass should have filled the buffer. So if decimation > 1, we might display
        // partial buffers (since that's likely slow update rate), but this has some flickery
        // effects.
        if (timebase.decimate > 1) read_length = available;
      } else {
        // buffer full, rearm and start again
        trigger_state_.triggered = false;
        trigger_state_.holdoff = channel.trigger_holdoff();
        read_length = kDisplayFrameSize;
      }
    } else {
      if (trigger_state_.holdoff) {
        --trigger_state_.holdoff;
      } else {
        if (trigger < kADCChunkSize) {
          trigger_state_.triggered = true;
          auto n = kADCChunkSize - trigger;
          if (timebase.decimate) n /= timebase.decimate;
          sample_buffer_.SetReadOffset(-n - kDisplayFrameSize / 2);
          if (timebase.decimate > 1) read_length = kDisplayFrameSize / 2;
        } else {
          // This provides a scrolling view
          trigger_lost = kTriggerLostIndicatorTimeoutTicks;
          sample_buffer_.SetReadOffset(-kDisplayFrameSize);
          read_length = kDisplayFrameSize;
        }
      }
    }

    if (read_length) {
      if (display_frame_buffer_.writeable()) {
        auto frame = display_frame_buffer_.writeable_frame();
        sample_buffer_.Read(frame->buffer, read_length);
        frame->info.num_channels = channel_config.linked() ? 2 : 1;
        frame->info.length = read_length;
        display_frame_buffer_.written();
      } else {
        ++debug_.skipped_frames_;
      }
    }
    // if (available >= kDisplayFrameSize) sample_buffer_.Consume(kDisplayFrameSize);
  }

  // Other regular book-keeping?
  ui_.trigger_lost = trigger_lost;
}

void ScopeApp::UpdateUI()
{
  auto ticks = TU::ui.ticks();
  ui_.edit_setting.Tick(ticks);
  ui_.status_bar.Tick(ticks);
}

size_t ScopeApp::SaveState(util::StreamBufferWriter &stream_buffer) const
{
  Save(stream_buffer);
  for (auto &channel : channels_) channel.Save(stream_buffer);

  return stream_buffer.overflow() ? 0 : stream_buffer.written();
}

size_t ScopeApp::RestoreState(util::StreamBufferReader &stream_buffer)
{
  Restore(stream_buffer);
  for (auto &channel : channels_) channel.Restore(stream_buffer);

  return stream_buffer.underflow() ? 0 : stream_buffer.read();
}

/*static*/ const ScopeApp::EventHandler ScopeApp::menu_event_handlers[] = {
    {UI::EVENT_BUTTON_PRESS, TU::CONTROL_BUTTON_UP, &ScopeApp::toggleMenu},
    {UI::EVENT_BUTTON_PRESS, TU::CONTROL_BUTTON_DOWN, &ScopeApp::toggleMenu},
    {UI::EVENT_BUTTON_LONG_PRESS, TU::CONTROL_BUTTON_L, &ScopeApp::toggleMenu},
    {UI::EVENT_BUTTON_PRESS, TU::CONTROL_BUTTON_L, &ScopeApp::menuButtonL},
    {UI::EVENT_BUTTON_PRESS, TU::CONTROL_BUTTON_R, &ScopeApp::menuButtonR},
    {UI::EVENT_ENCODER, TU::CONTROL_ENCODER_L, &ScopeApp::menuEncoderL},
    {UI::EVENT_ENCODER, TU::CONTROL_ENCODER_R, &ScopeApp::menuEncoderR},
    {},
};

/*static*/ const ScopeApp::EventHandler ScopeApp::scope_button_handlers[] = {
    {UI::EVENT_BUTTON_PRESS, TU::CONTROL_BUTTON_UP, &ScopeApp::toggleInputRange},
    {UI::EVENT_BUTTON_PRESS, TU::CONTROL_BUTTON_DOWN, &ScopeApp::scopeButtonDown},
    {UI::EVENT_BUTTON_PRESS, TU::CONTROL_BUTTON_L, &ScopeApp::scopeButtonL},
    {UI::EVENT_BUTTON_LONG_PRESS, TU::CONTROL_BUTTON_L, &ScopeApp::toggleMenu},
    {UI::EVENT_BUTTON_PRESS, TU::CONTROL_BUTTON_R, &ScopeApp::scopeButtonR},
    {UI::EVENT_ENCODER, TU::CONTROL_ENCODER_L, &ScopeApp::scopeEncoderL},
    {UI::EVENT_ENCODER, TU::CONTROL_ENCODER_R, &ScopeApp::scopeEncoderR},
    {},
};

EVENT_DISPATCH_DEFINE_HANDLER(ScopeApp, toggleMenu)
{
  EVENT_DISPATCH_HANDLER_STUB();

  if (ui_.menu_active) {
    ui_.menu_active = false;
  } else {
    ui_.menu_active = true;
  }
}

EVENT_DISPATCH_DEFINE_HANDLER(ScopeApp, toggleInputRange)
{
  EVENT_DISPATCH_HANDLER_STUB();

  auto &channel = selected_channel();
  channel.apply_value(SCOPE_CHANNEL_SETTING_RANGE,
                      INPUT_RANGE_BI == channel.input_range() ? INPUT_RANGE_UNI : INPUT_RANGE_BI);
}

EVENT_DISPATCH_DEFINE_HANDLER(ScopeApp, scopeButtonDown)
{
  EVENT_DISPATCH_HANDLER_STUB();

  auto &channel = selected_channel();
  if (channel.change_value_wrap(SCOPE_CHANNEL_SETTING_TRIG_TYPE, 1)) ConfigureTR();
}

EVENT_DISPATCH_DEFINE_HANDLER(ScopeApp, scopeButtonL)
{
  EVENT_DISPATCH_HANDLER_STUB();

  ui_.edit_setting_l = SCOPE_CHANNEL_SETTING_XDIV == ui_.edit_setting_l
                           ? SCOPE_CHANNEL_SETTING_LAST
                           : SCOPE_CHANNEL_SETTING_XDIV;
  ui_.status_bar.show();
  ui_.edit_setting.hide();
}

EVENT_DISPATCH_DEFINE_HANDLER(ScopeApp, scopeButtonR)
{
  EVENT_DISPATCH_HANDLER_STUB();

  if (SCOPE_CHANNEL_SETTING_YDIV == ui_.edit_setting_r &&
      TriggerProcessor::TRIGGER_TYPE_NONE != selected_channel().trigger_type()) {
    ui_.edit_setting_r = SCOPE_CHANNEL_SETTING_TRIG_LEVEL;
    ui_.edit_setting.show();
    ui_.status_bar.hide();
  } else {
    ui_.edit_setting_r = SCOPE_CHANNEL_SETTING_YDIV;
    ui_.status_bar.show();
    ui_.edit_setting.hide();
  }
}

EVENT_DISPATCH_DEFINE_HANDLER(ScopeApp, scopeEncoderL)
{
  EVENT_DISPATCH_HANDLER_STUB();

  bool update_adc = false;
  switch (ui_.edit_setting_l) {
    case SCOPE_CHANNEL_SETTING_XDIV:
      update_adc = main_channel().change_value(SCOPE_CHANNEL_SETTING_XDIV, event_value);
      break;
    case SCOPE_CHANNEL_SETTING_LAST:
      if (change_value(SCOPE_APP_SETTING_CHANNEL, event_value)) {
        UpdateChannelConfig();
        update_adc = true;
      }
      break;
    default: break;
  }
  ui_.edit_setting.hide();
  ui_.status_bar.show();
  if (update_adc) {
    ConfigureADC();
    ConfigureTR();
  }
}

EVENT_DISPATCH_DEFINE_HANDLER(ScopeApp, scopeEncoderR)
{
  EVENT_DISPATCH_HANDLER_STUB();

  switch (ui_.edit_setting_r) {
    case SCOPE_CHANNEL_SETTING_TRIG_LEVEL:
      main_channel().change_value(SCOPE_CHANNEL_SETTING_TRIG_LEVEL,
                                  event_value * kTriggerLevelIncrement);
      ui_.edit_setting.show();
      ui_.status_bar.hide();
      break;
    case SCOPE_CHANNEL_SETTING_YDIV:
      selected_channel().change_value(SCOPE_CHANNEL_SETTING_YDIV, event_value);
    default: ui_.status_bar.show(); break;
  }
}

EVENT_DISPATCH_DEFINE_HANDLER(ScopeApp, menuButtonL)
{
  EVENT_DISPATCH_HANDLER_STUB();
}

EVENT_DISPATCH_DEFINE_HANDLER(ScopeApp, menuButtonR)
{
  EVENT_DISPATCH_HANDLER_STUB();

  if (SCOPE_APP_SETTING_CHANNEL != ui_.cursor.cursor_pos()) ui_.cursor.toggle_editing();
}

EVENT_DISPATCH_DEFINE_HANDLER(ScopeApp, menuEncoderL)
{
  EVENT_DISPATCH_HANDLER_STUB();
}

EVENT_DISPATCH_DEFINE_HANDLER(ScopeApp, menuEncoderR)
{
  EVENT_DISPATCH_HANDLER_STUB();

  if (!ui_.cursor.editing()) {
    ui_.cursor.Scroll(event_value);
  } else {
    auto setting = ui_.cursor.cursor_pos();
    if (change_value(setting, event_value)) {
      switch (setting) {
        case SCOPE_APP_SETTING_LINK12:
        case SCOPE_APP_SETTING_LINK34:
          UpdateChannelConfig();
          ConfigureADC();
          ConfigureTR();
          break;
        default: break;
      }
    }
  }
}

void ScopeApp::UpdateChannelConfig()
{
  auto channel_index = selected_channel_index();

  ADC_CHANNEL main = static_cast<ADC_CHANNEL>(channel_index);
  ADC_CHANNEL aux = ADC_CHANNEL_LAST;

  if (channel_index < 2 && get_value(SCOPE_APP_SETTING_LINK12)) {
    main = ADC_CHANNEL_1;
    aux = ADC_CHANNEL_2;
  } else if (channel_index >= 2 && get_value(SCOPE_APP_SETTING_LINK34)) {
    main = ADC_CHANNEL_3;
    aux = ADC_CHANNEL_4;
  }

  channel_config_ = {main, aux};
}

void ScopeApp::ConfigureADC()
{
  TU::ADC::StartConversionBuffered(main_channel().timebase().adc_frequency,
                                   channel_config_.main_adc_channel(),
                                   channel_config_.aux_adc_channel());
}

void ScopeApp::ConfigureTR()
{
  auto trigger_type = main_channel().trigger_type();
  if (TriggerProcessor::TRIGGER_TYPE_EXT1 == trigger_type)
    TU::DigitalInputs::EnableDMARequest(TU::DIGITAL_INPUT_1);
  else if (TriggerProcessor::TRIGGER_TYPE_EXT2 == trigger_type)
    TU::DigitalInputs::EnableDMARequest(TU::DIGITAL_INPUT_2);
}

/*static*/ void ScopeApp::DrawGraticule(int density)
{
  static constexpr struct {
    uint8_t xaxis;
    uint8_t yaxis;
    uint8_t subdivision_x;
    uint8_t subdivision_y;
  } kGraticuleParameters[kMaxGraticuleDensity] = {
      {0, 0, 0, 0}, {8, 0x01, 0, 0}, {8, 0x01, 16, 0}, {4, 0x11, 8, 0x1}, {2, 0x55, 4, 0x11},
  };

  if (density) {
    auto params = kGraticuleParameters[density];

    graphics.drawHLinePattern(0, 32, 128, params.xaxis);
    graphics.drawVLinePattern(64, 0, 64, params.yaxis);

    if (density > 1) {
      graphics.drawHLinePattern(0, 16, 128, params.subdivision_x);
      graphics.drawHLinePattern(0, 48, 128, params.subdivision_x);

      if (density > 2) {
        graphics.drawVLinePattern(32, 0, 64, params.subdivision_y);
        graphics.drawVLinePattern(96, 0, 64, params.subdivision_y);

        graphics.drawVLinePattern(16, 0, 64, params.subdivision_y);
        graphics.drawVLinePattern(48, 0, 64, params.subdivision_y);
        graphics.drawVLinePattern(80, 0, 64, params.subdivision_y);
        graphics.drawVLinePattern(112, 0, 64, params.subdivision_y);
      }
    }
  }
}

void ScopeApp::Render()  // const
{
  if (ui_.menu_active) {
    RenderMenu();
  } else {
    UpdateDisplayBuffer();
    DrawGraticule(graticule_density());
    RenderDisplayBuffer();
    RenderScopeUI();
  }
}

void ScopeApp::RenderScreensaver()  // const
{
  UpdateDisplayBuffer();
  RenderDisplayBuffer();
}

void ScopeApp::EventScreensaverOff()
{
  ui_.menu_active = false;
  ui_.cursor.set_editing(false);
  ui_.status_bar.show();
}

void ScopeApp::UpdateDisplayBuffer()
{
  auto readable = display_frame_buffer_.readable();
  while (readable > 1) {
    display_frame_buffer_.read();
    --readable;
  }
  if (readable) {
    // TODO If length < kDisplayFrameSize, we could do an overwrite effect instead of copy
    auto frame = display_frame_buffer_.readable_frame();
    current_display_frame_.info = frame->info;
    memcpy(display_buffer_, frame->buffer, frame->info.length * 2);
    display_frame_buffer_.read();
  }
}

void ScopeApp::RenderMenu() const
{
  menu::DefaultTitleBar::Draw();
  graphics.print("SCOPE");

  menu::SettingsList<menu::kScreenLines, 0, menu::kDefaultValueX> settings_list{ui_.cursor};

  menu::SettingsListItem list_item;
  while (settings_list.available()) {
    int setting = settings_list.Next(list_item);
    int value = get_value(setting);
    auto &attr = ScopeApp::value_attr(setting);

    switch (setting) {
      default: list_item.DrawDefault(value, attr); break;
    }
  }
}

static inline weegfx::coord_t to_pixel(int16_t value, const int32_t multiplier,
                                       const weegfx::coord_t y)
{
  // kScalingShift = 10 + /64 = >>16
  auto px = y - signed_multiply_32x16b(multiplier, value);
  CONSTRAIN(px, 0, 63);
  return px;
}

/*static*/ void ScopeApp::DrawWaveform(const int16_t *buffer, size_t length, size_t stride,
                                       int32_t multiplier, const weegfx::coord_t y)
{
  auto end = buffer + length;

  weegfx::coord_t x = 0;
  auto y1 = to_pixel(*buffer, multiplier, y);
  buffer += stride;
  while (buffer < end) {
    auto y2 = to_pixel(*buffer, multiplier, y);
    buffer += stride;
    graphics.drawLine(x, y1, x + stride, y2);
    y1 = y2;
    x += stride;
  }
}

void ScopeApp::RenderDisplayBuffer() const
{
  auto length = current_display_frame_.info.length;

  if (current_display_frame_.info.num_channels > 1) {
    DrawWaveform(current_display_frame_.buffer, length, 2, main_channel().scaling().multiplier,
                 main_channel().screen_yoffset());
    DrawWaveform(current_display_frame_.buffer + 1, length, 2, aux_channel().scaling().multiplier,
                 aux_channel().screen_yoffset());
  } else {
    DrawWaveform(current_display_frame_.buffer, length, 1, main_channel().scaling().multiplier,
                 main_channel().screen_yoffset());
  }
}

namespace icons {
static const uint8_t channel_y_1_7x8[] = {0xff, 0x81, 0x81, 0x89, 0xbd, 0x81, 0xff};
static const uint8_t channel_y_2_7x8[] = {0xff, 0x81, 0xb5, 0xb5, 0xad, 0x81, 0xff};
static const uint8_t channel_y_3_7x8[] = {0xff, 0x81, 0xa5, 0xb5, 0xbd, 0x81, 0xff};
static const uint8_t channel_y_4_7x8[] = {0xff, 0x81, 0x9d, 0x91, 0xbd, 0x81, 0xff};

static const uint8_t trigger_rising_edge_8x8[] = {0x00, 0x80, 0x90, 0x98, 0xff, 0x19, 0x11, 0x00};
static const uint8_t trigger_falling_edge_8x8[] = {0x00, 0x01, 0x09, 0x19, 0xff, 0x98, 0x88, 0x00};
static const uint8_t trigger_ext1_8x8[] = {0x04, 0x7c, 0x04, 0x70, 0x10, 0x00, 0x08, 0x7c};
static const uint8_t trigger_ext2_8x8[] = {0x04, 0x7c, 0x04, 0x70, 0x10, 0x00, 0x74, 0x5c};

static const uint8_t trigger_lost_6x8[] = {0x00, 0x02, 0x01, 0x51, 0x09, 0x06};
static const uint8_t trigger_level_3x8[] = {0x3e, 0x1c, 0x08};

static const uint8_t range_bi_7x8[] = {0xf0, 0x50, 0x30, 0x10, 0x18, 0x14, 0x1e, 0x00};
static const uint8_t range_uni_7x8[] = {0x80, 0xc0, 0xa0, 0x90, 0x88, 0x84, 0xfe, 0x00};

static constexpr const uint8_t *channels[4] = {
    channel_y_1_7x8,
    channel_y_2_7x8,
    channel_y_3_7x8,
    channel_y_4_7x8,
};

static constexpr const uint8_t *trigger_type_icons[TriggerProcessor::TRIGGER_TYPE_LAST] = {
    nullptr, trigger_rising_edge_8x8, trigger_falling_edge_8x8, trigger_ext1_8x8, trigger_ext2_8x8,
};

const uint8_t edit_indicators_8[3 * 3] = {
    0x66, 0xe7, 0x66,  // both
    0x06, 0x07, 0x06,  // min
    0x60, 0xe0, 0x60,  // max
};

const uint8_t unit_ms_8[] = {0x78, 0x18, 0x78, 0x00, 0x58, 0x68};
const uint8_t unit_us_8[] = {0xf8, 0x40, 0x78, 0x00, 0x58, 0x68};
const uint8_t unit_s_8[] = {0x00, 0x48, 0x54, 0x54, 0x54, 0x20};
const uint8_t unit_khz_8[] = {0x00, 0x7c, 0x10, 0x68, 0x00,  // hz ->
                              0x7e, 0x08, 0x08, 0x7e, 0x00, 0x48, 0x68, 0x58};

const uint8_t channel_numbers_7x8[] = {
    0x00, 0x00, 0x42, 0x7F, 0x40, 0x00, 0x00,  // 1
    0x00, 0x42, 0x61, 0x51, 0x49, 0x46, 0x00,  // 2
    0x00, 0x21, 0x41, 0x45, 0x4B, 0x31, 0x00,  // 3
    0x00, 0x18, 0x14, 0x12, 0x7F, 0x10, 0x00,  // 4
};

inline void DrawEditIcon(weegfx::coord_t x, weegfx::coord_t y, int value,
                         const settings::value_attr &attr)
{
  const uint8_t *src = edit_indicators_8;
  if (value == attr.max_)
    src += 3 * 2;
  else if (value == attr.min_)
    src += 3;
  graphics.drawBitmap8(x - 3, y, 3, src);
}

static inline void DrawChannel(weegfx::coord_t x, weegfx::coord_t y, int channel)
{
  graphics.writeBitmap8(x, y, 7, icons::channel_numbers_7x8 + (channel * 7));
}
static inline void DrawLinkedChannels(weegfx::coord_t x, weegfx::coord_t y, int first_channel)
{
  graphics.writeBitmap8(x, y, 14, icons::channel_numbers_7x8 + (first_channel * 7));
  graphics.drawHLinePattern(x, y - 1, 14, 2);
}

};  // namespace icons

void ScopeApp::RenderScopeUI() const
{
  namespace DEBUG = TU::DEBUG;
  // NOTE In linked mode, some values are always from the main channel
  auto &main_ch = main_channel();
  auto &selected_ch = selected_channel();

  // LEFT channel label(s)/"GND" offset indicator
  if (!ui_.status_bar.visible()) {
    weegfx::coord_t x = 3;
    weegfx::coord_t my = main_channel().screen_yoffset() - 7;
    CONSTRAIN(my, 0, 64 - 8);

    if (channel_config_.linked()) {
      weegfx::coord_t y = aux_channel().screen_yoffset() - 7;
      CONSTRAIN(y, 0, 64 - 8);
      graphics.writeBitmap8(y == my ? x + 3 : x, y, 7, icons::channels[channel_config_.aux()]);
    }
    graphics.writeBitmap8(x, my, 7, icons::channels[channel_config_.main()]);
  }

  // LEFT Trigger level
  auto trigger_type = main_ch.trigger_type();
  auto trigger_level_y =
      to_pixel(main_ch.trigger_level(), main_ch.scaling().multiplier, main_ch.screen_yoffset()) - 3;
  if (TriggerProcessor::TRIGGER_TYPE_NONE != trigger_type) {
    auto y = trigger_level_y;
    CONSTRAIN(y, 0, 58);
    graphics.clearRect(0, y, 3, 8);
    graphics.drawBitmap8(0, y, 3, icons::trigger_level_3x8);
  }

  // TOP ... [freq][trigger]
  if (!channel_config_.linked() && display_frequency_counter() &&
      TriggerProcessor::TRIGGER_TYPE_NONE != trigger_type)
    DisplayFrequencyCounter(128 - 18, 0, main_ch.frequency());

  {
    const uint8_t *icon = icons::trigger_type_icons[trigger_type];
    weegfx::coord_t x = 128 - 8;
    if (icon) {
      graphics.writeBitmap8(x, 0, 8, icon);
      x -= 7;
    }
    if (ui_.trigger_lost) { graphics.writeBitmap8(x, 0, 6, icons::trigger_lost_6x8); }
  }

  if (ui_.edit_setting.visible()) {
    // ON SCREEN EDIT OVERLAY (TRIGGER LEVEL)
    const uint8_t *icon = nullptr;
    if (trigger_level_y < 0) {
      trigger_level_y = 0;
      icon = icons::edit_indicators_8 + 3 * 2;
    } else if (trigger_level_y > 57) {
      trigger_level_y = 57;
      icon = icons::edit_indicators_8 + 3;
    }
    graphics.setPrintPos(6, trigger_level_y);
    auto trigger_level = main_ch.trigger_level_displayable();
    char sign = '+';
    if (trigger_level < 0) {
      sign = '-';
      trigger_level = -trigger_level;
    }
    trigger_level += 50;
    auto v = trigger_level / 1000;
    graphics.printf("%c%d.%01d0V", sign, v, (trigger_level - (v * 1000)) / 100);

    if (icon) graphics.drawBitmap8(6 + 36 + 1, trigger_level_y - 1, 3, icon);
  } else if (ui_.status_bar.visible()) {
    // BOTTOM STATUS BAR [CHAN][TIME][????][SCALE]
    DrawStatusBar();

    weegfx::coord_t x = 0;
    weegfx::coord_t bottom_text_y = kStatusBarY;
    bottom_text_y++;

    // [CHAN]
    {
      if (SCOPE_CHANNEL_SETTING_LAST == ui_.edit_setting_l) {
        // Channel selection mode
        if (get_value(SCOPE_APP_SETTING_LINK12)) {
          icons::DrawLinkedChannels(x, bottom_text_y, 0);
        } else {
          icons::DrawChannel(x, bottom_text_y, 0);
          icons::DrawChannel(x + weegfx::kFixedFontW + 1, bottom_text_y, 1);
        }
        x += 2 * (weegfx::kFixedFontW + 1);
        if (get_value(SCOPE_APP_SETTING_LINK34)) {
          icons::DrawLinkedChannels(x, bottom_text_y, 2);
        } else {
          icons::DrawChannel(x, bottom_text_y, 2);
          icons::DrawChannel(x + weegfx::kFixedFontW + 1, bottom_text_y, 3);
        }
      } else {
        // Just display active channel(s)
        x = channel_config_.main() * (weegfx::kFixedFontW + 1);
        if (channel_config_.linked())
          icons::DrawLinkedChannels(x, bottom_text_y, channel_config_.main());
        else
          icons::DrawChannel(x, bottom_text_y, channel_config_.main());
      }
      graphics.invertRect(selected_channel_index() * (weegfx::kFixedFontW + 1), bottom_text_y,
                          weegfx::kFixedFontW + 1, weegfx::kFixedFontH + 1);
    }

    x = 32 + 6;
    if (SCOPE_CHANNEL_SETTING_XDIV == ui_.edit_setting_l)
      icons::DrawEditIcon(x - 1, bottom_text_y - 1, main_ch.xdiv(),
                          main_ch.value_attr(SCOPE_CHANNEL_SETTING_XDIV));
    graphics.setPrintPos(x, bottom_text_y);
    auto label = main_ch.timebase().label;
    graphics.print(label, 3);
    switch (label[3]) {
      case 'm': graphics.drawBitmap8(x + 18 + 1, bottom_text_y, 6, icons::unit_ms_8); break;
      case 'u': graphics.drawBitmap8(x + 18 + 1, bottom_text_y, 6, icons::unit_us_8); break;
      case 's': graphics.drawBitmap8(x + 18 + 1, bottom_text_y, 6, icons::unit_s_8); break;
    }

    x = 96 - 8;
    auto range_icon =
        INPUT_RANGE_BI == selected_ch.input_range() ? icons::range_bi_7x8 : icons::range_uni_7x8;
    graphics.writeBitmap8(x, bottom_text_y - 1, 7, range_icon);

    x = 96 + 6;
    if (SCOPE_CHANNEL_SETTING_YDIV == ui_.edit_setting_r)
      icons::DrawEditIcon(x - 1, bottom_text_y - 1, selected_ch.ydiv(),
                          selected_ch.value_attr(SCOPE_CHANNEL_SETTING_YDIV));
    graphics.setPrintPos(x, bottom_text_y);
    graphics.printf(selected_ch.scaling().label);
  }

  // Info/debug overlay
  auto stats_overlay = display_stats_overlay();
  if (stats_overlay == 1) {
    weegfx::coord_t x = 64 - 48;
    weegfx::coord_t y = 8;
    graphics.setPrintPos(x, y);
    graphics.write(main_ch.stats().trigger_count & 0xffff, 8);

    y += 8;
    graphics.setPrintPos(x, y);
    graphics.write(main_ch.stats().sample_count, 8);

    y += 8;
    graphics.setPrintPos(x, y);
    graphics.write(debug::cycles_to_us(process_cycles.value()), 8);

    y += 8;
    graphics.setPrintPos(x, y);
    graphics.write(main_ch.frequency(), 8);

    y += 8;
    graphics.setPrintPos(x, y);
    graphics.write(debug_.skipped_frames_, 8);
  } else if (stats_overlay == 2) {
    graphics.setPrintPos(128 - 30, 64 - 16);
    graphics.write(debug::cycles_to_us(DEBUG::MENU_draw_cycles.value()), 5);
  }
}

void ScopeApp::DrawStatusBar() const
{
  graphics.clearRect(0, kStatusBarY, 128, 8);
  graphics.drawHLine(0, kStatusBarY - 1, 128);
  graphics.drawAlignedByte(32, kStatusBarY, 0xaa);
  graphics.drawAlignedByte(64, kStatusBarY, 0xaa);
  graphics.drawAlignedByte(96, kStatusBarY, 0xaa);
}

void ScopeApp::DisplayFrequencyCounter(weegfx::coord_t x, weegfx::coord_t y,
                                       uint32_t frequency) const
{
  weegfx::coord_t w = sizeof(icons::unit_khz_8);
  auto unit = icons::unit_khz_8;

  char freq_str[16] = "-";
  if (frequency > 1000) {
    auto khz = frequency / 1000;
    sprintf(freq_str, "%lu.%.02lu", khz, ((frequency - (khz * 1000))) / 10);
  } else if (frequency) {
    sprintf(freq_str, "%lu", frequency);
    unit += 4;
    w -= 4;
  }
  x -= w;
  graphics.setPrintPos(x, y);
  graphics.write_right(freq_str);
  graphics.writeBitmap8(x + 1, y, w, unit);
}

void ScopeApp::Activate()
{
  UpdateChannelConfig();
  ConfigureADC();
  ConfigureTR();
}

static ScopeApp scope_app_instance;

}  // namespace scope

void SCOPE_init()
{
  scope::scope_app_instance.Init();
}

size_t SCOPE_storageSize()
{
  return scope::ScopeApp::storageSize() +
         scope::ScopeApp::kNumChannels * scope::ScopeChannel::storageSize();
}

size_t SCOPE_save(util::StreamBufferWriter &stream)
{
  return scope::scope_app_instance.SaveState(stream);
}

size_t SCOPE_restore(util::StreamBufferReader &stream)
{
  return scope::scope_app_instance.RestoreState(stream);
}

void SCOPE_reset()
{
  scope::scope_app_instance.Init();
}

void SCOPE_handleAppEvent(TU::AppEvent event)
{
  switch (event) {
    case TU::APP_EVENT_RESUME: scope::scope_app_instance.EventScreensaverOff(); break;
    case TU::APP_EVENT_SUSPEND: break;
    case TU::APP_EVENT_SCREENSAVER_ON: break;
    case TU::APP_EVENT_SCREENSAVER_OFF: scope::scope_app_instance.EventScreensaverOff(); break;
    case TU::APP_EVENT_ACTIVATE: scope::scope_app_instance.Activate();
    default: break;
  }
}

void SCOPE_loop()
{
  scope::scope_app_instance.UpdateUI();
}

void SCOPE_menu()
{
  scope::scope_app_instance.Render();
}

void SCOPE_screensaver()
{
  scope::scope_app_instance.RenderScreensaver();
}

void SCOPE_handleButtonEvent(const UI::Event &event)
{
  scope::scope_app_instance.DispatchEvent(event);
}

void SCOPE_handleEncoderEvent(const UI::Event &event)
{
  scope::scope_app_instance.DispatchEvent(event);
}

void SCOPE_isr()
{
  scope::scope_app_instance.Process();
}

