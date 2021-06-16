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
#include "TU_menus.h"
#include "TU_strings.h"
#include "TU_ui.h"
#include "UI/ui_event_dispatcher.h"
#include "arm_math.h"
#include "util/util_circular_sample_buffer.h"
#include "util/util_edge_detector.h"
#include "util/util_popup.h"
#include "util/util_settings.h"

#define SCOPE_DISPLAY_DRAW_CYCLES

// NOTES
// - SIMD processing of buffers?
// - Raw values from ADC are inverted, so we use calibration offset

namespace scope {

namespace menu = TU::menu;

static constexpr weegfx::coord_t kDisplayFrameSize = weegfx::Graphics::kWidth;
static constexpr size_t kADCChunkSize = TU::ADC::kDMAChunkSize;
static constexpr uint32_t kTriggerLostIndicatorTimeoutTicks = TU_CORE_ISR_FREQ / 3;

static debug::AveragedCycles process_cycles;

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
        &TriggerProcessor::Nop<buffer_length>,
        &TriggerProcessor::Nop<buffer_length>,
    };
    return (this->*processors[trigger_type])(threshold, buffer);
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

  using EdgeDetector = util::EdgeDetector<int16_t>;

  EdgeDetector::State edge_detector_state_ = 0;

  template <size_t buffer_length>
  TriggerOffset Nop(int16_t, const int16_t *buffer)
  {
    stats_.sample_count += buffer_length;
    edge_detector_state_ = 0;
    return buffer_length;
  }

  template <size_t buffer_length, int stride, typename cmp>
  TriggerOffset FindEdges(int16_t threshold, const int16_t *buffer)
  {
    auto buf = buffer;
    auto end = buffer + buffer_length;
    auto edge_count = stats_.edge_count;
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

    stats_.sample_count += buffer_length;
    stats_.edge_count = edge_count;
    edge_detector_state_ = edge_detector.state();

    if (first_edge) {
      ++stats_.trigger_count;
      return first_edge - buffer;
    } else {
      return buffer_length;
    }
  }
};

static constexpr const char *kTriggerTypeStrings[TriggerProcessor::TRIGGER_TYPE_LAST] = {
    "none", "rising", "falling", "ext1", "ext2",
};

struct TimebaseParameters {
  const char *const label;
  uint32_t adc_frequency;
  // auto gain?
  // retrigger delay
};

// BEGIN generated via resources/scope_divs.py
// clang-format off
enum Timebase {
  TIMEBASE_2000,
  TIMEBASE_1000,
  TIMEBASE_500,
  TIMEBASE_200,
  TIMEBASE_100,
  TIMEBASE_50,
  TIMEBASE_20,
  TIMEBASE_10,
  TIMEBASE_LAST,
};

static constexpr TimebaseParameters kTimebaseParameters[TIMEBASE_LAST] = {
{ .label = "500u", .adc_frequency = 256000 },
{ .label = "  1m", .adc_frequency = 128000 },
{ .label = "  2m", .adc_frequency = 64000 },
{ .label = "  5m", .adc_frequency = 25600 },
{ .label = " 10m", .adc_frequency = 12800 },
{ .label = " 20m", .adc_frequency = 6400 },
{ .label = " 50m", .adc_frequency = 2560 },
{ .label = "100m", .adc_frequency = 1280 },
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
  SCOPE_CHANNEL_SETTING_XOFF,
  SCOPE_CHANNEL_SETTING_YOFF,
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
  TriggerProcessor::TriggerOffset Process(const int16_t *buffer)
  {
    auto trigger =
        trigger_processor_.Process<buffer_length, stride>(trigger_type(), trigger_level(), buffer);

    auto &stats = trigger_processor_.stats();
    if (stats.sample_count >= timebase().adc_frequency) {
      freq_.push(stats.edge_count);
      trigger_processor_.ResetEdgeCounter();
    }

    return trigger;
  }

  const TriggerProcessor::Stats stats() const { return trigger_processor_.stats(); }
  uint32_t frequency() const { return freq_.value(); }

  // Settings getters
  int xdiv() const { return get_value(SCOPE_CHANNEL_SETTING_XDIV); }
  int xoffset() const { return get_value(SCOPE_CHANNEL_SETTING_XOFF); }
  int ydiv() const { return get_value(SCOPE_CHANNEL_SETTING_YDIV); }
  int yoffset() const { return get_value(SCOPE_CHANNEL_SETTING_YOFF); }

  TriggerProcessor::TriggerType trigger_type() const
  {
    return static_cast<TriggerProcessor::TriggerType>(get_value(SCOPE_CHANNEL_SETTING_TRIG_TYPE));
  }

  int16_t trigger_level() const
  {
    return static_cast<int16_t>(get_value(SCOPE_CHANNEL_SETTING_TRIG_LEVEL));
  }

  int trigger_holdoff() const { return get_value(SCOPE_CHANNEL_SETTING_TRIG_HOLDOFF); }

  const TimebaseParameters &timebase() const { return kTimebaseParameters[xdiv()]; }
  const ScalingParameters &scaling() const { return kScalingParameters[ydiv()]; }

  // UI helpers
  void UpdateEnabledSettings();

private:
  TriggerProcessor trigger_processor_;

  RunningAverage<uint32_t, 4> freq_;
};

SETTINGS_DECLARE(scope::ScopeChannel, scope::SCOPE_CHANNEL_SETTING_LAST){
    // default, min, max, name, value_names, storage_type, parent_index, parent_valuea
    {0, 0, 127, "XOFF", nullptr, settings::STORAGE_TYPE_I16},
    {0, -32, 32, "YOFF", nullptr, settings::STORAGE_TYPE_I16},
    {1, 0, scope::TIMEBASE_LAST - 1, "XDIV", nullptr, settings::STORAGE_TYPE_U8},
    {scope::DIV_5V, scope::DIV_0V5, scope::DIV_LAST - 1, "YDIV", nullptr,
     settings::STORAGE_TYPE_U8},
    {scope::TriggerProcessor::TRIGGER_TYPE_RISING, scope::TriggerProcessor::TRIGGER_TYPE_NONE,
     scope::TriggerProcessor::TRIGGER_TYPE_LAST - 1, "TRIG TYPE", scope::kTriggerTypeStrings,
     settings::STORAGE_TYPE_U8},
    {32, -2048, 2047, "TRIG LVL", nullptr, settings::STORAGE_TYPE_I16},
    {8, 0, 64, "TRIG HOLD", nullptr, settings::STORAGE_TYPE_U8},
};

void ScopeChannel::Init()
{
  InitDefaults();
  UpdateEnabledSettings();
}

void ScopeChannel::UpdateEnabledSettings()
{
  enabled_settings_reset();
  enabled_settings_add(SCOPE_CHANNEL_SETTING_TRIG_TYPE);
  switch (trigger_type()) {
    case TriggerProcessor::TRIGGER_TYPE_NONE:
    case TriggerProcessor::TRIGGER_TYPE_EXT1:
    case TriggerProcessor::TRIGGER_TYPE_EXT2: break;
    default: enabled_settings_add(SCOPE_CHANNEL_SETTING_TRIG_LEVEL);
  }
  enabled_settings_add(SCOPE_CHANNEL_SETTING_XOFF);
  enabled_settings_add(SCOPE_CHANNEL_SETTING_YOFF);
  enabled_settings_add(SCOPE_CHANNEL_SETTING_XDIV);
  enabled_settings_add(SCOPE_CHANNEL_SETTING_YDIV);
}

enum ScopeAppSetting {
  SCOPE_APP_SETTING_CHANNEL,
  SCOPE_APP_SETTING_FREQ,
  SCOPE_APP_SETTING_LINK12,
  SCOPE_APP_SETTING_LINK34,
  SCOPE_APP_SETTING_RESET,  // dummy
  SCOPE_APP_SETTING_LAST,
  SCOPE_APP_SETTING_FIRST = SCOPE_APP_SETTING_CHANNEL
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

private:
  struct {
    bool menu_active = false;

    ScopeChannelSetting edit_setting_l = SCOPE_CHANNEL_SETTING_XDIV;
    ScopeChannelSetting edit_setting_r = SCOPE_CHANNEL_SETTING_YDIV;

    volatile uint32_t trigger_lost = 0;

    util::PopupElement edit_setting;
    util::PopupElement status_bar;
    util::PopupElement info_overlay;

    menu::ScreenCursor<menu::kScreenLines> cursor;
  } ui_;

  struct {
    bool triggered = false;
    int holdoff = 0;
  } trigger_state_;

  struct ChannelConfig {
    constexpr ChannelConfig() : packed_value{Pack(ADC_CHANNEL_1, ADC_CHANNEL_LAST)} {}
    ChannelConfig(ADC_CHANNEL main) : packed_value{Pack(main, ADC_CHANNEL_LAST)} {}
    ChannelConfig(ADC_CHANNEL main, ADC_CHANNEL aux) : packed_value{Pack(main, aux)} {}

    ADC_CHANNEL main_adc_channel() const { return static_cast<ADC_CHANNEL>(packed_value & 0xffff); }
    ADC_CHANNEL aux_adc_channel() const { return static_cast<ADC_CHANNEL>((packed_value >> 16)); }

    int packed_value = Pack(ADC_CHANNEL_1, ADC_CHANNEL_LAST);

    static constexpr int Pack(int main, int aux) { return main | (aux << 16); }
  };

  ChannelConfig channel_config_ = {};

  struct FrameInfo {
    int num_channels = 1;
    size_t read_length = 0;
  };

  using CircularSampleBuffer = util::CircularSampleBuffer<int16_t, kADCChunkSize * 4>;
  using DisplayFrameBuffer = util::FrameBuffer<kDisplayFrameSize, 2, int16_t, FrameInfo>;

  const DisplayFrameBuffer::Frame *current_display_frame_ = nullptr;
  ScopeChannel channels_[kNumChannels];

  static CircularSampleBuffer sample_buffer_;
  static DisplayFrameBuffer display_frame_buffer_;

  const ScopeChannel &main_channel() const { return channels_[channel_config_.main_adc_channel()]; }
  ScopeChannel &main_channel() { return channels_[channel_config_.main_adc_channel()]; }

  const ScopeChannel &aux_channel() const { return channels_[channel_config_.aux_adc_channel()]; }

  int selected_channel_index() const { return get_value(SCOPE_APP_SETTING_CHANNEL); }
  ScopeChannel &selected_channel() { return channels_[get_value(SCOPE_APP_SETTING_CHANNEL)]; }
  const ScopeChannel &selected_channel() const
  {
    return channels_[get_value(SCOPE_APP_SETTING_CHANNEL)];
  }

  void UpdateChannelConfig();
  void ConfigureADC();

  static void DrawGrid();
  static void DrawWaveform(const int16_t *buffer, int stride, int32_t multiplier);
  void RenderDisplayBuffer() const;
  void RenderMenu() const;
  void RenderScopeUI() const;

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
  EVENT_DISPATCH_DECLARE_HANDLER(scopeInfoOverlay);
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

SETTINGS_DECLARE(scope::ScopeApp, scope::SCOPE_APP_SETTING_LAST){
    // default, min, max, name, value_names, storage_type, parent_index, parent_value
    {0, 0, scope::ScopeApp::kNumChannels - 1, "CHANNEL", nullptr, settings::STORAGE_TYPE_U8},
    {1, 0, 1, "Disp freq", TU::Strings::no_yes, settings::STORAGE_TYPE_U4},
    {0, 0, 1, "Link 1+2", TU::Strings::no_yes, settings::STORAGE_TYPE_U4},
    {0, 0, 1, "Link 3+4", TU::Strings::no_yes, settings::STORAGE_TYPE_U4},
    {0, 0, 1, "Reset", nullptr, settings::STORAGE_TYPE_NOP},
};

void ScopeApp::Init()
{
  InitDefaults();
  for (auto &channel : channels_) channel.Init();
  UpdateChannelConfig();

  display_frame_buffer_.Init();
  ui_.cursor.Init(SCOPE_APP_SETTING_FIRST, SCOPE_APP_SETTING_LAST - 1);
}

void ScopeApp::Process()
{
  // const auto channel_config = channel_config_;
  uint32_t trigger_lost = ui_.trigger_lost;

  auto &adc_chunks = TU::ADC::chunk_buffers();
  while (adc_chunks.readable()) {
    debug::ScopedCycleMeasurement cycles{process_cycles};

    auto adc_chunk_buffer = adc_chunks.readable_frame();

    // Process input buffer; if already triggered we don't really need to find a new one yet but
    // this might handle more than just triggers eventually
    auto trigger = main_channel().Process<kADCChunkSize, 1>(adc_chunk_buffer->buffer);

    auto sample_writer = sample_buffer_.writer();
#if 0
    // Decimate data into circular sample buffer/history
    util::SampleDecimator::Process<kADCChunkSize>(sample_writer, adc_chunk_buffer_, 1);
    // TODO account for decimation during further operations
    // It might also make more sense to decimate when _reading_ from the sample buffer (although this means increasing the size)
#else
    // TODO without decimation, this step is somewhat moot
    for (auto src = adc_chunk_buffer->buffer, end = src + kADCChunkSize; src < end; ++src)
      *sample_writer++ = *src;
#endif
    adc_chunks.read();
    sample_writer.Commit();

    // Trigger/display buffer handling
    size_t read_length = 0;
    if (trigger_state_.triggered) {
      if (sample_buffer_.available() < kDisplayFrameSize) {
        // still accumulating (doesn't happen, since we got more data to get here)
      } else {
        // buffer full, rearm and start again
        trigger_state_.triggered = false;
        trigger_state_.holdoff = main_channel().trigger_holdoff();
        read_length = kDisplayFrameSize;
      }
    } else {
      if (trigger_state_.holdoff) {
        --trigger_state_.holdoff;
      } else {
        if (trigger < kADCChunkSize) {
          trigger_state_.triggered = true;
          auto n = kADCChunkSize - trigger;
          sample_buffer_.SetReadOffset(-n /* - kDisplayFrameSize / 2*/);
        } else {
          trigger_lost = kTriggerLostIndicatorTimeoutTicks;
          sample_buffer_.SetReadOffset(-kDisplayFrameSize);
          read_length = kDisplayFrameSize;
        }
      }
    }

    if (read_length && display_frame_buffer_.writeable()) {
      auto frame = display_frame_buffer_.writeable_frame();
      sample_buffer_.Read(frame->buffer, read_length);
      frame->info.read_length = read_length;
      display_frame_buffer_.written();
    }
  }

  // Other regular book-keeping?
  if (trigger_lost) --trigger_lost;
  ui_.trigger_lost = trigger_lost;
}

void ScopeApp::UpdateUI()
{
  auto ticks = TU::ui.ticks();
  ui_.edit_setting.Tick(ticks);
  ui_.status_bar.Tick(ticks);
  ui_.info_overlay.Tick(ticks);
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
    {UI::EVENT_BUTTON_PRESS, TU::CONTROL_BUTTON_L, &ScopeApp::menuButtonL},
    {UI::EVENT_BUTTON_PRESS, TU::CONTROL_BUTTON_R, &ScopeApp::menuButtonR},
    {UI::EVENT_ENCODER, TU::CONTROL_ENCODER_L, &ScopeApp::menuEncoderL},
    {UI::EVENT_ENCODER, TU::CONTROL_ENCODER_R, &ScopeApp::menuEncoderR},
    {},
};

/*static*/ const ScopeApp::EventHandler ScopeApp::scope_button_handlers[] = {
    {UI::EVENT_BUTTON_PRESS, TU::CONTROL_BUTTON_UP, &ScopeApp::toggleMenu},
    {UI::EVENT_BUTTON_PRESS, TU::CONTROL_BUTTON_DOWN, &ScopeApp::scopeButtonDown},
    {UI::EVENT_BUTTON_PRESS, TU::CONTROL_BUTTON_L, &ScopeApp::scopeButtonL},
    {UI::EVENT_BUTTON_PRESS, TU::CONTROL_BUTTON_R, &ScopeApp::scopeButtonR},
    {UI::EVENT_BUTTON_LONG_PRESS, TU::CONTROL_BUTTON_DOWN, &ScopeApp::scopeInfoOverlay},
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
    // ui_.cursor.AdjustEnd(current_channel().num_enabled_settings());
  }
}

EVENT_DISPATCH_DEFINE_HANDLER(ScopeApp, scopeInfoOverlay)
{
  EVENT_DISPATCH_HANDLER_STUB();

  ui_.info_overlay.show();
}

EVENT_DISPATCH_DEFINE_HANDLER(ScopeApp, scopeButtonDown)
{
  EVENT_DISPATCH_HANDLER_STUB();

  auto &channel = selected_channel();
  channel.change_value_wrap(SCOPE_CHANNEL_SETTING_TRIG_TYPE, 1);
  channel.UpdateEnabledSettings();
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

  auto &channel = selected_channel();
  bool update_adc = false;
  switch (ui_.edit_setting_l) {
    case SCOPE_CHANNEL_SETTING_XDIV:
      update_adc = channel.change_value(SCOPE_CHANNEL_SETTING_XDIV, event_value);
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
  if (update_adc) ConfigureADC();
}

EVENT_DISPATCH_DEFINE_HANDLER(ScopeApp, scopeEncoderR)
{
  EVENT_DISPATCH_HANDLER_STUB();

  auto &channel = selected_channel();
  switch (ui_.edit_setting_r) {
    case SCOPE_CHANNEL_SETTING_TRIG_LEVEL:
      channel.change_value(SCOPE_CHANNEL_SETTING_TRIG_LEVEL, event_value * 32);
      ui_.edit_setting.show();
      ui_.status_bar.hide();
      break;
    case SCOPE_CHANNEL_SETTING_YDIV: channel.change_value(SCOPE_CHANNEL_SETTING_YDIV, event_value);
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
          break;
        default: break;
      }
    }
  }
}

void ScopeApp::UpdateChannelConfig()
{
  ADC_CHANNEL main = static_cast<ADC_CHANNEL>(get_value(SCOPE_APP_SETTING_CHANNEL));
  ADC_CHANNEL aux;
  if (ADC_CHANNEL_1 == main && get_value(SCOPE_APP_SETTING_LINK12))
    aux = ADC_CHANNEL_2;
  else if (ADC_CHANNEL_3 == main && get_value(SCOPE_APP_SETTING_LINK34))
    aux = ADC_CHANNEL_4;
  else
    aux = ADC_CHANNEL_LAST;

  channel_config_ = {main, aux};
}

void ScopeApp::ConfigureADC()
{
  TU::ADC::StartConversionBuffered(main_channel().timebase().adc_frequency,
                                   channel_config_.main_adc_channel(),
                                   channel_config_.aux_adc_channel());
}

/*static*/ void ScopeApp::DrawGrid()
{
  graphics.drawVLinePattern(16, 0, 64, 0x88);
  graphics.drawVLinePattern(32, 0, 64, 0x88);
  graphics.drawVLinePattern(48, 0, 64, 0x88);
  graphics.drawVLinePattern(64, 0, 64, 0xaa);
  graphics.drawVLinePattern(80, 0, 64, 0x88);
  graphics.drawVLinePattern(96, 0, 64, 0x88);
  graphics.drawVLinePattern(112, 0, 64, 0x88);

  graphics.drawHLinePattern(0, 16, 128, 4);
  graphics.drawHLinePattern(0, 32, 128, 2);
  graphics.drawHLinePattern(0, 48, 128, 4);
}

void ScopeApp::Render()  // const
{
  if (ui_.menu_active) {
    RenderMenu();
  } else {
    UpdateDisplayBuffer();
    DrawGrid();
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
  if (display_frame_buffer_.readable()) {
    if (current_display_frame_) display_frame_buffer_.read();
    current_display_frame_ = display_frame_buffer_.readable_frame();
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

static inline weegfx::coord_t to_pixel(int16_t value, const int32_t multiplier)
{
  // kScalingShift = 10 + /64 = >>16
  auto px = 32 - signed_multiply_32x16b(multiplier, value);
  CONSTRAIN(px, 0, 63);
  return px;
}

/*static*/ void ScopeApp::DrawWaveform(const int16_t *buffer, int stride, int32_t multiplier)
{
  auto end = buffer + kDisplayFrameSize;

  weegfx::coord_t x = 0;
  auto y1 = to_pixel(*buffer, multiplier);
  buffer += stride;
  while (buffer < end) {
    auto y2 = to_pixel(*buffer, multiplier);
    buffer += stride;
    graphics.drawLine(x, y1, x + stride, y2);
    y1 = y2;
    x += stride;
  }
}

void ScopeApp::RenderDisplayBuffer() const
{
  auto frame = current_display_frame_;
  if (!frame) return;

  if (ADC_CHANNEL_LAST != channel_config_.aux_adc_channel()) {
    DrawWaveform(frame->buffer, 2, main_channel().scaling().multiplier);
    DrawWaveform(frame->buffer + 1, 2, aux_channel().scaling().multiplier);
  } else {
    DrawWaveform(frame->buffer, 1, main_channel().scaling().multiplier);
  }
}

namespace icons {
static const uint8_t channel_1_7x8[] = {0xff, 0x81, 0x81, 0x89, 0xbd, 0x81, 0xff};
static const uint8_t channel_2_7x8[] = {0xff, 0x81, 0xb5, 0xb5, 0xad, 0x81, 0xff};
static const uint8_t channel_3_7x8[] = {0xff, 0x81, 0xa5, 0xb5, 0xbd, 0x81, 0xff};
static const uint8_t channel_4_7x8[] = {0xff, 0x81, 0x9d, 0x91, 0xbd, 0x81, 0xff};

static const uint8_t trigger_rising_edge_8x8[] = {0x00, 0x80, 0x90, 0x98, 0xff, 0x19, 0x11, 0x00};
static const uint8_t trigger_falling_edge_8x8[] = {0x00, 0x01, 0x09, 0x19, 0xff, 0x98, 0x88, 0x00};
static const uint8_t trigger_ext1_8x8[] = {0x04, 0x7c, 0x04, 0x70, 0x10, 0x00, 0x08, 0x7c};
static const uint8_t trigger_ext2_8x8[] = {0x04, 0x7c, 0x04, 0x70, 0x10, 0x00, 0x74, 0x5c};

static const uint8_t trigger_lost_6x8[] = {0x00, 0x02, 0x01, 0x51, 0x09, 0x06};

static const uint8_t trigger_level_3x8[] = {0x3e, 0x1c, 0x08};

static constexpr const uint8_t *channels[4] = {
    channel_1_7x8,
    channel_2_7x8,
    channel_3_7x8,
    channel_4_7x8,
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

};  // namespace icons

void ScopeApp::RenderScopeUI() const
{
  namespace DEBUG = TU::DEBUG;
  auto channel_index = selected_channel_index();
  auto &channel = selected_channel();

  // Top [channel] .... [freq][trigger]
  if (!ui_.status_bar.visible()) {
    weegfx::coord_t y = channel.yoffset();
    CONSTRAIN(y, 0, 8);
    graphics.writeBitmap8(3, y, 7, icons::channels[channel_index]);
  }
  if (get_value(SCOPE_APP_SETTING_FREQ)) {
    graphics.setPrintPos(128 - weegfx::kFixedFontW * 5 - 18, 0);
    auto freq = channel.frequency();
    if (freq)
      graphics.write(channel.frequency(), 5);
    else
      graphics.print("-----");
  }

  const uint8_t *icon = icons::trigger_type_icons[channel.trigger_type()];
  weegfx::coord_t x = 128 - 8;
  if (icon) {
    graphics.writeBitmap8(x, 0, 8, icon);
    x -= 7;
  }
  if (ui_.trigger_lost) { graphics.writeBitmap8(x, 0, 6, icons::trigger_lost_6x8); }

  // Left: Trigger level
  auto trigger_level_y = to_pixel(channel.trigger_level(), channel.scaling().multiplier) - 3;
  if (TriggerProcessor::TRIGGER_TYPE_NONE != channel.trigger_type()) {
    CONSTRAIN(trigger_level_y, 0, 58);
    graphics.writeBitmap8(0, trigger_level_y, 3, icons::trigger_level_3x8);
  }

  if (ui_.edit_setting.visible()) {
    // On-screen edit overlay?
    CONSTRAIN(trigger_level_y, 0, 56);
    graphics.setPrintPos(6, trigger_level_y);
    graphics.pretty_print(channel.trigger_level(), 5);
  } else if (ui_.status_bar.visible()) {
    // Bottom [channel][?????][timebase][scale]
    weegfx::coord_t bottom_text_y = 64 - weegfx::kFixedFontH;

    graphics.clearRect(0, bottom_text_y, 128, 8);
    graphics.drawHLine(0, bottom_text_y - 1, 128);
    graphics.drawAlignedByte(32, bottom_text_y, 0xaa);
    graphics.drawAlignedByte(64, bottom_text_y, 0xaa);
    graphics.drawAlignedByte(96, bottom_text_y, 0xaa);

    bool edit_channel = SCOPE_CHANNEL_SETTING_LAST == ui_.edit_setting_l;

    bottom_text_y++;
    if (edit_channel) {
      weegfx::coord_t x = 0;
      for (int c = 0; c < kNumChannels; ++c) {
        graphics.setPrintPos(x, bottom_text_y);
        graphics.print((char)('1' + c));
        x += weegfx::kFixedFontW + 1;
      }
      graphics.invertRect(channel_index * (weegfx::kFixedFontW + 1), bottom_text_y,
                          weegfx::kFixedFontW + 1, weegfx::kFixedFontH + 1);
    } else {
      graphics.setPrintPos(channel_index * (weegfx::kFixedFontW + 1), bottom_text_y);
      graphics.print((char)('1' + channel_index));
    }

    x = 32 + 6;
    if (!edit_channel)
      icons::DrawEditIcon(x - 1, bottom_text_y - 1, channel.xdiv(),
                          channel.value_attr(SCOPE_CHANNEL_SETTING_XDIV));
    graphics.setPrintPos(x, bottom_text_y);
    auto label = channel.timebase().label;
    graphics.print(label, 3);
    switch (label[3]) {
      case 'm': graphics.drawBitmap8(x + 18 + 1, bottom_text_y, 6, icons::unit_ms_8); break;
      case 'u': graphics.drawBitmap8(x + 18 + 1, bottom_text_y, 6, icons::unit_us_8); break;
    }

    x = 96 + 6;
    if (SCOPE_CHANNEL_SETTING_YDIV == ui_.edit_setting_r)
      icons::DrawEditIcon(x - 1, bottom_text_y - 1, channel.ydiv(),
                          channel.value_attr(SCOPE_CHANNEL_SETTING_YDIV));
    graphics.setPrintPos(x, bottom_text_y);
    graphics.printf(channel.scaling().label);
  }

  // Info/debug overlay
  if (ui_.info_overlay.visible()) {
    weegfx::coord_t x = 48;
    weegfx::coord_t y = 8;
    graphics.setPrintPos(x, y);
    graphics.write(channel.stats().trigger_count & 0xffff, 8);

    graphics.movePrintPos(0, 8);
    graphics.write(channel.stats().sample_count, 8);

    graphics.movePrintPos(0, 8);
    graphics.write(debug::cycles_to_us(process_cycles.value()), 8);
  }

#ifdef SCOPE_DISPLAY_DRAW_CYCLES
  graphics.setPrintPos(128 - 30, 64 - 16);
  graphics.write(debug::cycles_to_us(DEBUG::MENU_draw_cycles.value()), 5);
#endif
}

void ScopeApp::Activate()
{
  ConfigureADC();
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

