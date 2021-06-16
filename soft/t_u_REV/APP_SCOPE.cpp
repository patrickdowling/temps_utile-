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
#include "TU_ui.h"
#include "UI/ui_event_dispatcher.h"
#include "arm_math.h"
#include "util/util_circular_sample_buffer.h"
#include "util/util_popup.h"
#include "util/util_settings.h"

// NOTES
// - SIMD processing of buffers?
// - Raw values from ADC are inverted, so we use calibration offset

namespace scope {

namespace menu = TU::menu;

static constexpr weegfx::coord_t kDisplayBufferSize = weegfx::Graphics::kWidth;
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

  template <size_t buffer_length>
  static const int16_t *Process(TriggerType trigger_type, int16_t threshold, const int16_t *buffer)
  {
    using Impl = const int16_t *(*)(int16_t, const int16_t *);
    static constexpr Impl processors[TRIGGER_TYPE_LAST] = {
        Nop<buffer_length>,
        FindEdge<buffer_length, std::greater<int16_t>>,  // rising
        FindEdge<buffer_length, std::less<int16_t>>,     // falling
        Nop<buffer_length>,
        Nop<buffer_length>,
    };
    return processors[trigger_type](threshold, buffer);
  }

private:
  template <size_t buffer_length>
  static const int16_t *Nop(int16_t, const int16_t *buffer)
  {
    return nullptr;
  }

  template <size_t buffer_length, typename cmp>
  static const int16_t *FindEdge(int16_t threshold, const int16_t *buffer)
  {
    auto end = buffer + buffer_length;
    // ignore starting values that match
    while (buffer < end && cmp{}(buffer[0], threshold)) ++buffer;
    // find first value that matches
    while (buffer < end) {
      if (cmp{}(buffer[0], threshold)) return buffer;
      ++buffer;
    }
    return nullptr;
  }
};

static constexpr const char *kTriggerTypeStrings[TriggerProcessor::TRIGGER_TYPE_LAST] = {
    "none", "rising", "falling", "ext1", "ext2",
};

enum Timebase {
  TIMEBASE_100,
  TIMEBASE_200,
  TIMEBASE_500,
  TIMEBASE_1000,
  TIMEBASE_2000,
  TIMEBASE_LAST
};

struct TimebaseParameters {
  const char *const label;
  uint32_t adc_frequency;
  // auto gain?
  // retrigger delay
};

static constexpr TimebaseParameters kTimebaseParameters[TIMEBASE_LAST] = {
    {" 100", .adc_frequency = 100 * 128},  {" 200", .adc_frequency = 200 * 128},
    {" 500", .adc_frequency = 500 * 128},  {"1000", .adc_frequency = 1000 * 128},
    {"2000", .adc_frequency = 2000 * 128},
};

enum Scaling {
  DIV_0V5,
  DIV_1V,
  DIV_2V,
  DIV_5V,
  DIV_10V,
  DIV_LAST,
};

static constexpr int32_t kScalingShift = 8;
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
  SCOPE_CHANNEL_SETTING_LAST,
  SCOPE_CHANNEL_SETTING_FIRST = SCOPE_CHANNEL_SETTING_XOFF,
};

class ScopeChannel : public settings::SettingsBase<ScopeChannel, SCOPE_CHANNEL_SETTING_LAST> {
public:
  void Init();

  template <typename T>
  const int16_t *Process(const T &sample_buffer)
  {
    auto head = sample_buffer.head_buffer();
    auto trigger = TriggerProcessor::Process<T::kChunkSize>(trigger_type(), trigger_level(), head);
    if (trigger) ++trigger_count_;
    return trigger;
  }

  uint32_t trigger_count() const { return trigger_count_; }

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

  const TimebaseParameters &timebase() const { return kTimebaseParameters[xdiv()]; }
  const ScalingParameters &scaling() const { return kScalingParameters[ydiv()]; }

  // UI helpers
  void UpdateEnabledSettings();
  int num_enabled_settings() const { return num_enabled_settings_; }
  int enabled_setting_at(int index) const { return enabled_settings_[index]; }

private:
  uint32_t trigger_count_{0};

  int num_enabled_settings_{0};
  ScopeChannelSetting enabled_settings_[SCOPE_CHANNEL_SETTING_LAST];
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
};

void ScopeChannel::Init()
{
  InitDefaults();
  UpdateEnabledSettings();
}

void ScopeChannel::UpdateEnabledSettings()
{
  auto settings = enabled_settings_;

  *settings++ = SCOPE_CHANNEL_SETTING_TRIG_TYPE;
  switch (trigger_type()) {
    case TriggerProcessor::TRIGGER_TYPE_NONE:
    case TriggerProcessor::TRIGGER_TYPE_EXT1:
    case TriggerProcessor::TRIGGER_TYPE_EXT2: break;
    default: *settings++ = SCOPE_CHANNEL_SETTING_TRIG_LEVEL;
  }
  *settings++ = SCOPE_CHANNEL_SETTING_XOFF;
  *settings++ = SCOPE_CHANNEL_SETTING_YOFF;
  *settings++ = SCOPE_CHANNEL_SETTING_XDIV;
  *settings++ = SCOPE_CHANNEL_SETTING_YDIV;

  num_enabled_settings_ = settings - enabled_settings_;
}

class ScopeApp : public UI::EventDispatcher<ScopeApp> {
public:
  static constexpr int kNumChannels = 4;

  void Init();
  void Process();
  void UpdateUI();

  size_t Save(util::StreamBufferWriter &stream_buffer) const;
  size_t Restore(util::StreamBufferReader &stream_buffer);

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

  using CircularSampleBuffer = util::CircularSampleBuffer<int16_t, kADCChunkSize, 4>;
  using DisplayBuffers = FrameBuffer<kDisplayBufferSize, 2, int16_t>;

  int current_channel_{0};
  const int16_t *current_display_buffer_ = nullptr;

  static uint16_t adc_chunk_buffer_[kADCChunkSize];
  static CircularSampleBuffer sample_buffer_;
  static DisplayBuffers display_buffers_;

  ScopeChannel channels_[kNumChannels];

  ADC_CHANNEL current_adc_channel() const { return static_cast<ADC_CHANNEL>(current_channel_); }

  ScopeChannel &current_channel() { return channels_[current_channel_]; }
  const ScopeChannel &current_channel() const { return channels_[current_channel_]; }

  static void RenderGrid();
  static void RenderDisplayBuffer(const int16_t *display_buffer, const int32_t multiplier);
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

/*static*/ uint16_t ScopeApp::adc_chunk_buffer_[kADCChunkSize] __attribute__((aligned(4)));
/*static*/ ScopeApp::CircularSampleBuffer ScopeApp::sample_buffer_ __attribute__((aligned(4)));
/*static*/ ScopeApp::DisplayBuffers ScopeApp::display_buffers_ __attribute__((aligned(4)));

void ScopeApp::Init()
{
  for (auto &channel : channels_) channel.Init();
  display_buffers_.Init();

  ui_.cursor.Init(SCOPE_CHANNEL_SETTING_FIRST, SCOPE_CHANNEL_SETTING_LAST - 1);
  ui_.cursor.AdjustEnd(current_channel().num_enabled_settings() - 1);
}

void ScopeApp::Process()
{
  uint32_t trigger_lost = ui_.trigger_lost;
  if (TU::ADC::ReadChunk(adc_chunk_buffer_)) {
    debug::ScopedCycleMeasurement cycles{process_cycles};

    // Pre-process raw samples
    auto tail = sample_buffer_.tail_buffer();
    const auto offset =
        TU::ADC::channel_offset(current_adc_channel()) + current_channel().yoffset();
#if 1
    // Unnecessary premature optimization
    auto dst = tail;
    auto src = adc_chunk_buffer_;
    auto end = adc_chunk_buffer_ + kADCChunkSize;
    uint32_t offs = __PKHBT(offset, offset, 16);
    while (src < end) {
      *(uint32_t *)dst = __SSUB16(offs, *(uint32_t *)src);
      src += 2;
      dst += 2;
    }
#else
    std::transform(adc_chunk_buffer_, adc_chunk_buffer_ + kADCChunkSize, tail,
                   [offset](uint16_t raw) -> int16_t { return offset - raw; });
#endif
    sample_buffer_.advance();

    auto trigger = current_channel().Process(sample_buffer_);
    size_t trigger_offset;
    if (!trigger) {
      trigger_lost = kTriggerLostIndicatorTimeoutTicks;
      trigger_offset = 0;
    } else {
      // trigger_lost = 0;
      trigger_offset = trigger - sample_buffer_.head_buffer();
    }

    if (display_buffers_.writeable()) {
      sample_buffer_.ReadHead(display_buffers_.writeable_frame(),
                              trigger_offset - kDisplayBufferSize / 2, kDisplayBufferSize);
      display_buffers_.written();
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

size_t ScopeApp::Save(util::StreamBufferWriter &stream_buffer) const
{
  stream_buffer.Write(current_channel_);
  for (auto &channel : channels_) channel.Save(stream_buffer);

  return stream_buffer.overflow() ? 0 : stream_buffer.written();
}

size_t ScopeApp::Restore(util::StreamBufferReader &stream_buffer)
{
  stream_buffer.Read(current_channel_);
  for (auto &channel : channels_) channel.Restore(stream_buffer);

  return stream_buffer.underflow() ? 0 : stream_buffer.read();
}

/*static*/ const ScopeApp::EventHandler ScopeApp::menu_event_handlers[] = {
    {UI::EVENT_BUTTON_PRESS, TU::CONTROL_BUTTON_UP, &ScopeApp::toggleMenu},
    {UI::EVENT_BUTTON_PRESS, TU::CONTROL_BUTTON_R, &ScopeApp::menuButtonL},
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
    ui_.cursor.AdjustEnd(current_channel().num_enabled_settings());
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

  auto &channel = current_channel();
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
      TriggerProcessor::TRIGGER_TYPE_NONE != current_channel().trigger_type()) {
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

  auto &channel = current_channel();
  bool update_adc = false;
  switch (ui_.edit_setting_l) {
    case SCOPE_CHANNEL_SETTING_XDIV:
      update_adc = channel.change_value(SCOPE_CHANNEL_SETTING_XDIV, event_value);
      break;
    case SCOPE_CHANNEL_SETTING_LAST: {
      auto channel = current_channel_ + event_value;
      CONSTRAIN(channel, 0, kNumChannels - 1);
      if (channel != current_channel_) {
        current_channel_ = channel;
        update_adc = true;
      }
    } break;
    default: break;
  }
  ui_.edit_setting.hide();
  ui_.status_bar.show();
  if (update_adc)
    TU::ADC::StartConversionBuffered(channel.timebase().adc_frequency, current_adc_channel());
}

EVENT_DISPATCH_DEFINE_HANDLER(ScopeApp, scopeEncoderR)
{
  EVENT_DISPATCH_HANDLER_STUB();

  auto &channel = current_channel();
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

  ui_.cursor.toggle_editing();
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
    auto &channel = current_channel();
    auto selected = channel.enabled_setting_at(ui_.cursor.cursor_pos());
    if (channel.change_value(selected, event_value)) {
      channel.UpdateEnabledSettings();
      ui_.cursor.AdjustEnd(channel.num_enabled_settings());
    }
  }
}

/*static*/ void ScopeApp::RenderGrid()
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
    RenderGrid();
    auto display_buffer = current_display_buffer_;
    if (display_buffer) RenderDisplayBuffer(display_buffer, current_channel().scaling().multiplier);
    RenderScopeUI();
  }
}

void ScopeApp::RenderScreensaver()  // const
{
  UpdateDisplayBuffer();
  auto display_buffer = current_display_buffer_;
  if (display_buffer) RenderDisplayBuffer(display_buffer, current_channel().scaling().multiplier);
}

void ScopeApp::EventScreensaverOff()
{
  ui_.menu_active = false;
  ui_.cursor.set_editing(false);
  ui_.status_bar.show();
}

void ScopeApp::UpdateDisplayBuffer()
{
  if (display_buffers_.readable()) {
    if (current_display_buffer_) display_buffers_.read();
    current_display_buffer_ = display_buffers_.readable_frame();
  }
}

void ScopeApp::RenderMenu() const
{
  menu::QuadTitleBar::Draw(true);
  for (int i = 0; i < 4; ++i) {
    menu::QuadTitleBar::SetColumn(i);
    graphics.print((char)('1' + i));
  }
  menu::QuadTitleBar::Selected(current_channel_);

  auto &channel = current_channel();
  menu::SettingsList<menu::kScreenLines, 0, menu::kDefaultValueX> settings_list{ui_.cursor};

  menu::SettingsListItem list_item;
  while (settings_list.available()) {
    int setting = channel.enabled_setting_at(settings_list.Next(list_item));
    int value = channel.get_value(setting);
    auto &attr = ScopeChannel::value_attr(setting);

    switch (setting) {
      case SCOPE_CHANNEL_SETTING_YDIV:
        list_item.DrawDefault(channel.scaling().label, value, attr);
        break;
      default: list_item.DrawDefault(value, attr); break;
    }
  }
}

static inline weegfx::coord_t to_pixel(int16_t value, const int32_t multiplier)
{
  auto px = 32 - ((multiplier * value) >> (kScalingShift + 6));
  CONSTRAIN(px, 0, 63);
  return px;
}

/*static*/ void ScopeApp::RenderDisplayBuffer(const int16_t *display_buffer,
                                              const int32_t multiplier)
{
  auto end = display_buffer + kDisplayBufferSize;

  weegfx::coord_t x = 0;
  auto y1 = to_pixel(*display_buffer++, multiplier);
  while (display_buffer < end) {
    auto y2 = to_pixel(*display_buffer++, multiplier);
    graphics.drawLine(x, y1, x + 1, y2);
    y1 = y2;
    ++x;
  }
}

namespace icons {
static const uint8_t channel_1_8x8[] = {0xff, 0x01, 0x01, 0x09, 0x7d, 0x01, 0x01, 0xff};
static const uint8_t channel_2_8x8[] = {0xff, 0x01, 0x01, 0x75, 0x55, 0x59, 0x01, 0xff};
static const uint8_t channel_3_8x8[] = {0xff, 0x01, 0x01, 0x55, 0x55, 0x7d, 0x01, 0xff};
static const uint8_t channel_4_8x8[] = {0xff, 0x01, 0x01, 0x1d, 0x11, 0x79, 0x01, 0xff};

static const uint8_t trigger_rising_edge_8x8[] = {0x00, 0x80, 0x90, 0x98, 0xff, 0x19, 0x11, 0x00};
static const uint8_t trigger_falling_edge_8x8[] = {0x00, 0x01, 0x09, 0x19, 0xff, 0x98, 0x88, 0x00};
static const uint8_t trigger_ext1_8x8[] = {0x04, 0x7c, 0x04, 0x70, 0x10, 0x00, 0x08, 0x7c};
static const uint8_t trigger_ext2_8x8[] = {0x04, 0x7c, 0x04, 0x70, 0x10, 0x00, 0x74, 0x5c};

static const uint8_t trigger_level_3x8[] = {0x3e, 0x1c, 0x08};

static constexpr const uint8_t *channels[4] = {
    channel_1_8x8,
    channel_2_8x8,
    channel_3_8x8,
    channel_4_8x8,
};

static constexpr const uint8_t *trigger_type_icons[TriggerProcessor::TRIGGER_TYPE_LAST] = {
    nullptr, trigger_rising_edge_8x8, trigger_falling_edge_8x8, trigger_ext1_8x8, trigger_ext2_8x8,
};

const uint8_t edit_indicators_8[3 * 3] = {
    0x66, 0xe7, 0x66,  // both
    0x06, 0x07, 0x06,  // min
    0x60, 0xe0, 0x60,  // max
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

};  // namespace icons

void ScopeApp::RenderScopeUI() const
{
  namespace DEBUG = TU::DEBUG;
  auto &channel = current_channel();

  // Top [channel] .... [trigger]
  if (!ui_.status_bar.visible()) {
    graphics.drawBitmap8(3, 0, 8, icons::channels[current_channel_]);
    graphics.drawHLine(3, 8, 8);
  }

  const uint8_t *icon = icons::trigger_type_icons[channel.trigger_type()];
  weegfx::coord_t x = 128 - 8;
  if (icon) {
    graphics.drawBitmap8(128 - 8, 0, 8, icon);
    x -= 7;
  }
  if (ui_.trigger_lost) {
    graphics.setPrintPos(x, 0);
    graphics.print('?');
  }

  // Left: Trigger level
  auto trigger_level_y = to_pixel(channel.trigger_level(), channel.scaling().multiplier) - 3;
  if (TriggerProcessor::TRIGGER_TYPE_NONE != channel.trigger_type()) {
    CONSTRAIN(trigger_level_y, 0, 58);
    graphics.drawBitmap8(0, trigger_level_y, 3, icons::trigger_level_3x8);
  }

  if (ui_.edit_setting.visible()) {
    // On-screen edit overlay?
    CONSTRAIN(trigger_level_y, 0, 56);
    graphics.setPrintPos(6, trigger_level_y);
    graphics.pretty_print(channel.trigger_level(), 5);
  } else if (ui_.status_bar.visible()) {
    // Bottom [channel][?????][timebase][scale]
    weegfx::coord_t bottom_text_y = 64 - weegfx::Graphics::kFixedFontH;

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
        x += weegfx::Graphics::kFixedFontW + 1;
      }
      graphics.invertRect(current_channel_ * (weegfx::Graphics::kFixedFontW + 1), bottom_text_y,
                          weegfx::Graphics::kFixedFontW + 1, weegfx::Graphics::kFixedFontH + 1);
    } else {
      graphics.setPrintPos(current_channel_ * (weegfx::Graphics::kFixedFontW + 1), bottom_text_y);
      graphics.print((char)('1' + current_channel_));
    }

    x = 32 + 6;
    if (!edit_channel)
      icons::DrawEditIcon(x - 1, bottom_text_y - 1, channel.xdiv(),
                          channel.value_attr(SCOPE_CHANNEL_SETTING_XDIV));
    graphics.setPrintPos(x, bottom_text_y);
    graphics.print(channel.timebase().label);

    x = 96 + 6;
    if (SCOPE_CHANNEL_SETTING_YDIV == ui_.edit_setting_r)
      icons::DrawEditIcon(x - 1, bottom_text_y - 1, channel.ydiv(),
                          channel.value_attr(SCOPE_CHANNEL_SETTING_YDIV));
    graphics.setPrintPos(x, bottom_text_y);
    graphics.printf(channel.scaling().label);
  }

  // Info/debug overlay
  if (ui_.info_overlay.visible()) {
    graphics.setPrintPos(32, 0);
    graphics.print(channel.trigger_count() & 0xffff, 5);

    graphics.setPrintPos(32, 8);
    graphics.print(debug::cycles_to_us(process_cycles.value()), 5);
  }

  graphics.setPrintPos(128 - 30 - 18, 0);
  graphics.print(debug::cycles_to_us(DEBUG::MENU_draw_cycles.value()), 5);
}

void ScopeApp::Activate()
{
  TU::ADC::StartConversionBuffered(current_channel().timebase().adc_frequency,
                                   current_adc_channel());
}

static ScopeApp scope_app_instance;

}  // namespace scope

void SCOPE_init()
{
  scope::scope_app_instance.Init();
}

size_t SCOPE_storageSize()
{
  return sizeof(int) + scope::ScopeApp::kNumChannels * scope::ScopeChannel::storageSize();
}

size_t SCOPE_save(util::StreamBufferWriter &stream)
{
  return scope::scope_app_instance.Save(stream);
}

size_t SCOPE_restore(util::StreamBufferReader &stream)
{
  return scope::scope_app_instance.Restore(stream);
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

