// Copyright 2021 Patrick Dowling
// Author: Patrick Dowling (pld@gurkenkiste.com)
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

#include "TU_ADC.h"
#include "TU_debug.h"
#include "TU_menus.h"
#include "TU_ui.h"
#include "util/util_popup.h"
#include "util/util_settings.h"

// NOTES
// - SIMD processing of buffers?
// - Raw values from ADC are inverted, so we use calibration offset

namespace scope {

namespace menu = TU::menu;

static constexpr weegfx::coord_t kDisplayBufferSize = weegfx::Graphics::kWidth;
static constexpr size_t kADCChunkSize = TU::ADC::kDMAChunkSize;

template <typename T, size_t chunk_size, size_t num_chunks>
class CircularSampleBuffer {
public:
  static constexpr size_t kChunkSize = chunk_size;
  static constexpr size_t kNumChunks = num_chunks;
  static constexpr size_t kBufferSize = kChunkSize * kNumChunks;

  void advance()
  {
    ++head_;
    ++tail_;
  }

  const T *head_buffer() const { return buffer_ + (head_ % kNumChunks) * kChunkSize; }
  const T *head_buffer(size_t i) const { return buffer_ + ((head_ + 1) % kNumChunks) * kChunkSize; }

  T *tail_buffer() { return buffer_ + (tail_ % kNumChunks) * kChunkSize; }

private:
  T buffer_[kBufferSize];
  size_t head_ = 0;
  size_t tail_ = kNumChunks - 1;
};

static constexpr uint32_t kSettingTimeoutTicks = 5000;
static constexpr uint32_t kMenuTimeoutTicks = 30000;

class TriggerProcessor {
public:
  enum TriggerType {
    TRIGGER_TYPE_NONE,
    TRIGGER_TYPE_RISING,
    TRIGGER_TYPE_FALLING,
    TRIGGER_TYPE_EXT,
    TRIGGER_TYPE_LAST
  };

  template <size_t length>
  static const int16_t *ScanBuffer(int16_t trigger_level, const int16_t *buffer)
  {
    size_t len = length;
    // Starting value is above trigger, find if/where it drops below
    while (len && buffer[0] > trigger_level) {
      ++buffer;
      --len;
    }

    while (len--) {
      if (buffer[0] > trigger_level) return buffer;
      ++buffer;
    }

    return nullptr;
  }
};

static constexpr const char *kTriggerTypeStrings[TriggerProcessor::TRIGGER_TYPE_LAST] = {
    "none",
    "rising",
    "falling",
    "ext",
};

enum TimebaseDivision { TIMEBASE_1, TIMEBASE_2, TIMEBASE_3, TIMEBASE_LAST };
struct TimebaseParameters {
  const char *const label;
  uint32_t adc_frequency;
  // gain?
};

static constexpr TimebaseParameters kTimebaseParameters[TIMEBASE_LAST] = {
    {"500", .adc_frequency = 500 * 128},
    {"1000", .adc_frequency = 1000 * 128},
    {"2000", .adc_frequency = 2000 * 128},
};

enum ScopeChannelSetting {
  SCOPE_CHANNEL_SETTING_XOFF,
  SCOPE_CHANNEL_SETTING_YOFF,
  SCOPE_CHANNEL_SETTING_XDIV,
  SCOPE_CHANNEL_SETTING_YDIV,
  SCOPE_CHANNEL_SETTING_TRIG_TYPE,
  SCOPE_CHANNEL_SETTING_TRIG_LEVEL,
  SCOPE_CHANNEL_SETTING_LAST,
  SCOPE_CHANNEL_SETTING_FIRST = SCOPE_CHANNEL_SETTING_XOFF
};

class ScopeChannel : public settings::SettingsBase<ScopeChannel, SCOPE_CHANNEL_SETTING_LAST> {
public:
  void Init();
  void Process(ADC_CHANNEL adc_channel, const uint16_t *adc_chunk);

  const int16_t *UpdateDisplayBuffer();

  uint32_t trigger_count() const { return trigger_count_; }

  // settings wrappers

  int xdiv() const { return get_value(SCOPE_CHANNEL_SETTING_XDIV); }
  int ydiv() const { return get_value(SCOPE_CHANNEL_SETTING_YDIV); }

  TriggerProcessor::TriggerType trigger_type() const
  {
    return static_cast<TriggerProcessor::TriggerType>(get_value(SCOPE_CHANNEL_SETTING_TRIG_TYPE));
  }

  int16_t trigger_level() const
  {
    return static_cast<int16_t>(get_value(SCOPE_CHANNEL_SETTING_TRIG_LEVEL));
  }

  const TimebaseParameters &current_timebase() const { return kTimebaseParameters[xdiv()]; }

  void UpdateEnabledSettings();
  int num_enabled_settings() const { return num_enabled_settings_; }
  int enabled_setting_at(int index) const { return enabled_settings_[index]; }

private:
  uint32_t trigger_count_{0};

  int num_enabled_settings_{0};
  ScopeChannelSetting enabled_settings_[SCOPE_CHANNEL_SETTING_LAST];

  FrameBuffer<kDisplayBufferSize, 2, int16_t> display_buffers_;
  const int16_t *current_display_buffer_ = nullptr;

  CircularSampleBuffer<int16_t, kADCChunkSize, 4> sample_buffer_;
};

SETTINGS_DECLARE(scope::ScopeChannel, scope::SCOPE_CHANNEL_SETTING_LAST){
    // default, min, max, name, value_names, storage_type, parent_index, parent_valuea
    {0, 0, 127, "XOFF", nullptr, settings::STORAGE_TYPE_I16},
    {0, -32, 32, "YOFF", nullptr, settings::STORAGE_TYPE_I16},
    {1, 0, scope::TIMEBASE_LAST - 1, "XDIV", nullptr, settings::STORAGE_TYPE_U8},
    {1, 1, 4, "YDIV", nullptr, settings::STORAGE_TYPE_U8},
    {scope::TriggerProcessor::TRIGGER_TYPE_RISING, scope::TriggerProcessor::TRIGGER_TYPE_NONE,
     scope::TriggerProcessor::TRIGGER_TYPE_RISING, "TRIG TYPE", scope::kTriggerTypeStrings,
     settings::STORAGE_TYPE_U8},
    {32, -2048, 2047, "TRIG LVL", nullptr, settings::STORAGE_TYPE_I16},
};

void ScopeChannel::Init()
{
  InitDefaults();
  UpdateEnabledSettings();

  display_buffers_.Init();
}

static debug::AveragedCycles process_cycles;

void ScopeChannel::Process(ADC_CHANNEL adc_channel, const uint16_t *adc_chunk)
{
  debug::ScopedCycleMeasurement cycles{process_cycles};

  // Offset raw samples
  auto tail = sample_buffer_.tail_buffer();
  for (auto src = adc_chunk; src < adc_chunk + kADCChunkSize; ++src)
    *tail++ = TU::ADC::offset_value(adc_channel, *src);

  sample_buffer_.advance();
  auto head = sample_buffer_.head_buffer();
  auto trigger = TriggerProcessor::TRIGGER_TYPE_NONE == trigger_type()
                     ? head
                     : TriggerProcessor::ScanBuffer<kADCChunkSize>(trigger_level(), head);
  if (trigger) {
    ++trigger_count_;
    if (display_buffers_.writeable()) {
      auto display_buffer = display_buffers_.writeable_frame();

      size_t n = trigger - head;
      std::copy(trigger, trigger + kADCChunkSize - n, display_buffer);
      display_buffer += kADCChunkSize - n;
      std::copy(sample_buffer_.head_buffer(1), sample_buffer_.head_buffer(1) + n, display_buffer);
      display_buffers_.written();
    }
  }
}

const int16_t *ScopeChannel::UpdateDisplayBuffer()
{
  if (display_buffers_.readable()) {
    if (current_display_buffer_) display_buffers_.read();
    current_display_buffer_ = display_buffers_.readable_frame();
  }
  return current_display_buffer_;
}
void ScopeChannel::UpdateEnabledSettings()
{
  auto settings = enabled_settings_;

  *settings++ = SCOPE_CHANNEL_SETTING_TRIG_TYPE;
  switch (trigger_type()) {
    case TriggerProcessor::TRIGGER_TYPE_NONE:
    case TriggerProcessor::TRIGGER_TYPE_EXT: break;
    default: *settings++ = SCOPE_CHANNEL_SETTING_TRIG_LEVEL;
  }
  *settings++ = SCOPE_CHANNEL_SETTING_XOFF;
  *settings++ = SCOPE_CHANNEL_SETTING_YOFF;
  *settings++ = SCOPE_CHANNEL_SETTING_XDIV;
  *settings++ = SCOPE_CHANNEL_SETTING_YDIV;

  num_enabled_settings_ = settings - enabled_settings_;
}

class ScopeApp {
public:
  static constexpr int kNumChannels = 4;

  void Init();
  void Process();
  void UpdateUI();

  size_t Save(util::StreamBufferWriter &stream_buffer) const;
  size_t Restore(util::StreamBufferReader &stream_buffer);

  void OnButton(const UI::Event &event);
  void OnEncoder(const UI::Event &event);
  void Render();             // const;
  void RenderScreensaver();  // const;

  void EventScreensaverOff();
  void Activate();

  static void RenderGrid();

private:
  struct {
    bool menu_active = false;
    bool edit_trigger_level = false;

    util::PopupElement xdiv_display;
    util::PopupElement ydiv_display;
    util::PopupElement info_overlay;

    menu::ScreenCursor<menu::kScreenLines> cursor;
  } ui_;

  int current_channel_{0};
  static uint16_t adc_chunk_buffer_[kADCChunkSize];

  ScopeChannel channels_[kNumChannels];

  ADC_CHANNEL current_adc_channel() const { return static_cast<ADC_CHANNEL>(current_channel_); }

  ScopeChannel &current_channel() { return channels_[current_channel_]; }
  const ScopeChannel &current_channel() const { return channels_[current_channel_]; }

  void RenderMenu() const;
  void RenderScope();  // const;
  void RenderScopeUI() const;
};

/*static*/ uint16_t ScopeApp::adc_chunk_buffer_[kADCChunkSize] __attribute__((aligned(4)));

void ScopeApp::Init()
{
  for (auto &channel : channels_) channel.Init();

  ui_.cursor.Init(SCOPE_CHANNEL_SETTING_FIRST, SCOPE_CHANNEL_SETTING_LAST - 1);
  ui_.cursor.AdjustEnd(current_channel().num_enabled_settings() - 1);
}

void ScopeApp::Process()
{
  if (TU::ADC::ReadChunk(adc_chunk_buffer_))
    channels_[current_channel_].Process(current_adc_channel(), adc_chunk_buffer_);

  // Other regular book-keeping?
}

void ScopeApp::UpdateUI()
{
  auto ticks = TU::ui.ticks();
  ui_.xdiv_display.Tick(ticks);
  ui_.ydiv_display.Tick(ticks);
  ui_.info_overlay.Tick(ticks);
}

size_t ScopeApp::Save(util::StreamBufferWriter &stream_buffer) const
{
  stream_buffer.Write(current_channel_);

  return stream_buffer.overflow() ? 0 : stream_buffer.written();
}

size_t ScopeApp::Restore(util::StreamBufferReader &stream_buffer)
{
  stream_buffer.Read(current_channel_);

  return stream_buffer.underflow() ? 0 : stream_buffer.read();
}

void ScopeApp::RenderGrid()
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

void ScopeApp::OnButton(const UI::Event &event)
{
  if (UI::EVENT_BUTTON_PRESS == event.type) {
    switch (event.control) {
      case TU::CONTROL_BUTTON_UP: {
        ui_.menu_active = !ui_.menu_active;
      } break;
      case TU::CONTROL_BUTTON_DOWN: {
        if (!ui_.menu_active) { ui_.info_overlay.show(); }
      } break;
      case TU::CONTROL_BUTTON_R: {
        if (!ui_.menu_active) {
          ui_.edit_trigger_level = !ui_.edit_trigger_level;
          ui_.ydiv_display.show();
        } else {
          ui_.cursor.toggle_editing();
        }
      } break;
      default: break;
    }
  }
}

void ScopeApp::OnEncoder(const UI::Event &event)
{
  auto &current_channel = channels_[current_channel_];

  if (ui_.menu_active) {
    if (TU::CONTROL_ENCODER_L == event.control) {
      // auto channel = current_channel_ + event.value;
      // CONSTRAIN(channel, 0, kNumChannels - 1);
      // current_channel_ = channel;
    } else if (TU::CONTROL_ENCODER_R == event.control) {
      if (!ui_.cursor.editing()) {
        ui_.cursor.Scroll(event.value);
      } else {
        auto selected = current_channel.enabled_setting_at(ui_.cursor.cursor_pos());
        if (current_channel.change_value(selected, event.value)) {
          current_channel.UpdateEnabledSettings();
          ui_.cursor.AdjustEnd(current_channel.num_enabled_settings());
        }
      }
    }

  } else {
    if (TU::CONTROL_ENCODER_L == event.control) {
      if (current_channel.change_value(SCOPE_CHANNEL_SETTING_XDIV, event.value))
        TU::ADC::StartConversionBuffered(ADC_CHANNEL_1,
                                         kTimebaseParameters[current_channel.xdiv()].adc_frequency);
      ui_.xdiv_display.show();
    } else if (TU::CONTROL_ENCODER_R == event.control) {
      if (ui_.edit_trigger_level) {
        current_channel.change_value(SCOPE_CHANNEL_SETTING_TRIG_LEVEL, event.value * 32);
      } else {
        current_channel.change_value(SCOPE_CHANNEL_SETTING_YDIV, event.value);
      }
      ui_.ydiv_display.show();
    }
  }
}

void ScopeApp::Render()  // const
{
  if (ui_.menu_active) {
    RenderMenu();
  } else {
    RenderGrid();
    RenderScope();
    RenderScopeUI();
  }
}

void ScopeApp::RenderScreensaver()  // const
{
  RenderScope();
}

void ScopeApp::EventScreensaverOff()
{
  ui_.menu_active = false;
  ui_.cursor.set_editing(false);
  ui_.xdiv_display.show();
  ui_.ydiv_display.show();
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
      default: list_item.DrawDefault(value, attr); break;
    }
  }
}

void ScopeApp::RenderScope()  // const
{
  auto &channel = current_channel();

  auto display_buffer = channel.UpdateDisplayBuffer();
  if (display_buffer) {
    auto ydiv = channel.ydiv();
    auto y1 = 32 - ((ydiv * display_buffer[0]) >> 6);
    CONSTRAIN(y1, 0, 63);
    for (weegfx::coord_t x = 0; x < kDisplayBufferSize - 1; ++x) {
      auto y2 = 32 - ((ydiv * display_buffer[x]) >> 6);
      CONSTRAIN(y2, 0, 63);

      graphics.drawLine(x, y1, x + 1, y2);
      y1 = y2;
    }
  }
}

namespace icons {
static const uint8_t rising_edge_8x8[] = {0x60, 0x60, 0x60, 0x7f, 0x7f, 0x03, 0x03, 0x03};
};

void ScopeApp::RenderScopeUI() const
{
  namespace DEBUG = TU::DEBUG;
  auto &channel = current_channel();

  graphics.setPrintPos(1, 1);
  graphics.print((char)('1' + current_channel_));
  graphics.drawFrame(0, 0, weegfx::Graphics::kFixedFontW + 3, weegfx::Graphics::kFixedFontH + 2);

  if (ui_.xdiv_display.visible()) {
    graphics.setPrintPos(0, 64 - 8);
    graphics.print(channel.current_timebase().label);
  }
  if (ui_.ydiv_display.visible()) {
    if (ui_.edit_trigger_level) {
      graphics.setPrintPos(128 - 5 * weegfx::Graphics::kFixedFontW, 64 - 8);
      graphics.pretty_print(channel.trigger_level(), 5);
    } else {
      graphics.setPrintPos(128 - 2 * weegfx::Graphics::kFixedFontW, 64 - 8);
      graphics.printf("x%d", channel.ydiv());
    }
  }

  graphics.drawBitmap8(0, 32 - (channel.trigger_level() >> 6) - 1, TU::kBitmapLoopMarkerW,
                       TU::bitmap_loop_markers_8);

  const uint8_t *icon = nullptr;
  switch (channel.trigger_type()) {
    case TriggerProcessor::TRIGGER_TYPE_RISING: icon = icons::rising_edge_8x8; break;
    default: break;
  }
  if (icon) graphics.drawBitmap8(128 - 8, 0, 8, icon);
  if (ui_.info_overlay.visible()) {
    graphics.setPrintPos(32, 0);
    graphics.print(channel.trigger_count() & 0xffff, 5);

    graphics.setPrintPos(32, 8);
    graphics.print(debug::cycles_to_us(process_cycles.value()), 5);
  }

  graphics.setPrintPos(128 - 30, weegfx::Graphics::kFixedFontH);
  graphics.print(debug::cycles_to_us(DEBUG::MENU_draw_cycles.value()), 5);
}

void ScopeApp::Activate()
{
  TU::ADC::StartConversionBuffered(ADC_CHANNEL_1, 1000 * 128);
}

static ScopeApp scope_app_instance;

}  // namespace scope

void SCOPE_init()
{
  scope::scope_app_instance.Init();
}

size_t SCOPE_storageSize()
{
  return sizeof(int);
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
  scope::scope_app_instance.OnButton(event);
}

void SCOPE_handleEncoderEvent(const UI::Event &event)
{
  scope::scope_app_instance.OnEncoder(event);
}

void SCOPE_isr()
{
  scope::scope_app_instance.Process();
}

