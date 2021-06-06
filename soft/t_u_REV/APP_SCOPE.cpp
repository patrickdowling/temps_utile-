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
#include "util/util_settings.h"

// NOTES
// - SIMD processing of buffers?

namespace scope {

static constexpr weegfx::coord_t kDisplayBufferSize = weegfx::Graphics::kWidth;
static constexpr size_t kAcquireBufferSize = 256;

static int32_t acquire_buffer[kAcquireBufferSize];
static size_t acquire_buffer_index = 0;

static constexpr uint32_t kSettingTimeoutTicks = 5000;
static constexpr uint32_t kMenuTimeoutTicks = 30000;

class PopupElement {
public:
  bool visible() const { return visible_; }

  void Tick(uint32_t ticks)
  {
    if (visible_) {
      if (ticks - start_ticks_ > timeout_) visible_ = false;
    }
  }

  void hide() { visible_ = false; };
  void show()
  {
    visible_ = true;
    start_ticks_ = TU::ui.ticks();
  }

  void poke() { start_ticks_ = TU::ui.ticks(); }

  void set_timeout(uint32_t timeout) { timeout_ = timeout; }

private:
  bool visible_ = false;
  uint32_t start_ticks_ = 0;
  uint32_t timeout_ = kSettingTimeoutTicks;
};

enum ScopeChannelSettings {
  SCOPE_CHANNEL_SETTING_YDIV,
  SCOPE_CHANNEL_SETTING_TRIG_LEVEL,
  SCOPE_CHANNEL_SETTING_LAST,
};

class ScopeChannel : public settings::SettingsBase<ScopeChannel, SCOPE_CHANNEL_SETTING_LAST> {
public:
  void Init();
  void Process(int32_t sample);

  static const int32_t *ProcessBuffer(int32_t trigger_level, const int32_t *buffer, size_t length);

  const int32_t *display_buffer() const { return display_buffer_; }

  uint16_t trigger_count() const { return trigger_count_; }

private:
  int32_t display_buffer_[kDisplayBufferSize];
  uint16_t trigger_count_{0};
};

SETTINGS_DECLARE(scope::ScopeChannel, scope::SCOPE_CHANNEL_SETTING_LAST){
    // default, min, max, name, value_names, storage_type, parent_index, parent_value
    {1, 1, 2, "YDIV", nullptr, settings::STORAGE_TYPE_U8},
    {32, 0, 32, "TRIGLVL", nullptr, settings::STORAGE_TYPE_I32},
};

void ScopeChannel::Init()
{
  InitDefaults();
  std::fill(std::begin(display_buffer_), std::end(display_buffer_), 0);
}

void ScopeChannel::Process(int32_t sample)
{
  acquire_buffer[acquire_buffer_index] = sample;
  if (acquire_buffer_index < kAcquireBufferSize - 1) {
    ++acquire_buffer_index;
  } else {
    auto trigger_sample =
        ProcessBuffer(32, acquire_buffer, kAcquireBufferSize - kDisplayBufferSize);
    if (trigger_sample) {
      std::copy(trigger_sample, trigger_sample + kDisplayBufferSize, display_buffer_);
      ++trigger_count_;
    }
    acquire_buffer_index = 0;
  }
}

/*static*/ const int32_t *ScopeChannel::ProcessBuffer(int32_t trigger_level, const int32_t *buffer,
                                                      size_t length)
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
  void Render() const;
  void RenderScreensaver() const;

  void EventScreensaverOff();

  static void RenderGrid();

  ADC_CHANNEL current_adc_channel() const { return static_cast<ADC_CHANNEL>(current_channel_); }

private:
  struct {
    bool menu_active = false;
    PopupElement ydiv_display;
  } ui_;

  int current_channel_{0};

  ScopeChannel channels_[kNumChannels];

  void RenderMenu() const;
  void RenderScope() const;
  void RenderScopeUI() const;
};

void ScopeApp::Init()
{
  for (auto &channel : channels_) channel.Init();
}

void ScopeApp::Process()
{
  auto sample = TU::ADC::raw_offset_value(current_adc_channel());
  channels_[current_channel_].Process(sample);
}

void ScopeApp::UpdateUI()
{
  auto ticks = TU::ui.ticks();
  ui_.ydiv_display.Tick(ticks);
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
    }
  }
}

void ScopeApp::OnEncoder(const UI::Event &event)
{
  auto &current_channel = channels_[current_channel_];

  if (ui_.menu_active) {
    if (TU::CONTROL_ENCODER_L == event.control) {
      auto channel = current_channel_ + event.value;
      CONSTRAIN(channel, 0, kNumChannels - 1);
      current_channel_ = channel;
    }
  }
  if (TU::CONTROL_ENCODER_R == event.control) {
    current_channel.change_value(SCOPE_CHANNEL_SETTING_YDIV, event.value);
    ui_.ydiv_display.show();
  }
}

void ScopeApp::Render() const
{
  if (ui_.menu_active) {
    RenderMenu();
  } else {
    RenderGrid();
    RenderScope();
    RenderScopeUI();
  }
}

void ScopeApp::RenderScreensaver() const
{
  RenderScope();
}

void ScopeApp::EventScreensaverOff()
{
  ui_.menu_active = false;
}

void ScopeApp::RenderMenu() const
{
  namespace menu = TU::menu;
  menu::QuadTitleBar::Draw(true);
  for (int i = 0; i < 4; ++i) {
    menu::QuadTitleBar::SetColumn(i);
    graphics.print((char)('1' + i));
  }
  menu::QuadTitleBar::Selected(current_channel_);
}

void ScopeApp::RenderScope() const
{
  auto &current_channel = channels_[current_channel_];

  auto ydiv = current_channel.get_value(SCOPE_CHANNEL_SETTING_YDIV);
  auto display_buffer = current_channel.display_buffer();
  for (weegfx::coord_t x = 0; x < (kDisplayBufferSize - 1); ++x) {
    auto y1 = 32 - ((ydiv * display_buffer[x]) >> 6);
    CONSTRAIN(y1, 0, 63);
    auto y2 = 32 - ((ydiv * display_buffer[x + 1]) >> 6);
    CONSTRAIN(y2, 0, 63);

    graphics.drawLine(x, y1, x + 1, y2);
  }
}

void ScopeApp::RenderScopeUI() const
{
  namespace DEBUG = TU::DEBUG;
  auto &current_channel = channels_[current_channel_];

  graphics.setPrintPos(1, 1);
  graphics.print((char)('1' + current_channel_));
  graphics.drawFrame(0, 0, weegfx::Graphics::kFixedFontW + 3, weegfx::Graphics::kFixedFontH + 2);

  if (ui_.ydiv_display.visible()) {
    graphics.setPrintPos(128 - 2 * weegfx::Graphics::kFixedFontW, 0);
    graphics.printf("x%d", current_channel.get_value(SCOPE_CHANNEL_SETTING_YDIV));
  }

  graphics.drawBitmap8(0,
                       32 - (current_channel.get_value(SCOPE_CHANNEL_SETTING_TRIG_LEVEL) >> 6) - 4,
                       TU::kBitmapLoopMarkerW, TU::bitmap_loop_markers_8);

  auto x = 128 - weegfx::Graphics::kFixedFontW * 5;

  graphics.setPrintPos(x, 64 - weegfx::Graphics::kFixedFontH);
  graphics.print(current_channel.trigger_count(), 5);

  graphics.setPrintPos(x, 64 - weegfx::Graphics::kFixedFontH * 2);
  graphics.print(debug::cycles_to_us(DEBUG::MENU_draw_cycles.value()), 5);
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
    case TU::APP_EVENT_RESUME: break;
    case TU::APP_EVENT_SUSPEND: break;
    case TU::APP_EVENT_SCREENSAVER_ON: break;
    case TU::APP_EVENT_SCREENSAVER_OFF: scope::scope_app_instance.EventScreensaverOff(); break;
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

