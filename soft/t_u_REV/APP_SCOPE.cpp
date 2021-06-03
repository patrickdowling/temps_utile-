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

#include "TU_ADC.h"
#include "TU_menus.h"
#include "TU_ui.h"
#include "util/util_settings.h"

static int32_t data_buffer[128];
static size_t data_buffer_index = 0;

namespace scope {

enum ScopeChannelSettings {
  SCOPE_CHANNEL_SETTING_YDIV,
  SCOPE_CHANNEL_SETTING_LAST,
};

class ScopeChannel : public settings::SettingsBase<ScopeChannel, SCOPE_CHANNEL_SETTING_LAST> {
public:
  void Init();

private:
};

SETTINGS_DECLARE(scope::ScopeChannel, scope::SCOPE_CHANNEL_SETTING_LAST){
    // default, min, max, name, value_names, storage_type, parent_index, parent_value
    {1, 1, 2, "YDIV", nullptr, settings::STORAGE_TYPE_U8},
};

void ScopeChannel::Init()
{
  InitDefaults();
}

class ScopeApp {
public:
  static constexpr uint32_t kMenuTimeoutTicks = 5000;
  static constexpr uint32_t kSettingTimeoutTicks = 2000;
  static constexpr int kNumChannels = 4;

  void Init();
  void UpdateUI();

  size_t Save(util::StreamBufferWriter &stream_buffer) const;
  size_t Restore(util::StreamBufferReader &stream_buffer);

  void OnButton(const UI::Event &event);
  void OnEncoder(const UI::Event &event);
  void RenderMenu() const;

  static void RenderGrid();

  ADC_CHANNEL current_adc_channel() const { return static_cast<ADC_CHANNEL>(current_channel_); }

private:
  struct {
    bool menu_active = false;
    uint32_t menu_active_ticks = 0;
    bool ydiv_display = false;
    uint32_t ydiv_active_ticks = 0;
  } ui_;

  int current_channel_{0};

  ScopeChannel channels_[kNumChannels];
};

void ScopeApp::Init()
{
  for (auto &channel : channels_) channel.Init();
}

void ScopeApp::UpdateUI()
{
  auto ticks = TU::ui.ticks();
  if (ui_.menu_active) {
    if (ticks - ui_.menu_active_ticks > kMenuTimeoutTicks) ui_.menu_active = false;
  }
  if (ui_.ydiv_display) {
    if (ticks - ui_.ydiv_active_ticks > kSettingTimeoutTicks) ui_.ydiv_display = false;
  }
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
      case TU::CONTROL_BUTTON_L: {
        ui_.menu_active = !ui_.menu_active;
        if (ui_.menu_active) ui_.menu_active_ticks = TU::ui.ticks();
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
    ui_.ydiv_display = true;
    ui_.ydiv_active_ticks = TU::ui.ticks();
  }
}

void ScopeApp::RenderMenu() const
{
  namespace menu = TU::menu;

  auto &current_channel = channels_[current_channel_];

  RenderGrid();

  auto ydiv = current_channel.get_value(SCOPE_CHANNEL_SETTING_YDIV);
  for (weegfx::coord_t x = 0; x < 127; ++x) {
    auto y1 = 32 - ((ydiv * data_buffer[x]) >> 6);
    CONSTRAIN(y1, 0, 63);
    auto y2 = 32 - ((ydiv * data_buffer[x + 1]) >> 6);
    CONSTRAIN(y2, 0, 63);

    graphics.drawLine(x, y1, x + 1, y2);
  }

  if (ui_.menu_active) {
    menu::QuadTitleBar::Draw(true);
    for (int i = 0; i < 4; ++i) {
      menu::QuadTitleBar::SetColumn(i);
      graphics.print((char)('1' + i));
    }
    menu::QuadTitleBar::Selected(current_channel_);
  } else {
    graphics.setPrintPos(1, 1);
    graphics.print((char)('1' + current_channel_));
    graphics.drawFrame(0, 0, weegfx::Graphics::kFixedFontW + 3, weegfx::Graphics::kFixedFontH + 2);

    if (ui_.ydiv_display) {
      graphics.setPrintPos(128 - 2 * weegfx::Graphics::kFixedFontW, 0);
      graphics.printf("x%d", current_channel.get_value(SCOPE_CHANNEL_SETTING_YDIV));
    }
  }
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
    case TU::APP_EVENT_RESUME:
    case TU::APP_EVENT_SUSPEND:
    case TU::APP_EVENT_SCREENSAVER_ON:
    case TU::APP_EVENT_SCREENSAVER_OFF:
    default: break;
  }
}

void SCOPE_loop()
{
  scope::scope_app_instance.UpdateUI();
}

void SCOPE_menu()
{
  scope::scope_app_instance.RenderMenu();
}

void SCOPE_screensaver() {}

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
  data_buffer[data_buffer_index] =
      TU::ADC::raw_offset_value(scope::scope_app_instance.current_adc_channel());
  data_buffer_index = (data_buffer_index + 1) & 0x7f;
}

