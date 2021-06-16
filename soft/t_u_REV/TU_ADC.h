// Copyright 2015-2021 Max Stadler, Patrick Dowling
// Author: Max Stadler, Patrick Dowling (pld@gurkenkiste.com)
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

#ifndef TU_ADC_H_
#define TU_ADC_H_

#include <DMAChannel.h>
#include <stdint.h>
#include <string.h>

#include "TU_config.h"
#include "src/ADC/OC_util_ADC.h"

enum ADC_CHANNEL {
  ADC_CHANNEL_1,
  ADC_CHANNEL_2,
  ADC_CHANNEL_3,
  ADC_CHANNEL_4,
  ADC_CHANNEL_LAST,
};

#ifdef TU_ADC_DEBUG_SERIAL
#define ADC_SERIAL_PRINTLN(...) SERIAL_PRINTLN("[ADC] " __VA_ARGS__)
#else
#define ADC_SERIAL_PRINTLN(...) \
  do {                          \
  } while (0)
#endif

namespace TU {

// There are two modes for ADC use:
// ADC_MODE_NORMAL
// This is just the "classic" implementation as ported from o_C. All channels are scanned via DMA,
// the DMA disables after completion, and is restarted in ::Update after the values have been
// averaged. This isn't _great_ but gets the job done. The scan runs "as fast as the ADC can
// convert" since it's self-triggering. The app interface is via the per-channel getter functions.
//
// ADC_MODE_BUFFERED
// This mode was implemented for the 'scope app, and just runs a continuous double/quad buffered DMA
// acquisition, with the app grabbing chunks. The expectation is that any ReadChunk calls will be
// more frequent than the buffer wraps around. This mode also provides a way to change the timing.
class ADC {
public:
  static constexpr size_t kDMAChunkSize = 128;
  static constexpr size_t kDMAMaxChunkCount = 2;
  static constexpr size_t kDMABufferSize = kDMAChunkSize * kDMAMaxChunkCount;

  enum ADC_MODE { ADC_MODE_INVALID, ADC_MODE_NORMAL, ADC_MODE_BUFFERED };

  struct CalibrationData {
    uint16_t offset[ADC_CHANNEL_LAST];
    uint16_t pitch_cv_scale;
    int16_t pitch_cv_offset;
  };

  struct Config;

  // Base initialization in default/normal mode
  static void Init(CalibrationData *calibration_data);

  // Start conversions in "original" mode, i.e. all four channels, averaging, etc.
  static void StartConversionNormal();

  // Start conversions in buffered mode (details TBD)
  static void StartConversionBuffered(uint32_t freq, ADC_CHANNEL channel1,
                                      ADC_CHANNEL channel2 = ADC_CHANNEL_LAST);

  // Periodic update function (expected to run in main ISR)
  static void Update();

  static ADC_MODE mode() { return mode_; }

  // BUFFERED_MODE
  static size_t ReadChunk(uint16_t *buffer);

  static int16_t offset_value(ADC_CHANNEL adc_channel, uint16_t value)
  {
    return calibration_data_->offset[adc_channel] - value;
  }

  static uint16_t channel_offset(ADC_CHANNEL adc_channel)
  {
    return calibration_data_->offset[adc_channel];
  }

  // NORMAL_MODE
  // These are the default settings for the original ADC use (as seen on o_C as well)
  static constexpr uint8_t kAdcResolution = 12;
  static constexpr uint32_t kAdcSmoothing = 4;
  static constexpr uint32_t kAdcSmoothBits = 8;      // fractional bits for smoothing
  static constexpr uint8_t kAdcScanResolution = 16;  // normal mode

  template <ADC_CHANNEL channel>
  static int32_t value()
  {
    return calibration_data_->offset[channel] - (smoothed_[channel] >> kAdcSmoothBits);
  }

  static int32_t value(ADC_CHANNEL channel)
  {
    return calibration_data_->offset[channel] - (smoothed_[channel] >> kAdcSmoothBits);
  }

  static uint32_t raw_value(ADC_CHANNEL channel) { return raw_[channel] >> kAdcSmoothBits; }

  static int32_t raw_offset_value(ADC_CHANNEL channel)
  {
    return calibration_data_->offset[channel] - raw_value(channel);
  }

  static uint32_t smoothed_raw_value(ADC_CHANNEL channel)
  {
    return smoothed_[channel] >> kAdcSmoothBits;
  }

  static int32_t pitch_value(ADC_CHANNEL channel)
  {
    return (value(channel) * calibration_data_->pitch_cv_scale) >> 12;
  }

  static int32_t raw_pitch_value(ADC_CHANNEL channel)
  {
    int32_t value = calibration_data_->offset[channel] - raw_value(channel);
    return (value * calibration_data_->pitch_cv_scale) >> 12;
  }

  // DEBUG
  static volatile void *DEBUG_DADDR();

private:
  template <ADC_CHANNEL channel>
  static void update(uint32_t value)
  {
    value = (value >> (kAdcScanResolution - kAdcResolution)) << kAdcSmoothBits;
    raw_[channel] = value;
    // division should be shift if kAdcSmoothing is power-of-two
    value = (smoothed_[channel] * (kAdcSmoothing - 1) + value) / kAdcSmoothing;
    smoothed_[channel] = value;
  }

  static CalibrationData *calibration_data_;
  static ADC_MODE mode_;
  static ::ADC adc_;

  static size_t last_chunk_;

  static uint32_t raw_[ADC_CHANNEL_LAST];
  static uint32_t smoothed_[ADC_CHANNEL_LAST];

  static void Configure(const Config &config);

  static void InitDMASettingsNormal();
  static void InitDMASettingsBuffered();
  static void StartDMA(ADC_MODE mode, DMASetting *dma_settings);
  static void StopDMA();
  static void StartPDB(uint32_t freq);
  static void StopPDB();

  // Deprecated?
public:
  static void CalibratePitch(int32_t c2, int32_t c4);
};

}  // namespace TU

#endif  // TU_ADC_H_
