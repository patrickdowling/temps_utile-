#ifndef TU_ADC_H_
#define TU_ADC_H_

#include <stdint.h>
#include <string.h>

#include "TU_config.h"
#include "src/ADC/OC_util_ADC.h"

//#define ENABLE_ADC_DEBUG

enum ADC_CHANNEL {
  ADC_CHANNEL_1,
  ADC_CHANNEL_2,
  ADC_CHANNEL_3,
  ADC_CHANNEL_4,
  ADC_CHANNEL_LAST,
};

namespace TU {

class ADC {
public:
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

  // Start conversions in immediate mode (details TBD)
  static void StartConversionImmediate();

  // Periodic update function (expected to run in main ISR)
  static void Update();

  // IMMEDIATE_MODE

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

  enum ADC_MODE { ADC_MODE_INVALID, ADC_MODE_NORMAL, ADC_MODE_IMMEDIATE };

  static CalibrationData *calibration_data_;
  static ADC_MODE mode_;
  static ::ADC adc_;

  static uint32_t raw_[ADC_CHANNEL_LAST];
  static uint32_t smoothed_[ADC_CHANNEL_LAST];

  static void Configure(const Config &config);

  static void InitDMASettingsNormal();
  static void InitDMASettingsImmediate();
  static void StartDMA();
  static void StopDMA();

  // Deprecated?
public:
  static void CalibratePitch(int32_t c2, int32_t c4);
};

}  // namespace TU

#endif  // TU_ADC_H_
