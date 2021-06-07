#include "TU_ADC.h"

// #define TU_ADC_ENABLE_DMA_INTERRUPT
#ifdef TU_ADC_ENABLE_DMA_INTERRUPT
static volatile bool dma0_complete = false;
#endif

#include <algorithm>

#include "DMAChannel.h"
#include "TU_gpio.h"
#include "src/util_misc.h"

// NOTES
// There are two ADCs, but we might not be able to usefully map pins; it seems like only A2/A3
// are ADC0/1 capable, so CV1 only. This would also require 2x2 DMA streams to handle the muxing.
//
// - DMASetting/replaceSettingsOnCompletion to provide double buffering
// - There's a half-transfer interrupt, but no equivalent to DMA_TCD_CSR_DONE?
// - For scope use, there are comparators?

namespace TU {

struct ADC::Config {
  const uint8_t resolution;
  const uint8_t averaging;
  const uint8_t sampling_speed;
  const uint8_t conversion_speed;
};

// 16 bit has best-case 13 bits useable, but we only want 12 so we discard 4 anyway
static constexpr ADC::Config kConfigNormal = {
    .resolution = ADC::kAdcScanResolution,
    .averaging = 4,
    .sampling_speed = ADC_HIGH_SPEED_16BITS,
    .conversion_speed = ADC_HIGH_SPEED,
};

/*static*/ ADC::CalibrationData* ADC::calibration_data_ = nullptr;
/*static*/ ADC::ADC_MODE ADC::mode_ = ADC::ADC_MODE_INVALID;
/*static*/ ::ADC ADC::adc_;
/*static*/ uint32_t ADC::raw_[ADC_CHANNEL_LAST];
/*static*/ uint32_t ADC::smoothed_[ADC_CHANNEL_LAST];

#define DMA_BUF_SIZE 16
#define DMA_NUM_CH ADC_CHANNEL_LAST
// below: channel ids for the ADCx_SCA register: we have 4 inputs
// CV1 (17) = A3 = 0x49; CV2 (20) = A6 = 0x46; CV3 (19) = A5 = 0x4C; CV4 (18) = A4 = 0x4D
//
// We have to ensure DMA is started in the correct order, so that SCA is written first. Otherwise,
// the channel that reads from the ADC will read the "old" value and the values are out of order
// (see older revisions of this file).
static constexpr uint16_t SCA_CHANNEL_ID[DMA_NUM_CH] = {0x49, 0x46, 0x4C, 0x4D};

static DMAChannel dma0{false};  // dma0 channel, fills adcbuffer_0
static DMAChannel dma1{false};  // dma1 channel, updates ADC0_SC1A which holds the channel/pin IDs
DMAMEM static volatile uint16_t adcbuffer_0[DMA_BUF_SIZE]
    __attribute__((aligned(DMA_BUF_SIZE + 0)));

// Maintain basic DMA settings for each mode
// This doesn't include some things like linking, which we have to setup manually
static DMASetting dma_settings_normal[2];
static DMASetting dma_settings_immediate[2];

// #define TU_ADC_ENABLE_DMA_INTERRUPT
#ifdef TU_ADC_ENABLE_DMA_INTERRUPT
static volatile bool dma0_complete = false;
#endif

/*static*/ void ADC::Init(CalibrationData* calibration_data)
{
  calibration_data_ = calibration_data;
  std::fill(raw_, raw_ + ADC_CHANNEL_LAST, 0);
  std::fill(smoothed_, smoothed_ + ADC_CHANNEL_LAST, 0);
  std::fill(adcbuffer_0, adcbuffer_0 + DMA_BUF_SIZE, 0);

  adc_.setReference(ADC_REF_3V3);

  dma0.begin(true);  // allocate the DMA channel
  dma1.begin(true);  // allocate the DMA channel
  SERIAL_PRINTLN("[ADC] dma0.channel=%x", dma0.channel);
  SERIAL_PRINTLN("[ADC] dma1.channel=%x", dma1.channel);

  InitDMASettingsNormal();
  InitDMASettingsImmediate();

  StartConversionNormal();
}

/*static*/ void ADC::Configure(const Config& config)
{
  adc_.setResolution(config.resolution);
  adc_.setConversionSpeed(config.conversion_speed);
  adc_.setSamplingSpeed(config.sampling_speed);
  adc_.setAveraging(config.averaging);
}

#ifdef TU_ADC_ENABLE_DMA_INTERRUPT
static void DMA0_ISR_NORMAL()
{
  dma0_complete = true;
  dma0.TCD->DADDR = &adcbuffer_0[0];
  dma0.clearInterrupt();
  // DMA is either restarted in ::Update, or disableOnCompletion is not set
}
#endif

// DMA/ADC à la
// https://forum.pjrc.com/threads/30171-Reconfigure-ADC-via-a-DMA-transfer-to-allow-multiple-Channel-Acquisition
// basically, this sets up two DMA channels and cycles through the four adc mux channels (until the
// buffer is full), resets, and so on; dma1 advances SCA_CHANNEL_ID somewhat like
// https://www.nxp.com/docs/en/application-note/AN4590.pdf but w/o the PDB.
/*static*/ void ADC::InitDMASettingsNormal()
{
  // Normal DMA settings
  dma_settings_normal[0].TCD->SADDR = &ADC0_RA;
  dma_settings_normal[0].TCD->SOFF = 0;
  dma_settings_normal[0].TCD->ATTR = 0x101;
  dma_settings_normal[0].TCD->NBYTES = 2;
  dma_settings_normal[0].TCD->SLAST = 0;
  dma_settings_normal[0].TCD->DADDR = &adcbuffer_0[0];
  dma_settings_normal[0].TCD->DOFF = 2;
  dma_settings_normal[0].TCD->DLASTSGA = -(2 * DMA_BUF_SIZE);
  dma_settings_normal[0].TCD->BITER = DMA_BUF_SIZE;
  dma_settings_normal[0].TCD->CITER = DMA_BUF_SIZE;

  dma_settings_normal[1].TCD->SADDR = &SCA_CHANNEL_ID[0];
  dma_settings_normal[1].TCD->SOFF = 2;  // source increment each transfer (n bytes)
  dma_settings_normal[1].TCD->ATTR = 0x101;
  dma_settings_normal[1].TCD->SLAST = -(DMA_NUM_CH * 2);  // num ADC0 samples * 2
  dma_settings_normal[1].TCD->BITER = DMA_NUM_CH;
  dma_settings_normal[1].TCD->CITER = DMA_NUM_CH;
  dma_settings_normal[1].TCD->DADDR = &ADC0_SC1A;
  dma_settings_normal[1].TCD->DLASTSGA = 0;
  dma_settings_normal[1].TCD->NBYTES = 2;
  dma_settings_normal[1].TCD->DOFF = 0;
}

/*static*/ void ADC::StartConversionNormal()
{
  if (ADC_MODE_NORMAL != mode_) {
    StopDMA();
    Configure(kConfigNormal);

    SERIAL_PRINTLN("[ADC] StartConversionNormal");
    mode_ = ADC_MODE_NORMAL;

    dma0 = dma_settings_normal[0];
    dma0.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC0);
    dma0.disableOnCompletion();
#ifdef TU_ADC_ENABLE_DMA_INTERRUPT
    dma0.interruptAtCompletion();
    dma0.attachInterrupt(DMA0_ISR_NORMAL);
#endif
    dma1 = dma_settings_normal[1];
    dma1.triggerAtTransfersOf(dma0);
    dma1.triggerAtCompletionOf(dma0);

    StartDMA();
  }
}

/*static*/ void ADC::InitDMASettingsImmediate()
{
  int32_t buf_size = 4;
  dma_settings_immediate[0].TCD->SADDR = &ADC0_RA;
  dma_settings_immediate[0].TCD->SOFF = 0;
  dma_settings_immediate[0].TCD->ATTR = 0x101;
  dma_settings_immediate[0].TCD->NBYTES = 2;
  dma_settings_immediate[0].TCD->SLAST = 0;
  dma_settings_immediate[0].TCD->DADDR = &adcbuffer_0[0];
  dma_settings_immediate[0].TCD->DOFF = 2;
  dma_settings_immediate[0].TCD->DLASTSGA = -(2 * buf_size);
  dma_settings_immediate[0].TCD->BITER = buf_size;
  dma_settings_immediate[0].TCD->CITER = buf_size;

  uint16_t num_channels = 1;
  dma_settings_immediate[1].TCD->SADDR = &SCA_CHANNEL_ID[0];
  dma_settings_immediate[1].TCD->SOFF = 2;  // source increment each transfer (n bytes)
  dma_settings_immediate[1].TCD->ATTR = 0x101;
  dma_settings_immediate[1].TCD->SLAST = -(num_channels * 2);  // num ADC0 samples * 2
  dma_settings_immediate[1].TCD->BITER = num_channels;
  dma_settings_immediate[1].TCD->CITER = num_channels;
  dma_settings_immediate[1].TCD->DADDR = &ADC0_SC1A;
  dma_settings_immediate[1].TCD->DLASTSGA = 0;
  dma_settings_immediate[1].TCD->NBYTES = 2;
  dma_settings_immediate[1].TCD->DOFF = 0;
}

/*static*/ void ADC::StartConversionImmediate()
{
  if (ADC_MODE_IMMEDIATE != mode_) {
    StopDMA();

    SERIAL_PRINTLN("[ADC] StartConversionImmediate");
    mode_ = ADC_MODE_IMMEDIATE;

    dma0 = dma_settings_immediate[0];
    dma0.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC0);

    dma1 = dma_settings_immediate[1];
    dma1.triggerAtTransfersOf(dma0);
    dma1.triggerAtCompletionOf(dma0);

    StartDMA();
  }
}

/*static*/ void ADC::StopDMA()
{
  SERIAL_PRINTLN("[ADC] StopDMA");

  adc_.disableDMA();

  dma1.disable();
  dma1.clearComplete();

  dma0.disable();
  dma0.clearComplete();
  dma0.detachInterrupt();
  dma0.TCD->CSR &= ~(DMA_TCD_CSR_INTMAJOR | DMA_TCD_CSR_DREQ);

#ifdef TU_ADC_ENABLE_DMA_INTERRUPT
  dma0_complete = false;
#endif
}

/*static*/ void ADC::StartDMA()
{
  SERIAL_PRINTLN("[ADC] StartDMA");
  adc_.enableDMA();
  dma1.enable();
  dma1.triggerManual();
  dma0.enable();
}

/*static*/ void FASTRUN ADC::Update()
{
  if (ADC_MODE_NORMAL == mode_) {
#ifdef TU_ADC_ENABLE_DMA_INTERRUPT
    if (dma0_complete) {
      dma0_complete = false;
#else
    if (dma0.complete()) {
      dma0.clearComplete();
#endif

      // collect results from adcbuffer_0; there's DMA_BUF_SIZE = 16 samples in the buffer.
      uint32_t value;
      // / 4 = DMA_BUF_SIZE / DMA_NUM_CH
      value = (adcbuffer_0[0] + adcbuffer_0[4] + adcbuffer_0[8] + adcbuffer_0[12]) >> 2;
      update<ADC_CHANNEL_1>(value);

      value = (adcbuffer_0[1] + adcbuffer_0[5] + adcbuffer_0[9] + adcbuffer_0[13]) >> 2;
      update<ADC_CHANNEL_2>(value);

      value = (adcbuffer_0[2] + adcbuffer_0[6] + adcbuffer_0[10] + adcbuffer_0[14]) >> 2;
      update<ADC_CHANNEL_3>(value);

      value = (adcbuffer_0[3] + adcbuffer_0[7] + adcbuffer_0[11] + adcbuffer_0[15]) >> 2;
      update<ADC_CHANNEL_4>(value);

      dma0.enable();  // disableOnCompletion -> need to restart
    }
  } else {
    update<ADC_CHANNEL_1>((adcbuffer_0[0] + adcbuffer_0[1] + adcbuffer_0[2] + adcbuffer_0[3]) >> 2);
  }
}

/*static*/ void ADC::CalibratePitch(int32_t c2, int32_t c4)
{
  // This is the method used by the Mutable Instruments calibration and
  // extrapolates from two octaves. I guess an alternative would be to get the
  // lowest (-3v) and highest (+6v) and interpolate between them
  // *vague handwaving*
  if (c2 < c4) {
    int32_t scale = (24 * 128 * 4096L) / (c4 - c2);
    calibration_data_->pitch_cv_scale = scale;
  }
}

}  // namespace TU
