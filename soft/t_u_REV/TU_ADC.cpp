#include "TU_ADC.h"

#include <algorithm>

#include "TU_gpio.h"
#include "src/util_misc.h"

// NOTES
// - There are two ADCs, but we might not be able to usefully map pins; it seems like only A2/A3
//   are ADC0/1 capable, so CV1 only. This would also require 2x2 DMA streams to handle the muxing.
// - There's a half-transfer interrupt, but no equivalent to DMA_TCD_CSR_DONE?
// - For scope use, there are comparators?
// - Many of the lengths are in bytes, even if a different pointer type is specified
//
// NOTE about continuous mode
// I tried to use scatter/gather/replaceSettingsOnCompletion (i.e. setting DLASTSGA to point to a
// new TCD) which worked great, and seems to lead to cleaner handling of multiple buffers. But
// there's a but -- as soon as this is activated, it stops the SPI DMA tx from working, and I was
// unable to figure out why (which isn't saying much). Neither chip erratas nor forums provided any
// insight, the DMA channel would complete but SPI0_TCR wouldn't change. Among the things tried were
// adding an rx channel (which is normally ignored) etc. pp. The only thing that seemed promising
// was upping the transfer size to 16/32 bits which fills the SPI0_PUSHR register differently so is
// completely different. So, we're just using a big buffer and polling the position.

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

static constexpr ADC::Config kConfigBuffered = {
    .resolution = 16,
    .averaging = 1,
    .sampling_speed = ADC_HIGH_SPEED_16BITS,
    .conversion_speed = ADC_HIGH_SPEED,
};

/*static*/ ADC::CalibrationData* ADC::calibration_data_ = nullptr;
/*static*/ ADC::ADC_MODE ADC::mode_ = ADC::ADC_MODE_INVALID;
/*static*/ ::ADC ADC::adc_;
/*static*/ size_t ADC::last_chunk_ = 0xffffffff;

/*static*/ uint32_t ADC::raw_[ADC_CHANNEL_LAST];
/*static*/ uint32_t ADC::smoothed_[ADC_CHANNEL_LAST];

// below: channel ids for the ADCx_SCA register: we have 4 inputs
// CV1 (17) = A3 = 0x49; CV2 (20) = A6 = 0x46; CV3 (19) = A5 = 0x4C; CV4 (18) = A4 = 0x4D
static constexpr uint16_t SCA_CHANNEL_ID[ADC_CHANNEL_LAST] = {0x49, 0x46, 0x4C, 0x4D};

DMAMEM static uint16_t adc_mux_buffer[ADC_CHANNEL_LAST] __attribute__((aligned(4)));
DMAMEM static uint16_t adc_dma_buffer[ADC::kDMABufferSize] __attribute__((aligned(4)));

static DMAChannel dma_channel_mux{false};  // buffer which holds the channel/pin IDs -> ADC0_SC1A
static DMAChannel dma_channel_adc{false};  // ADC0_RA -> buffer

// Maintain basic DMA settings for each mode for "easy" switching.
// This doesn't include some things like linking, which we have to setup manually.
// settings[0] = mux
// settings[1] = adc -> buffer
static DMASetting dma_settings_normal[2];
static DMASetting dma_settings_buffered[2];

#ifdef TU_ADC_ENABLE_DEBUG_ISR
#define TU_ADC_DEBUG_PIN 12
static void ADC_DMA_ISR()
{
  digitalWriteFast(TU_ADC_DEBUG_PIN, HIGH);
  dma_channel_adc.clearInterrupt();
  digitalWriteFast(TU_ADC_DEBUG_PIN, LOW);
}
#endif

/*static*/ void ADC::Init(CalibrationData* calibration_data)
{
  calibration_data_ = calibration_data;
  std::fill(raw_, raw_ + ADC_CHANNEL_LAST, 0);
  std::fill(smoothed_, smoothed_ + ADC_CHANNEL_LAST, 0);
  std::fill(adc_dma_buffer, adc_dma_buffer + kDMABufferSize, 0);

  adc_.setReference(ADC_REF_3V3);

  dma_channel_mux.begin(true);  // allocate the DMA channel
  dma_channel_adc.begin(true);  // allocate the DMA channel
  SERIAL_PRINTLN("[ADC] dma_channel_mux.channel=%x", dma_channel_mux.channel);
  SERIAL_PRINTLN("[ADC] dma_channel_adc.channel=%x", dma_channel_adc.channel);

  dma_channel_adc.triggerAtHardwareEvent(DMAMUX_SOURCE_ADC0);
#ifdef TU_ADC_ENABLE_DEBUG_ISR
  dma_channel_adc.attachInterrupt(ADC_DMA_ISR);
  pinMode(TU_ADC_DEBUG_PIN, OUTPUT);
#endif

  InitDMASettingsNormal();
  InitDMASettingsBuffered();

  StartConversionNormal();
  // StartConversionBuffered(ADC_CHANNEL_1);
}

/*static*/ void ADC::Configure(const Config& config)
{
  adc_.setResolution(config.resolution);
  adc_.setConversionSpeed(config.conversion_speed);
  adc_.setSamplingSpeed(config.sampling_speed);
  adc_.setAveraging(config.averaging);
}

// DMA/ADC à la
// https://forum.pjrc.com/threads/30171-Reconfigure-ADC-via-a-DMA-transfer-to-allow-multiple-Channel-Acquisition
// basically, this sets up two DMA channels and cycles through the four adc mux channels (until the
// buffer is full), resets, and so on; dma_channel_mux advances SCA_CHANNEL_ID somewhat like
// https://www.nxp.com/docs/en/application-note/AN4590.pdf but w/o the PDB.
/*static*/ void ADC::InitDMASettingsNormal()
{
  static constexpr unsigned int num_channels = ADC_CHANNEL_LAST;
  static constexpr unsigned int num_samples = 4;
  static constexpr unsigned int buffer_size = (num_channels * num_samples);
  static_assert(buffer_size <= kDMABufferSize, "DMA buffer too small for normal mode");

  dma_settings_normal[0].sourceBuffer(adc_mux_buffer, 2 * num_channels);
  dma_settings_normal[0].destination(*(volatile uint16_t*)&ADC0_SC1A);

  auto tcd = dma_settings_normal[1].TCD;
  tcd->SADDR = &ADC0_RA;
  tcd->SOFF = 0;
  tcd->ATTR = DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_DSIZE(1);
  tcd->NBYTES = 2;
  tcd->SLAST = 0;
  tcd->DADDR = adc_dma_buffer;
  tcd->DOFF = 2;
  tcd->CITER = tcd->BITER = buffer_size | DMA_TCD_BITER_ELINKYES_ELINK |
                            DMA_TCD_BITER_ELINKYES_LINKCH(dma_channel_mux.channel);
  tcd->CSR = DMA_TCD_CSR_MAJORELINK | DMA_TCD_CSR_MAJORLINKCH(dma_channel_mux.channel);
  tcd->CSR |= DMA_TCD_CSR_DREQ;
#ifdef TU_ADC_ENABLE_DEBUG_ISR
  tcd->CSR |= DMA_TCD_CSR_INTMAJOR;
#endif
  tcd->DLASTSGA = -(2 * buffer_size);
}

/*static*/ void ADC::StartConversionNormal()
{
  if (ADC_MODE_NORMAL != mode_) {
    SERIAL_PRINTLN("[ADC] StartConversionNormal");

    StopDMA();
    Configure(kConfigNormal);

    std::copy(SCA_CHANNEL_ID, SCA_CHANNEL_ID + ADC_CHANNEL_LAST, adc_mux_buffer);
    StartDMA(ADC_MODE_NORMAL, dma_settings_normal);
  }
}

/*static*/ void ADC::InitDMASettingsBuffered()
{
  unsigned int num_channels = 1;
  unsigned int num_samples = 4;

  auto& mux = dma_settings_buffered[0];
  mux.sourceBuffer(adc_mux_buffer, 2 * num_channels);
  mux.destination(*(volatile uint16_t*)&ADC0_SC1A);

  // These are a bit more complex since we want to link them, as well as use the copy-on-completion
  auto tcd = dma_settings_buffered[1].TCD;
  tcd->SADDR = &ADC0_RA;
  tcd->SOFF = 0;
  tcd->ATTR = DMA_TCD_ATTR_SSIZE(1) | DMA_TCD_ATTR_DSIZE(1);
  tcd->NBYTES = 2;
  tcd->SLAST = 0;
  tcd->DADDR = adc_dma_buffer;
  tcd->DOFF = 2;
  tcd->CITER = tcd->BITER = num_samples | DMA_TCD_BITER_ELINKYES_LINKCH(dma_channel_mux.channel) |
                            DMA_TCD_BITER_ELINKYES_ELINK;
  tcd->CSR = DMA_TCD_CSR_MAJORLINKCH(dma_channel_mux.channel) | DMA_TCD_CSR_MAJORELINK;
#ifdef TU_ADC_ENABLE_DEBUG_ISR
  tcd->CSR |= DMA_TCD_CSR_INTMAJOR;
#endif
  tcd->DLASTSGA = -(2 * num_samples);
}

/*static*/ void ADC::StartConversionBuffered(ADC_CHANNEL channel)
{
  if (ADC_MODE_BUFFERED != mode_) {
    SERIAL_PRINTLN("[ADC] StartConversionBuffered");

    StopDMA();
    Configure(kConfigBuffered);

    adc_mux_buffer[0] = SCA_CHANNEL_ID[channel];
    StartDMA(ADC_MODE_BUFFERED, dma_settings_buffered);
  }
}

/*static*/ void ADC::StopDMA()
{
  if (ADC_MODE_INVALID != mode_) {
    SERIAL_PRINTLN("[ADC] StopDMA (mode=%x)", mode_);
    adc_.disableDMA();
    dma_channel_mux.disable();
    dma_channel_adc.disable();
    dma_channel_adc.clearComplete();

    mode_ = ADC_MODE_INVALID;
  }
}

/*static*/ void ADC::StartDMA(ADC_MODE mode, DMASetting* dma_settings)
{
  mode_ = mode;
  SERIAL_PRINTLN("[ADC] StartDMA (mode=%x)", mode_);

  dma_channel_mux = dma_settings[0];
  dma_channel_adc = dma_settings[1];

  // We have to ensure DMA is started in the correct order, so that SCA is written first. Otherwise,
  // the channel that reads from the ADC will read the "old" value and the values are out of order
  // (see older revisions of this file).
  adc_.enableDMA();
  dma_channel_mux.enable();
  dma_channel_mux.triggerManual();
  dma_channel_adc.enable();
}

/*static*/ void FASTRUN ADC::Update()
{
  if (ADC_MODE_NORMAL == mode_) {
    if (dma_channel_adc.complete()) {
      dma_channel_adc.clearComplete();
      // Update channel values from adc_dma_buffer; there's 4 samples per channel in the buffer so
      // we can average the values.
      uint32_t value;
      value = (adc_dma_buffer[0] + adc_dma_buffer[4] + adc_dma_buffer[8] + adc_dma_buffer[12]) >> 2;
      update<ADC_CHANNEL_1>(value);

      value = (adc_dma_buffer[1] + adc_dma_buffer[5] + adc_dma_buffer[9] + adc_dma_buffer[13]) >> 2;
      update<ADC_CHANNEL_2>(value);

      value =
          (adc_dma_buffer[2] + adc_dma_buffer[6] + adc_dma_buffer[10] + adc_dma_buffer[14]) >> 2;
      update<ADC_CHANNEL_3>(value);

      value =
          (adc_dma_buffer[3] + adc_dma_buffer[7] + adc_dma_buffer[11] + adc_dma_buffer[15]) >> 2;
      update<ADC_CHANNEL_4>(value);

      dma_channel_adc.enable();  // disableOnCompletion -> need to restart
    }
  } else {
    auto src = adc_dma_buffer;  // + kDMAChunkSize;
    update<ADC_CHANNEL_1>((src[0] + src[1] + src[2] + src[3]) >> 2);
  }
}

/*static*/ size_t ADC::ReadChunk(uint16_t* buffer)
{
  auto ptr = (const uint16_t*)dma_channel_adc.TCD->DADDR;

  auto chunk =
      ((((uint32_t)ptr - (uint32_t)adc_dma_buffer) / kDMAChunkSize) + kDMAMaxChunkCount / 2) %
      kDMAMaxChunkCount;
  if (chunk != last_chunk_) {
    memcpy(buffer, adc_dma_buffer + chunk * kDMAChunkSize * 2, kDMAChunkSize);
    last_chunk_ = chunk;
  }
  return chunk;
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
