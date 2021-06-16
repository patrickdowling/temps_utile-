#include "TU_digital_inputs.h"

#include <Arduino.h>

#include <algorithm>

#include "TU_gpio.h"
#include "src/util_misc.h"

namespace TU {
#ifdef TU_TR_DEBUG_SERIAL
#define TR_SERIAL_PRINTLN(...) SERIAL_PRINTLN("[TRx] " __VA_ARGS__)
#else
#define TR_SERIAL_PRINTLN(...) \
  do {                         \
  } while (0)
#endif

/*static*/ uint32_t DigitalInputs::clocked_mask_ = 0;
/*static*/ uint8_t DigitalInputs::global_divisor_TR1_ = 0;
/*static*/ bool DigitalInputs::master_clock_TR1_;

/*static*/ volatile uint32_t DigitalInputs::clocked_[DIGITAL_INPUT_LAST] = {0};

void FASTRUN tr1_ISR()
{
  DigitalInputs::clock<DIGITAL_INPUT_1>();
}  // main clock

void FASTRUN tr2_ISR()
{
  DigitalInputs::clock<DIGITAL_INPUT_2>();
}

void DigitalInputs::Clear()
{
  clocked_mask_ = 0x0;
}

static constexpr struct {
  uint8_t pin;
  int mode;
  void (*isr_fn)();
} digital_input_pins[DIGITAL_INPUT_LAST] = {
    {TR1, FALLING, tr1_ISR},
    {TR2, FALLING, tr2_ISR},
};

/*static*/
void DigitalInputs::Init()
{
  clocked_mask_ = 0x0;
  std::fill(clocked_, clocked_ + DIGITAL_INPUT_LAST, 0);
  global_divisor_TR1_ = 0x0;

  for (auto pin : digital_input_pins) pinMode(pin.pin, TU_GPIO_TRx_PINMODE);
  EnableInterrupts();
}

/*static*/
void DigitalInputs::Scan()
{
  clocked_mask_ = ScanInput<DIGITAL_INPUT_1>() | ScanInput<DIGITAL_INPUT_2>();
}

/*static*/ void DigitalInputs::EnableInterrupts()
{
  for (auto &pin : digital_input_pins) {
    TR_SERIAL_PRINTLN("pin %d isr_fn=%p", pin.pin, pin.isr_fn);
    attachInterrupt(pin.pin, pin.isr_fn, FALLING);
  }
}

/*static*/ void DigitalInputs::EnableDMARequest(DigitalInput digital_input)
{
  auto enable_pin = digital_input_pins[digital_input].pin;
  auto disable_pin =
      digital_input_pins[DIGITAL_INPUT_1 == digital_input ? DIGITAL_INPUT_2 : DIGITAL_INPUT_1].pin;
  {
    auto pcr = portConfigRegister(disable_pin);
    *pcr &= ~PORT_PCR_IRQC_MASK;

    TR_SERIAL_PRINTLN("pin %d  PCR=%08lx", disable_pin, *pcr);
  }

  {
    auto pcr = portConfigRegister(enable_pin);
    uint32_t config = *pcr;
    config &= ~PORT_PCR_IRQC_MASK;
    config |= PORT_PCR_IRQC(0x2);  // 0010 DMA request on falling edge.
    *pcr = config;

    TR_SERIAL_PRINTLN("pin %d  PCR=%08lx", enable_pin, *pcr);
  }
}

}  // namespace TU
