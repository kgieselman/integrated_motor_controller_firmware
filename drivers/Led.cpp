/*******************************************************************************
 * @file Led.cpp
 * @brief Debug LED driver — a thin GPIO wrapper, one instance per LED.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#include "Led.hpp"

/* Construction --------------------------------------------------------------*/

Led::Led(GPIO_TypeDef* port, uint16_t pin, bool activeLow)
    : m_port(port)
    , m_pin(pin)
    , m_activeLow(activeLow)
    , m_isOn(false)
{}

/* Public methods ------------------------------------------------------------*/

bool Led::init()
{
  off();
  return true;
}

void Led::on()
{
  set(true);
}

void Led::off()
{
  set(false);
}

void Led::toggle()
{
  set(!m_isOn);
}

void Led::set(bool lit)
{
  if (m_port == nullptr)
  {
    return;
  }

  // Resolve the logical state to a pin level once, here, so that every other
  // method stays free of polarity handling.
  const GPIO_PinState level = (lit != m_activeLow) ? GPIO_PIN_SET : GPIO_PIN_RESET;

  HAL_GPIO_WritePin(m_port, m_pin, level);
  m_isOn = lit;
}

bool Led::isOn() const
{
  return m_isOn;
}

/* EOF -----------------------------------------------------------------------*/
