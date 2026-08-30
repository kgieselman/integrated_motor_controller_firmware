/*******************************************************************************
 * @file Led.hpp
 * @brief Debug LED driver — a thin GPIO wrapper, one instance per LED.
 *
 * The Rev A board carries three debug LEDs, all active high (a logic high on
 * the pin lights the LED):
 *
 *   - DEBUG_LED_0 — PC15 — conventionally the heartbeat / mode indicator
 *   - DEBUG_LED_1 — PC0  — conventionally the radio-link indicator
 *   - DEBUG_LED_2 — PC1  — conventionally the fault indicator
 *
 * Those roles are conventions owned by the application, not by this class.
 * The driver knows only how to drive one pin.
 *
 * Blink patterns deliberately live in the application. A driver that owned a
 * blink state machine would need a notion of time, and drivers in this
 * codebase depend on nothing but the HAL.
 *
 * Usage:
 * @code
 *   Led statusLed(DEBUG_LED_0_GPIO_Port, DEBUG_LED_0_Pin);
 *   statusLed.init();
 *   statusLed.toggle();
 * @endcode
 *
 * @note GPIO port clocks and pin modes are configured by MX_GPIO_Init().
 *       init() only forces a known initial state.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#pragma once

extern "C"
{
#include "stm32h5xx_hal.h"
}

#include <cstdint>

/*******************************************************************************
 * @brief Single debug LED on a GPIO pin.
 ******************************************************************************/
class Led final
{
public:
  /**
   * @brief Construct an LED bound to one GPIO pin.
   *
   * @param port       GPIO port the LED is wired to.
   * @param pin        GPIO pin mask.
   * @param activeLow  true if a logic low lights the LED. All Rev A debug
   *                   LEDs are active high, so this defaults to false; the
   *                   parameter exists so a future board revision does not
   *                   need a different driver.
   */
  Led(GPIO_TypeDef* port, uint16_t pin, bool activeLow = false);

  /**
   * @brief Drive the LED to a known off state.
   *
   * @return Always true. Present for consistency with the other drivers so
   *         that application init code can treat every driver identically.
   */
  bool init();

  /**
   * @brief Light the LED.
   */
  void on();

  /**
   * @brief Extinguish the LED.
   */
  void off();

  /**
   * @brief Invert the current state.
   */
  void toggle();

  /**
   * @brief Set the LED from a boolean.
   *
   * @param lit true to light the LED, false to extinguish it.
   */
  void set(bool lit);

  /**
   * @brief Query the commanded state.
   *
   * @return true if the LED was last commanded on.
   */
  bool isOn() const;

private:
  GPIO_TypeDef* m_port;      ///< GPIO port the LED is wired to.
  uint16_t      m_pin;       ///< GPIO pin mask.
  bool          m_activeLow; ///< true when a logic low lights the LED.
  bool          m_isOn;      ///< Last commanded state.
};

/* EOF -----------------------------------------------------------------------*/
