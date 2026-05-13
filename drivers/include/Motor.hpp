/*******************************************************************************
 * @file Motor.hpp
 * @brief DRV8874 H-bridge motor driver wrapper — IN1/IN2 control mode.
 *
 * Wraps one DRV8874 channel operating in IN1/IN2 (independent half-bridge) mode.
 * Each motor requires:
 *  - Two PWM-capable TIM channels on the same timer (IN1 and IN2 pins)
 *  - One GPIO output (PMODE pin — driven high to select IN1/IN2 mode)
 *  - One GPIO input  (~FAULT pin, active-low fault indicator)
 *  - One ADC slot index (IPROPI — read from the shared ADC1 DMA buffer, see AdcDma.hpp)
 *  - Shared GPIO output (ENABLE_MOTORS / nSLEEP — PA5, shared between left and right)
 *
 * Pin assignments (from board_pins.h):
 *  - Left  motor: IN1=PA7 (TIM3_CH2), IN2=PA6 (TIM3_CH1), PMODE=PA3, FAULT=PC3 (EXTI3), IPROPI=PA4 (ADC1 INP18, slot 0)
 *  - Right motor: IN1=PB1 (TIM3_CH4), IN2=PB0 (TIM3_CH3), PMODE=PB10, FAULT=PC7 (EXTI7), IPROPI=PC5 (ADC1 INP8,  slot 1)
 *  - Shared ENABLE (nSLEEP): PA5 — must be driven high before either motor is used
 *
 * IN1/IN2 truth table (PMODE = high on DRV8874):
 *  - IN1=PWM, IN2=0   → Forward (speed proportional to duty)
 *  - IN1=0,   IN2=PWM → Reverse (speed proportional to duty)
 *  - IN1=100%, IN2=100% → Brake (low-side FETs both on, terminals shorted)
 *  - IN1=0,   IN2=0   → Coast  (outputs Hi-Z)
 *
 * @note Compile with -fno-exceptions. All error states are returned as bool or
 *       exposed via isFaulted().
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#pragma once

extern "C"
{
#include "stm32h5xx_hal.h"
}

#include "AdcDma.hpp"
#include <cstdint>

/**
 * @brief Thin C++ wrapper around one DRV8874 channel in IN1/IN2 mode.
 *
 * Both IN1 and IN2 are driven by independent TIM PWM channels on the same
 * timer peripheral. Speed is controlled by the active channel's duty cycle;
 * the inactive channel is held at 0. Brake and coast are both available.
 */
class Motor
{
public:
  /**
   * @brief Motor direction constants.
   */
  enum class Direction : uint8_t
  {
    Forward = 0, ///< IN1=duty, IN2=0
    Reverse = 1  ///< IN1=0,   IN2=duty
  };

  /**
   * @brief Default IPROPI sense resistor (Ω).
   *
   * Board value: 100 Ω + 1.5 kΩ series chain = 1 600 Ω.
   * Pass a different value to the constructor to tune for real-world tolerance.
   */
  static constexpr float kDefaultRIpropi = 1600.0f;

  /**
   * @brief Default DRV8874 IPROPI current mirror ratio (A/A).
   *
   * Fixed by the DRV8874 silicon: 1 : 2000.
   * Exposed here so it can be overridden if a different sense topology is used.
   */
  static constexpr float kDefaultIpropGain = 2000.0f;

  /**
   * @brief Construct a Motor driver instance.
   *
   * Both IN1 and IN2 must be PWM-output channels of the same TIM peripheral.
   * Does not start PWM — call init() after HAL peripheral init is complete.
   *
   * @param pwmTimer      Pointer to the HAL TIM handle (shared by IN1 and IN2).
   * @param in1Channel    TIM channel constant for IN1 (e.g. TIM_CHANNEL_2).
   * @param in2Channel    TIM channel constant for IN2 (e.g. TIM_CHANNEL_1).
   * @param pmodePort     GPIO port for the PMODE pin.
   * @param pmodePin      GPIO pin mask for the PMODE pin.
   * @param faultPort     GPIO port for the ~FAULT input pin.
   * @param faultPin      GPIO pin mask for the ~FAULT input pin.
   * @param ipropSlot     Slot index into g_adcBuf[] for this motor's IPROPI channel.
   *                      Use kSlotLeftIpropi or kSlotRightIpropi from AdcDma.hpp.
   * @param rIpropi       IPROPI sense resistor value (Ω). Defaults to kDefaultRIpropi.
   *                      Adjust to compensate for real-world resistor tolerances.
   * @param ipropGain     DRV8874 current mirror ratio (A/A). Defaults to kDefaultIpropGain.
   */
  Motor(TIM_HandleTypeDef* pwmTimer,
        uint32_t           in1Channel,
        uint32_t           in2Channel,
        GPIO_TypeDef*      pmodePort,
        uint16_t           pmodePin,
        GPIO_TypeDef*      faultPort,
        uint16_t           faultPin,
        uint8_t            ipropSlot,
        float              rIpropi   = kDefaultRIpropi,
        float              ipropGain = kDefaultIpropGain);

  /**
   * @brief Initialise the motor driver.
   *
   * Drives PMODE high (selects IN1/IN2 mode on DRV8874), starts PWM on both
   * IN1 and IN2 channels, and coasts the motor (both channels at 0% duty).
   * Call once after MX_TIMx_Init() has run.
   *
   * @return true on success, false if either HAL_TIM_PWM_Start() call fails.
   */
  bool init();

  /**
   * @brief Set motor speed and direction.
   *
   * The active channel is set to @p duty; the inactive channel is held at 0.
   *
   * @param duty      Normalised duty cycle in [0.0, 1.0]. Clamped if out of range.
   * @param direction Forward or Reverse.
   */
  void set(float duty, Direction direction);

  /**
   * @brief Set motor speed with sign encoding direction.
   *
   * Positive values → forward, negative → reverse.
   *
   * @param duty Signed duty cycle in [-1.0, 1.0]. Clamped if out of range.
   */
  void setSigned(float duty);

  /**
   * @brief Apply active braking.
   *
   * Drives both IN1 and IN2 to 100% duty, shorting the motor terminals
   * through the low-side FETs.
   */
  void brake();

  /**
   * @brief Coast the motor (both outputs Hi-Z).
   *
   * Drives both IN1 and IN2 to 0% duty.
   */
  void coast();

  /**
   * @brief Check the DRV8874 ~FAULT pin.
   *
   * Fault conditions include overcurrent, overtemperature, and undervoltage.
   * The ~FAULT pin is active-low; this method returns true when fault is asserted.
   *
   * @return true if a fault is active.
   */
  bool isFaulted() const;

  /**
   * @brief Read the IPROPI current-sense value from the DMA buffer in milliamps.
   *
   * Reads g_adcBuf[m_ipropSlot] — populated continuously by the ADC1 DMA scan.
   * DRV8874 with R_IPROPI = 620 Ω: V_IPROPI = I_OUT × 620 / 2000 → ~412 mV/A.
   * Assumes VDDA = 3.3 V and 12-bit ADC.
   *
   * @return Estimated motor current in milliamps.
   *         Returns 0 if the DMA buffer has not yet been populated (value == 0).
   */
  int32_t readCurrentMilliamps();

private:
  TIM_HandleTypeDef* m_pwmTimer;   ///< HAL TIM handle (shared by IN1 and IN2).
  uint32_t           m_in1Channel; ///< TIM channel constant for IN1.
  uint32_t           m_in2Channel; ///< TIM channel constant for IN2.
  GPIO_TypeDef*      m_pmodePort;  ///< GPIO port for the PMODE pin.
  uint16_t           m_pmodePin;   ///< GPIO pin mask for the PMODE pin.
  GPIO_TypeDef*      m_faultPort;  ///< GPIO port for the ~FAULT input pin.
  uint16_t           m_faultPin;   ///< GPIO pin mask for the ~FAULT input pin.
  uint8_t            m_ipropSlot;  ///< Index into g_adcBuf[] for this motor's IPROPI.
  float              m_rIpropi;    ///< IPROPI sense resistor value (Ω).
  float              m_ipropGain;  ///< DRV8874 current mirror ratio (A/A).

  uint32_t           m_timerPeriod; ///< Cached ARR value for duty cycle scaling.
};

/* EOF -----------------------------------------------------------------------*/
