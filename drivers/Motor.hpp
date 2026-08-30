/*******************************************************************************
 * @file Motor.hpp
 * @brief DRV8874 H-bridge motor driver wrapper — IN1/IN2 control mode.
 *
 * Wraps one DRV8874 channel operating in IN1/IN2 (independent half-bridge) mode.
 * Each motor requires:
 *  - Two PWM-capable TIM channels on the same timer (IN1 and IN2 pins)
 *  - One GPIO output (PMODE pin — driven high to select IN1/IN2 mode)
 *  - One GPIO output (ENABLE / nSLEEP pin — per channel on this board)
 *  - One GPIO input  (~FAULT pin, active-low fault indicator, on an EXTI line)
 *  - One ADC slot index (IPROPI — read from the shared ADC1 DMA buffer, see AdcDma.hpp)
 *
 * Pin assignments (authoritative source: cubemx/integrated_motor_controller.ioc):
 *  - Motor 0: IN1=PA7 (TIM3_CH2), IN2=PA6 (TIM3_CH1), PMODE=PA3, ENABLE=PA5,
 *             FAULT=PC3 (EXTI3),  IPROPI=PA4 (ADC1 INP18, slot 0)
 *  - Motor 1: IN1=PB1 (TIM3_CH4), IN2=PB0 (TIM3_CH3), PMODE=PB10, ENABLE=PB2,
 *             FAULT=PC7 (EXTI7),  IPROPI=PC5 (ADC1 INP8,  slot 1)
 *
 * @note ENABLE (nSLEEP) is per channel on Rev A, not shared. Earlier revisions
 *       of this header described a single shared enable on PA4 — PA4 is the
 *       motor 0 current-sense input, and driving it as an output destroys the
 *       current sense.
 *
 * IN1/IN2 truth table (PMODE = high on DRV8874):
 *  - IN1=PWM, IN2=0     → Forward (speed proportional to duty)
 *  - IN1=0,   IN2=PWM   → Reverse (speed proportional to duty)
 *  - IN1=100%, IN2=100% → Brake (low-side FETs both on, terminals shorted)
 *  - IN1=0,   IN2=0     → Coast  (outputs Hi-Z)
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
class Motor final
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

  /// Sentinel returned by readCurrentMilliamps() when no measurement exists yet.
  static constexpr int32_t kCurrentUnavailable = -1;

  /**
   * @brief Default IPROPI sense resistor (Ω).
   *
   * Board value: 1.6 kΩ (100 Ω + 1.5 kΩ series chain), confirmed against the
   * schematic on 2026-08-30. Pass a different value to the constructor to
   * compensate for real-world resistor tolerance without recompiling.
   *
   * With the DRV8874's fixed 1:2000 current mirror this gives
   * 1600 / 2000 = **0.8 V per amp** at the ADC input.
   */
  static constexpr float kDefaultRIpropi = 1600.0f;

  /**
   * @brief Highest current the sense chain can represent, in milliamps.
   *
   * At 0.8 V/A a 3.3 V ADC reference saturates at 3.3 / 0.8 ≈ 4.125 A. The
   * DRV8874 itself handles considerably more than that, so a hard stall pins
   * the reading at full scale rather than reporting its true magnitude.
   */
  static constexpr int32_t kCurrentFullScaleMa = 4125;

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
   * @param enablePort    GPIO port for this channel's ENABLE / nSLEEP pin.
   * @param enablePin     GPIO pin mask for this channel's ENABLE / nSLEEP pin.
   * @param faultPort     GPIO port for the ~FAULT input pin.
   * @param faultPin      GPIO pin mask for the ~FAULT input pin.
   * @param ipropSlot     Slot index into g_adcBuf[] for this motor's IPROPI channel.
   *                      Use kSlotLeftIpropi or kSlotRightIpropi from AdcDma.hpp.
   *                      An out-of-range index is replaced with kSlotLeftIpropi.
   * @param rIpropi       IPROPI sense resistor value (Ω). Defaults to kDefaultRIpropi.
   * @param ipropGain     DRV8874 current mirror ratio (A/A). Defaults to kDefaultIpropGain.
   */
  Motor(TIM_HandleTypeDef* pwmTimer,
        uint32_t           in1Channel,
        uint32_t           in2Channel,
        GPIO_TypeDef*      pmodePort,
        uint16_t           pmodePin,
        GPIO_TypeDef*      enablePort,
        uint16_t           enablePin,
        GPIO_TypeDef*      faultPort,
        uint16_t           faultPin,
        uint8_t            ipropSlot,
        float              rIpropi   = kDefaultRIpropi,
        float              ipropGain = kDefaultIpropGain);

  /**
   * @brief Initialise the motor driver.
   *
   * Drives PMODE high (selects IN1/IN2 mode), holds ENABLE low (bridge asleep),
   * starts PWM on both IN1 and IN2 channels at 0% duty. The motor stays asleep
   * until enable() is called, so a init() alone can never move anything.
   *
   * @return true on success, false if either HAL_TIM_PWM_Start() call fails.
   */
  bool init();

  /**
   * @brief Wake the DRV8874 (drive nSLEEP high).
   *
   * No output is produced until a duty cycle is commanded.
   */
  void enable();

  /**
   * @brief Sleep the DRV8874 (drive nSLEEP low) and command zero duty.
   *
   * The strongest available "off": the bridge is powered down rather than
   * merely commanded to 0%. Safe to call repeatedly.
   */
  void disable();

  /**
   * @brief Report whether the bridge is currently awake.
   *
   * @return true if enable() was the last of enable()/disable() to be called.
   */
  bool isEnabled() const;

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
   * Drives both IN1 and IN2 to a full 100% duty, shorting the motor terminals
   * through the low-side FETs.
   *
   * @note Braking requires the bridge to be awake. Calling this while disabled
   *       leaves the motor coasting, not braked.
   */
  void brake();

  /**
   * @brief Coast the motor (both outputs Hi-Z).
   *
   * Drives both IN1 and IN2 to 0% duty. Leaves the bridge awake — use
   * disable() to also drop nSLEEP.
   */
  void coast();

  /**
   * @brief Check whether a DRV8874 fault is or has been active.
   *
   * Fault conditions include overcurrent, overtemperature and undervoltage.
   * Returns true if the ~FAULT pin is currently asserted (active low) OR if a
   * fault edge was latched by onFaultInterrupt() since the last clearFault().
   * The latch matters because the DRV8874 retries automatically: a transient
   * overcurrent can come and go entirely between two polls.
   *
   * @return true if a fault is active or was latched.
   */
  bool isFaulted() const;

  /**
   * @brief Latch a fault from the ~FAULT EXTI handler.
   *
   * Wire to HAL_GPIO_EXTI_Callback for this motor's fault pin.
   *
   * @note ISR-safe. Calls no FreeRTOS API, so it is not constrained to the
   *       kernel-maskable priority band — but keep it there anyway for
   *       uniformity with the rest of the interrupt policy.
   */
  void onFaultInterrupt();

  /**
   * @brief Clear the latched fault.
   *
   * Does not clear a fault that is still physically asserted — isFaulted()
   * continues to report true while the pin is low.
   */
  void clearFault();

  /**
   * @brief Read the IPROPI current-sense value from the DMA buffer in milliamps.
   *
   * Reads g_adcBuf[m_ipropSlot] — populated continuously by the ADC1 DMA scan.
   * DRV8874 current mirror is 1:2000 and the sense resistor is m_rIpropi, so
   * with the board's 1.6 kΩ the scale factor is 0.8 V/A.
   *
   * @return Estimated motor current in milliamps, or kCurrentUnavailable if the
   *         ADC DMA scan has not been started. Distinguishing the two matters:
   *         an unstarted scan reads as a perfectly plausible 0 mA.
   *
   * @note Saturates at kCurrentFullScaleMa (≈4.1 A) — see that constant. Any
   *       stall-detection logic must treat a reading at or near full scale as
   *       "at least this much", not as a measurement.
   */
  int32_t readCurrentMilliamps();

private:
  /**
   * @brief Convert a normalised duty cycle to a timer compare value.
   *
   * Full scale is ARR + 1, not ARR: a compare equal to ARR still leaves one
   * timer tick low each period, so 1.0 would not be a continuous output and
   * brake() would not be a true low-side short.
   *
   * @param duty Normalised duty in [0.0, 1.0]; clamped.
   * @return Compare register value.
   */
  uint32_t dutyToCompare(float duty) const;

  TIM_HandleTypeDef* m_pwmTimer;   ///< HAL TIM handle (shared by IN1 and IN2).
  uint32_t           m_in1Channel; ///< TIM channel constant for IN1.
  uint32_t           m_in2Channel; ///< TIM channel constant for IN2.
  GPIO_TypeDef*      m_pmodePort;  ///< GPIO port for the PMODE pin.
  uint16_t           m_pmodePin;   ///< GPIO pin mask for the PMODE pin.
  GPIO_TypeDef*      m_enablePort; ///< GPIO port for the ENABLE / nSLEEP pin.
  uint16_t           m_enablePin;  ///< GPIO pin mask for the ENABLE / nSLEEP pin.
  GPIO_TypeDef*      m_faultPort;  ///< GPIO port for the ~FAULT input pin.
  uint16_t           m_faultPin;   ///< GPIO pin mask for the ~FAULT input pin.
  uint8_t            m_ipropSlot;  ///< Index into g_adcBuf[]; clamped by the constructor.
  float              m_rIpropi;    ///< IPROPI sense resistor value (Ω).
  float              m_ipropGain;  ///< DRV8874 current mirror ratio (A/A).

  uint32_t           m_timerPeriod;  ///< Cached ARR value for duty cycle scaling.
  bool               m_enabled;      ///< true when nSLEEP is driven high.
  volatile bool      m_faultLatched; ///< Set by onFaultInterrupt(); cleared by clearFault().
};

/* EOF -----------------------------------------------------------------------*/
