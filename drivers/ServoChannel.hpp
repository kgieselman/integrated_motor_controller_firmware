/*******************************************************************************
 * @file ServoChannel.hpp
 * @brief Single TIM PWM channel wrapper for RC servo control.
 *
 * Generates a standard hobby servo PWM signal (50 Hz, 1000–2000 µs pulse width).
 * Each channel maps to one TIM output compare unit; multiple ServoChannel
 * instances can share the same TIM (different channels) as long as the timer
 * period is set to 20 ms (50 Hz).
 *
 * Pin assignments (from integrated_motor_controller.ioc):
 *  - Servo 0: PA0 (TIM2 CH1)
 *  - Servo 1: PA1 (TIM2 CH2)
 *  - Servo 2: PA2 (TIM2 CH3)
 *
 * @note CubeMX must configure the TIM prescaler and ARR so that one tick = 1 µs
 *       (or adjust kTicksPerUs accordingly). At 250 MHz SYSCLK with prescaler
 *       249 → 1 MHz timer clock → 1 µs/tick, ARR = 19999 → 50 Hz.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#pragma once

extern "C"
{
#include "stm32h5xx_hal.h"
}

#include <cstdint>

/**
 * @brief RC servo PWM channel.
 */
class ServoChannel
{
public:
  static constexpr uint16_t kPulseMinUs = 1000U; ///< Minimum pulse width (µs)
  static constexpr uint16_t kPulseMaxUs = 2000U; ///< Maximum pulse width (µs)
  static constexpr uint16_t kPulseMidUs = 1500U; ///< Centre / neutral (µs)

  /**
   * @brief Construct a ServoChannel.
   *
   * @param htim        Pointer to the HAL TIM handle (50 Hz period configured).
   * @param channel     TIM channel (e.g. TIM_CHANNEL_1).
   * @param ticksPerUs  Timer ticks per microsecond. Defaults to 1 (assumes
   *                    prescaler set so timer clock = 1 MHz).
   */
  explicit ServoChannel(TIM_HandleTypeDef* htim,
                        uint32_t           channel,
                        uint32_t           ticksPerUs = 1U);

  /**
   * @brief Start PWM output and centre the servo.
   *
   * @return true on success, false if HAL_TIM_PWM_Start() fails.
   */
  bool init();

  /**
   * @brief Set the servo pulse width in microseconds.
   *
   * Values outside [kPulseMinUs, kPulseMaxUs] are clamped.
   *
   * @param pulseUs Desired pulse width in µs.
   */
  void setPulseUs(uint16_t pulseUs);

  /**
   * @brief Set servo position from a normalised value.
   *
   * @param position Value in [-1.0, 1.0].
   *                 -1.0 → kPulseMinUs, 0.0 → kPulseMidUs, 1.0 → kPulseMaxUs.
   *                 Clamped if out of range.
   */
  void setNormalised(float position);

  /**
   * @brief Return the currently commanded pulse width in microseconds.
   *
   * @return Current pulse width (µs).
   */
  uint16_t currentPulseUs() const;

private:
  TIM_HandleTypeDef* m_htim;          ///< HAL TIM handle.
  uint32_t           m_channel;       ///< TIM channel constant.
  uint32_t           m_ticksPerUs;    ///< Timer ticks per microsecond.
  uint16_t           m_currentPulseUs; ///< Currently commanded pulse width (µs).
};

/* EOF -----------------------------------------------------------------------*/
