/*******************************************************************************
 * @file Drivetrain.hpp
 * @brief High-level differential drivetrain controller.
 *
 * Owns two Motor instances (left and right) and two Encoder instances.
 * Provides a normalised drive interface and integrates encoder feedback for
 * basic open-loop speed commands. The shared ENABLE_MOTORS line (PA4 /
 * nSLEEP on both DRV8874s) is controlled centrally here.
 *
 * Usage:
 * @code
 *   // Left:  IN1=PA7 (TIM3_CH2), IN2=PA6 (TIM3_CH1)
 *   // Right: IN1=PB1 (TIM3_CH4), IN2=PB0 (TIM3_CH3)
 *   Motor    left (&htim3, TIM_CHANNEL_2, TIM_CHANNEL_1,
 *                  GPIOC, GPIO_PIN_3,          // PMODE
 *                  GPIOC, GPIO_PIN_1,          // ~FAULT (PC1/EXTI1)
 *                  kSlotLeftIpropi);           // ADC1 DMA slot (PA5, INP19)
 *   Motor    right(&htim3, TIM_CHANNEL_4, TIM_CHANNEL_3,
 *                  GPIOB, GPIO_PIN_13,         // PMODE
 *                  GPIOB, GPIO_PIN_2,          // ~FAULT (PB2/EXTI2)
 *                  kSlotRightIpropi);          // ADC1 DMA slot (PC5, INP8)
 *   Encoder  encL (&htim2, 2000, 100.0f);
 *   Encoder  encR (&htim8, 2000, 100.0f);
 *
 *   Drivetrain drive(left, right, encL, encR, GPIOA, GPIO_PIN_4);
 *   drive.init();
 *
 *   // In control loop (called at 100 Hz):
 *   drive.update();
 *   drive.drive(0.5f, 0.5f); // 50% forward both sides
 * @endcode
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#pragma once

#include "Motor.hpp"
#include "Encoder.hpp"

/**
 * @brief Differential drivetrain: two DRV8874 motors with quadrature encoder feedback.
 */
class Drivetrain
{
public:
  /**
   * @brief Construct the drivetrain.
   *
   * Takes references to pre-constructed Motor and Encoder objects.
   * The caller owns the underlying objects; the Drivetrain holds references only.
   *
   * @param left          Left Motor instance.
   * @param right         Right Motor instance.
   * @param encoderLeft   Left Encoder instance.
   * @param encoderRight  Right Encoder instance.
   * @param enablePort    GPIO port for the shared ENABLE_MOTORS (nSLEEP) pin.
   * @param enablePin     GPIO pin mask for ENABLE_MOTORS.
   */
  Drivetrain(Motor&        left,
             Motor&        right,
             Encoder&      encoderLeft,
             Encoder&      encoderRight,
             GPIO_TypeDef* enablePort,
             uint16_t      enablePin);

  /**
   * @brief Initialise all subsystems.
   *
   * Calls Motor::init() and Encoder::init() on all four instances.
   * Motors are left disabled until enable() is called.
   *
   * @return true if all inits succeed.
   */
  bool init();

  /**
   * @brief Enable the motor drivers (assert nSLEEP / ENABLE_MOTORS high).
   *
   * Both DRV8874s share this pin. Call before sending drive commands.
   */
  void enable();

  /**
   * @brief Disable the motor drivers (de-assert nSLEEP — motors coast).
   */
  void disable();

  /**
   * @brief Update encoder velocity estimates.
   *
   * Must be called at the rate passed to Encoder constructors (e.g. 100 Hz).
   * Typically called from a TIM period-elapsed interrupt or RTOS task.
   */
  void update();

  /**
   * @brief Command normalised wheel speeds.
   *
   * @param leftDuty  Left wheel speed in [-1.0, 1.0]. Positive = forward.
   * @param rightDuty Right wheel speed in [-1.0, 1.0]. Positive = forward.
   */
  void drive(float leftDuty, float rightDuty);

  /**
   * @brief Command an arcade-style manoeuvre.
   *
   * Mixes throttle and steering into left/right wheel speeds using standard
   * arcade mixing: left = throttle + steering, right = throttle - steering,
   * scaled down if either exceeds ±1 (ratio-preserving).
   *
   * @param throttle Forward/backward speed in [-1.0, 1.0].
   * @param steering Turn rate in [-1.0, 1.0]. Positive = turn right.
   */
  void arcade(float throttle, float steering);

  /**
   * @brief Apply braking to both motors simultaneously.
   */
  void brake();

  /**
   * @brief Coast both motors (PWM off, high-Z).
   */
  void coast();

  /**
   * @brief Return left encoder velocity in RPM.
   * @return Left shaft velocity (RPM).
   */
  float leftVelocityRpm() const;

  /**
   * @brief Return right encoder velocity in RPM.
   * @return Right shaft velocity (RPM).
   */
  float rightVelocityRpm() const;

  /**
   * @brief Check whether either motor is reporting a fault condition.
   *
   * @return true if left or right DRV8874 ~FAULT is asserted.
   */
  bool isFaulted() const;

private:
  Motor&        m_left;         ///< Left motor driver.
  Motor&        m_right;        ///< Right motor driver.
  Encoder&      m_encoderLeft;  ///< Left wheel quadrature encoder.
  Encoder&      m_encoderRight; ///< Right wheel quadrature encoder.
  GPIO_TypeDef* m_enablePort;   ///< GPIO port for ENABLE_MOTORS (nSLEEP).
  uint16_t      m_enablePin;    ///< GPIO pin mask for ENABLE_MOTORS.
  bool          m_enabled;      ///< true when motor drivers are active.
};

/* EOF -----------------------------------------------------------------------*/
