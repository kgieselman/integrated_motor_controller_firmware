/*******************************************************************************
 * @file Drivetrain.cpp
 * @brief Differential drivetrain implementation.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#include "Drivetrain.hpp"

#include <algorithm>
#include <cmath>

Drivetrain::Drivetrain(Motor&        left,
                       Motor&        right,
                       Encoder&      encoderLeft,
                       Encoder&      encoderRight,
                       GPIO_TypeDef* enablePort,
                       uint16_t      enablePin)
  : m_left(left)
  , m_right(right)
  , m_encoderLeft(encoderLeft)
  , m_encoderRight(encoderRight)
  , m_enablePort(enablePort)
  , m_enablePin(enablePin)
  , m_enabled(false)
{}

bool Drivetrain::init()
{
  // Motors start disabled.
  HAL_GPIO_WritePin(m_enablePort, m_enablePin, GPIO_PIN_RESET);
  m_enabled = false;

  // Intentional: all four inits are called unconditionally (no short-circuit)
  // so the bringup console can report every failure in a single pass rather
  // than stopping at the first one. Do not "clean up" to early-exit on failure.
  bool ok = true;
  ok &= m_left.init();
  ok &= m_right.init();
  ok &= m_encoderLeft.init();
  ok &= m_encoderRight.init();
  return ok;
}

void Drivetrain::enable()
{
  HAL_GPIO_WritePin(m_enablePort, m_enablePin, GPIO_PIN_SET);
  m_enabled = true;
}

void Drivetrain::disable()
{
  coast();
  HAL_GPIO_WritePin(m_enablePort, m_enablePin, GPIO_PIN_RESET);
  m_enabled = false;
}

void Drivetrain::update()
{
  m_encoderLeft.update();
  m_encoderRight.update();
}

void Drivetrain::drive(float leftDuty, float rightDuty)
{
  if (!m_enabled)
  {
    return;
  }
  m_left.setSigned(leftDuty);
  m_right.setSigned(rightDuty);
}

void Drivetrain::arcade(float throttle, float steering)
{
  // Standard arcade mix: each side = throttle ± steering, clamped to [-1, 1].
  float leftDuty  = throttle + steering;
  float rightDuty = throttle - steering;

  // Scale down if either value exceeds ±1 (preserves ratio).
  float maxVal = std::max(std::abs(leftDuty), std::abs(rightDuty));
  if (maxVal > 1.0f)
  {
    leftDuty  /= maxVal;
    rightDuty /= maxVal;
  }

  drive(leftDuty, rightDuty);
}

void Drivetrain::brake()
{
  m_left.brake();
  m_right.brake();
}

void Drivetrain::coast()
{
  m_left.coast();
  m_right.coast();
}

float Drivetrain::leftVelocityRpm() const
{
  return m_encoderLeft.velocityRpm();
}

float Drivetrain::rightVelocityRpm() const
{
  return m_encoderRight.velocityRpm();
}

bool Drivetrain::isFaulted() const
{
  return m_left.isFaulted() || m_right.isFaulted();
}

/* EOF -----------------------------------------------------------------------*/
