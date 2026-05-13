/*******************************************************************************
 * @file Motor.cpp
 * @brief DRV8874 motor driver implementation — IN1/IN2 control mode.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#include "Motor.hpp"

#include "AdcDma.hpp"
#include <algorithm>
#include <cmath>

// IPROPI sense resistor value (Ω) and DRV8874 current-sense gain (A/A)
static constexpr float kRIpropi   = 620.0f;  ///< External sense resistor (Ω)
static constexpr float kIpropGain = 2000.0f; ///< DRV8874 IPROPI current mirror ratio
static constexpr float kVdda      = 3.3f;    ///< ADC reference voltage (V)
static constexpr float kAdcMaxF   = 4095.0f; ///< 12-bit ADC full-scale (float)

Motor::Motor(TIM_HandleTypeDef* pwmTimer,
             uint32_t           in1Channel,
             uint32_t           in2Channel,
             GPIO_TypeDef*      pmodePort,
             uint16_t           pmodePin,
             GPIO_TypeDef*      faultPort,
             uint16_t           faultPin,
             uint8_t            ipropSlot)
  : m_pwmTimer(pwmTimer)
  , m_in1Channel(in1Channel)
  , m_in2Channel(in2Channel)
  , m_pmodePort(pmodePort)
  , m_pmodePin(pmodePin)
  , m_faultPort(faultPort)
  , m_faultPin(faultPin)
  , m_ipropSlot(ipropSlot)
  , m_timerPeriod(0U)
{}

bool Motor::init()
{
  // PMODE = high selects IN1/IN2 (independent half-bridge) mode on DRV8874.
  HAL_GPIO_WritePin(m_pmodePort, m_pmodePin, GPIO_PIN_SET);

  // Cache the auto-reload register so we can compute compare values later.
  m_timerPeriod = m_pwmTimer->Instance->ARR;

  // Start both channels and coast immediately (IN1=0, IN2=0 → Hi-Z).
  __HAL_TIM_SET_COMPARE(m_pwmTimer, m_in1Channel, 0U);
  __HAL_TIM_SET_COMPARE(m_pwmTimer, m_in2Channel, 0U);

  if (HAL_TIM_PWM_Start(m_pwmTimer, m_in1Channel) != HAL_OK)
  {
    return false;
  }
  if (HAL_TIM_PWM_Start(m_pwmTimer, m_in2Channel) != HAL_OK)
  {
    return false;
  }

  return true;
}

void Motor::set(float duty, Direction direction)
{
  // Clamp duty to valid range.
  duty = std::max(0.0f, std::min(1.0f, duty));

  uint32_t compare = static_cast<uint32_t>(duty * static_cast<float>(m_timerPeriod));

  if (direction == Direction::Forward)
  {
    // Forward: IN1=duty, IN2=0
    __HAL_TIM_SET_COMPARE(m_pwmTimer, m_in2Channel, 0U);
    __HAL_TIM_SET_COMPARE(m_pwmTimer, m_in1Channel, compare);
  }
  else
  {
    // Reverse: IN1=0, IN2=duty
    __HAL_TIM_SET_COMPARE(m_pwmTimer, m_in1Channel, 0U);
    __HAL_TIM_SET_COMPARE(m_pwmTimer, m_in2Channel, compare);
  }
}

void Motor::setSigned(float duty)
{
  if (duty >= 0.0f)
  {
    set(duty, Direction::Forward);
  }
  else
  {
    set(-duty, Direction::Reverse);
  }
}

void Motor::brake()
{
  // IN1=100%, IN2=100% → both low-side FETs on, motor terminals shorted.
  __HAL_TIM_SET_COMPARE(m_pwmTimer, m_in1Channel, m_timerPeriod);
  __HAL_TIM_SET_COMPARE(m_pwmTimer, m_in2Channel, m_timerPeriod);
}

void Motor::coast()
{
  // IN1=0, IN2=0 → outputs Hi-Z.
  __HAL_TIM_SET_COMPARE(m_pwmTimer, m_in1Channel, 0U);
  __HAL_TIM_SET_COMPARE(m_pwmTimer, m_in2Channel, 0U);
}

bool Motor::isFaulted() const
{
  // ~FAULT is active-low; GPIO_PIN_RESET means fault asserted.
  return (HAL_GPIO_ReadPin(m_faultPort, m_faultPin) == GPIO_PIN_RESET);
}

int32_t Motor::readCurrentMilliamps()
{
  // Snapshot the volatile slot into a local variable before doing arithmetic.
  // g_adcBuf[] is written by the ADC1 DMA circular scan; see AdcDma.hpp.
  uint16_t raw = g_adcBuf[m_ipropSlot];

  // V_IPROPI (V) = (raw / 4095) × VDDA
  // I_OUT    (A) = V_IPROPI × kIpropGain / kRIpropi
  float vIpropi = (static_cast<float>(raw) / kAdcMaxF) * kVdda;
  float iOut    = vIpropi * kIpropGain / kRIpropi;
  return static_cast<int32_t>(iOut * 1000.0f); // convert A → mA
}

/* EOF -----------------------------------------------------------------------*/
