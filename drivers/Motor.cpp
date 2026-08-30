/*******************************************************************************
 * @file Motor.cpp
 * @brief DRV8874 motor driver implementation — IN1/IN2 control mode.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#include "Motor.hpp"

#include "AdcDma.hpp"
#include <algorithm>

static constexpr float kVdda    = 3.3f;    ///< ADC reference voltage (V)
static constexpr float kAdcMaxF = 4095.0f; ///< 12-bit ADC full-scale (float)

Motor::Motor(TIM_HandleTypeDef* pwmTimer,
             uint32_t           in1Channel,
             uint32_t           in2Channel,
             GPIO_TypeDef*      pmodePort,
             uint16_t           pmodePin,
             GPIO_TypeDef*      enablePort,
             uint16_t           enablePin,
             GPIO_TypeDef*      faultPort,
             uint16_t           faultPin,
             uint8_t            ipropSlot,
             float              rIpropi,
             float              ipropGain)
  : m_pwmTimer(pwmTimer)
  , m_in1Channel(in1Channel)
  , m_in2Channel(in2Channel)
  , m_pmodePort(pmodePort)
  , m_pmodePin(pmodePin)
  , m_enablePort(enablePort)
  , m_enablePin(enablePin)
  , m_faultPort(faultPort)
  , m_faultPin(faultPin)
  , m_ipropSlot((ipropSlot < ADC_DMA_NUM_CHANNELS) ? ipropSlot : kSlotLeftIpropi)
  , m_rIpropi(rIpropi)
  , m_ipropGain(ipropGain)
  , m_timerPeriod(0U)
  , m_enabled(false)
  , m_faultLatched(false)
{
  // An out-of-range slot would index past g_adcBuf[]; fall back to a real slot
  // rather than reading whatever follows the array in memory.
}

bool Motor::init()
{
  // PMODE = high selects IN1/IN2 (independent half-bridge) mode on DRV8874.
  HAL_GPIO_WritePin(m_pmodePort, m_pmodePin, GPIO_PIN_SET);

  // nSLEEP low: the bridge stays powered down until enable() is called, so
  // init() on its own can never produce motion.
  HAL_GPIO_WritePin(m_enablePort, m_enablePin, GPIO_PIN_RESET);
  m_enabled = false;

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

void Motor::enable()
{
  HAL_GPIO_WritePin(m_enablePort, m_enablePin, GPIO_PIN_SET);
  m_enabled = true;
}

void Motor::disable()
{
  // Command zero duty first, then drop nSLEEP — in that order, so the bridge is
  // never asked to wake into a live duty cycle on the next enable().
  coast();
  HAL_GPIO_WritePin(m_enablePort, m_enablePin, GPIO_PIN_RESET);
  m_enabled = false;
}

bool Motor::isEnabled() const
{
  return m_enabled;
}

void Motor::set(float duty, Direction direction)
{
  const uint32_t compare = dutyToCompare(duty);

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
  // Full scale is ARR + 1: a compare of exactly ARR leaves one tick low every
  // period, which is a very short coast rather than a continuous short.
  const uint32_t full = m_timerPeriod + 1U;
  __HAL_TIM_SET_COMPARE(m_pwmTimer, m_in1Channel, full);
  __HAL_TIM_SET_COMPARE(m_pwmTimer, m_in2Channel, full);
}

void Motor::coast()
{
  // IN1=0, IN2=0 → outputs Hi-Z.
  __HAL_TIM_SET_COMPARE(m_pwmTimer, m_in1Channel, 0U);
  __HAL_TIM_SET_COMPARE(m_pwmTimer, m_in2Channel, 0U);
}

bool Motor::isFaulted() const
{
  // ~FAULT is active-low; GPIO_PIN_RESET means fault asserted right now.
  const bool pinAsserted =
      (HAL_GPIO_ReadPin(m_faultPort, m_faultPin) == GPIO_PIN_RESET);

  // The DRV8874 retries automatically, so a transient fault can appear and
  // clear entirely between two polls. The EXTI latch catches those.
  return pinAsserted || m_faultLatched;
}

void Motor::onFaultInterrupt()
{
  m_faultLatched = true;
}

void Motor::clearFault()
{
  m_faultLatched = false;
}

int32_t Motor::readCurrentMilliamps()
{
  // Before the DMA scan starts, every slot reads zero — indistinguishable from
  // a genuine 0 mA. Report "no measurement" instead of a plausible lie.
  if (!adcDmaIsRunning())
  {
    return kCurrentUnavailable;
  }

  // Snapshot the volatile slot into a local variable before doing arithmetic.
  // g_adcBuf[] is written by the ADC1 DMA circular scan; see AdcDma.hpp.
  const uint16_t raw = g_adcBuf[m_ipropSlot];

  // V_IPROPI (V) = (raw / 4095) × VDDA
  // I_OUT    (A) = V_IPROPI × m_ipropGain / m_rIpropi
  const float vIpropi = (static_cast<float>(raw) / kAdcMaxF) * kVdda;
  const float iOut    = vIpropi * m_ipropGain / m_rIpropi;

  return static_cast<int32_t>(iOut * 1000.0f); // convert A → mA
}

/* Private Helpers -----------------------------------------------------------*/

uint32_t Motor::dutyToCompare(float duty) const
{
  duty = std::max(0.0f, std::min(1.0f, duty));

  // Full scale is ARR + 1 so that duty == 1.0f produces a continuously high
  // output rather than ARR/(ARR+1).
  const float full = static_cast<float>(m_timerPeriod) + 1.0f;

  return static_cast<uint32_t>(duty * full);
}

/* EOF -----------------------------------------------------------------------*/
