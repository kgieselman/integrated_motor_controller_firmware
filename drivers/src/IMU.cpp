/*******************************************************************************
 * @file IMU.cpp
 * @brief LSM6DSV SPI driver implementation.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#include "IMU.hpp"

#include <cmath>
#include <cstring>

// LSM6DSV register map (main bank unless noted)
namespace Reg
{
  static constexpr uint8_t kFuncCfgAccess  = 0x01U;  // bit 7: embedded func bank
  static constexpr uint8_t kFifoCtrl4      = 0x0AU;  // bits [2:0]: FIFO mode
  static constexpr uint8_t kWhoAmI         = 0x0FU;
  static constexpr uint8_t kCtrl1XL        = 0x10U;  // Accel ODR / FS
  static constexpr uint8_t kCtrl2G         = 0x11U;  // Gyro ODR / FS
  static constexpr uint8_t kCtrl3C         = 0x12U;  // Interface config / soft reset
  static constexpr uint8_t kFifoStatus1    = 0x1BU;  // DIFF_FIFO[7:0]
  static constexpr uint8_t kStatusReg      = 0x1EU;
  static constexpr uint8_t kOutXLG         = 0x22U;  // First gyro output byte (X low)
  static constexpr uint8_t kFifoDataOutTag = 0x78U;  // FIFO entry: 1 tag + 6 data bytes
  // Embedded function bank (accessible when kFuncCfgAccess bit 7 = 1):
  static constexpr uint8_t kEmbFuncEnA     = 0x04U;  // bit 1: sflp_game_en
  static constexpr uint8_t kEmbFuncFifoEnA = 0x44U;  // bits 1,4,5: SFLP FIFO batching
  static constexpr uint8_t kSflpOdr        = 0x5EU;  // bits [5:3]: sflp_game_odr
}

static constexpr uint8_t kFifoTagGameRV = 0x13U;  // SFLP game rotation vector tag

static constexpr uint8_t  kReadFlag   = 0x80U;
static constexpr uint32_t kSpiTimeout = 10U;   // ms

static constexpr float kG = 9.80665f;

IMU::IMU(SPI_HandleTypeDef* hspi,
         GPIO_TypeDef*      csPort,
         uint16_t           csPin)
  : m_hspi(hspi)
  , m_csPort(csPort)
  , m_csPin(csPin)
  , m_accelScale(0.0f)
  , m_gyroScale(0.0f)
{}

bool IMU::init()
{
  if (whoAmI() != kWhoAmIValue)
  {
    return false;
  }

  // Soft reset — SW_RESET bit auto-clears; wait 2 ms for completion.
  if (!writeReg(Reg::kCtrl3C, 0x01U))
  {
    return false;
  }
  HAL_Delay(2U);

  // Enable block data update (BDU) and address auto-increment (IF_INC).
  if (!writeReg(Reg::kCtrl3C, 0x44U))
  {
    return false;
  }

  // Gyroscope: ±2000 °/s, 960 Hz ODR (ODR_G=1001, FS_G=11).
  if (!writeReg(Reg::kCtrl2G, 0x9CU))
  {
    return false;
  }
  m_gyroScale = 2000.0f / 32768.0f;  // LSB → °/s

  // Accelerometer: ±8 g, 960 Hz ODR (ODR_XL=1001, FS_XL=11).
  if (!writeReg(Reg::kCtrl1XL, 0x9CU))
  {
    return false;
  }
  m_accelScale = (8.0f * kG) / 32768.0f;  // LSB → m/s²

  return true;
}

bool IMU::read(AccelGyro& out)
{
  uint8_t buf[12] = {};

  // Burst-read 12 bytes: OUTX_L_G (0x22) through OUTZ_H_A (0x2D).
  // Gyro occupies bytes 0–5, accel occupies bytes 6–11.
  if (!readBurst(Reg::kOutXLG, buf, 12U))
  {
    return false;
  }

  // Data is little-endian: low byte at lower address.
  auto toInt16 = [](uint8_t lo, uint8_t hi) -> int16_t
  {
    return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8U) | lo);
  };

  out.gyroX  = static_cast<float>(toInt16(buf[0],  buf[1]))  * m_gyroScale;
  out.gyroY  = static_cast<float>(toInt16(buf[2],  buf[3]))  * m_gyroScale;
  out.gyroZ  = static_cast<float>(toInt16(buf[4],  buf[5]))  * m_gyroScale;
  out.accelX = static_cast<float>(toInt16(buf[6],  buf[7]))  * m_accelScale;
  out.accelY = static_cast<float>(toInt16(buf[8],  buf[9]))  * m_accelScale;
  out.accelZ = static_cast<float>(toInt16(buf[10], buf[11])) * m_accelScale;

  return true;
}

uint8_t IMU::whoAmI()
{
  uint8_t val = 0xFFU;
  readReg(Reg::kWhoAmI, val);
  return val;
}

bool IMU::dataReady()
{
  uint8_t status = 0U;
  if (!readReg(Reg::kStatusReg, status))
  {
    return false;
  }
  return (status & 0x03U) == 0x03U;  // XLDA=bit0, GDA=bit1
}

bool IMU::enableFusion()
{
  // Enter embedded function register bank.
  if (!writeReg(Reg::kFuncCfgAccess, 0x80U))
  {
    return false;
  }
  HAL_Delay(1U);

  // Enable SFLP game rotation vector (EMB_FUNC_EN_A bit 1).
  if (!writeReg(Reg::kEmbFuncEnA, 0x02U))
  {
    return false;
  }

  // SFLP ODR = 120 Hz (sflp_game_odr = 0x3, placed at bits [5:3]).
  if (!writeReg(Reg::kSflpOdr, 0x18U))
  {
    return false;
  }

  // Enable FIFO batching: game RV (bit 1), gravity (bit 4), gyro bias (bit 5).
  if (!writeReg(Reg::kEmbFuncFifoEnA, 0x32U))
  {
    return false;
  }

  // Return to main register bank.
  if (!writeReg(Reg::kFuncCfgAccess, 0x00U))
  {
    return false;
  }

  // Set FIFO to continuous mode (fifo_mode = 0x6).
  if (!writeReg(Reg::kFifoCtrl4, 0x06U))
  {
    return false;
  }

  return true;
}

// Convert IEEE 754 half-precision (FP16) to float32.
static float fp16ToFloat(uint16_t h)
{
  const uint32_t sign = static_cast<uint32_t>(h & 0x8000U);
  const uint32_t exp  = static_cast<uint32_t>((h >> 10U) & 0x1FU);
  const uint32_t mant = static_cast<uint32_t>(h & 0x03FFU);

  if (exp == 0U)
  {
    // Zero or subnormal: value = ±2^(−14) × (mant / 1024)
    const float val = static_cast<float>(mant) * 5.9604644775390625e-8f;
    return sign ? -val : val;
  }
  if (exp == 0x1FU)
  {
    // Inf or NaN — return 0 so the caller always gets a usable float.
    return 0.0f;
  }
  // Normal: rebias exponent from 15 (FP16) to 127 (FP32).
  const uint32_t bits = (sign << 16U) | ((exp + 112U) << 23U) | (mant << 13U);
  float result;
  memcpy(&result, &bits, sizeof(result));
  return result;
}

bool IMU::readFusion(Quaternion& out)
{
  uint8_t level = 0U;
  if (!readReg(Reg::kFifoStatus1, level) || level == 0U)
  {
    return false;
  }

  // Drain all pending entries, keeping the last game rotation vector found.
  bool     found = false;
  uint8_t  entry[7] = {};
  const uint8_t limit = (level < 64U) ? level : 64U;

  for (uint8_t i = 0U; i < limit; i++)
  {
    if (!readBurst(Reg::kFifoDataOutTag, entry, 7U))
    {
      break;
    }

    const uint8_t tag = (entry[0] >> 3U) & 0x1FU;
    if (tag == kFifoTagGameRV)
    {
      auto toU16 = [](uint8_t lo, uint8_t hi) -> uint16_t
      {
        return static_cast<uint16_t>(lo) | (static_cast<uint16_t>(hi) << 8U);
      };

      out.x = fp16ToFloat(toU16(entry[1], entry[2]));
      out.y = fp16ToFloat(toU16(entry[3], entry[4]));
      out.z = fp16ToFloat(toU16(entry[5], entry[6]));

      const float n2 = out.x * out.x + out.y * out.y + out.z * out.z;
      out.w = (n2 <= 1.0f) ? sqrtf(1.0f - n2) : 0.0f;

      found = true;
    }
  }

  return found;
}

/* Private Helpers -----------------------------------------------------------*/

void IMU::csAssert()
{
  HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_RESET);
}

void IMU::csDeassert()
{
  HAL_GPIO_WritePin(m_csPort, m_csPin, GPIO_PIN_SET);
}

bool IMU::writeReg(uint8_t reg, uint8_t value)
{
  uint8_t tx[2] = { static_cast<uint8_t>(reg & 0x7FU), value };
  csAssert();
  HAL_StatusTypeDef status = HAL_SPI_Transmit(m_hspi, tx, 2U, kSpiTimeout);
  csDeassert();
  return (status == HAL_OK);
}

bool IMU::readReg(uint8_t reg, uint8_t& out)
{
  uint8_t tx[2] = { static_cast<uint8_t>(reg | kReadFlag), 0x00U };
  uint8_t rx[2] = {};
  csAssert();
  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(m_hspi, tx, rx, 2U, kSpiTimeout);
  csDeassert();
  out = rx[1];
  return (status == HAL_OK);
}

bool IMU::readBurst(uint8_t reg, uint8_t* buf, uint16_t len)
{
  // Single TransmitReceive keeps CS low for the whole transaction.
  // TX[0] is the address byte; TX[1..len] are dummy bytes.
  // RX[0] is the byte received during the address phase (discarded); real
  // data occupies RX[1..len].
  // Sized for the largest burst this driver issues (12 data bytes = 13 total).
  uint8_t tx[13] = { static_cast<uint8_t>(reg | kReadFlag) };
  uint8_t rx[13] = {};

  csAssert();
  HAL_StatusTypeDef s = HAL_SPI_TransmitReceive(m_hspi, tx, rx, len + 1U, kSpiTimeout);
  csDeassert();

  if (s == HAL_OK)
  {
    for (uint16_t i = 0U; i < len; i++)
    {
      buf[i] = rx[i + 1U];
    }
  }
  return (s == HAL_OK);
}

/* EOF -----------------------------------------------------------------------*/
