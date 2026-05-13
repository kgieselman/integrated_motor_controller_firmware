/*******************************************************************************
 * @file IMU.cpp
 * @brief ICM-42688-P SPI driver implementation.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#include "IMU.hpp"

/* Private Helpers -----------------------------------------------------------*/

// ICM-42688-P register map (Bank 0)
namespace Reg
{
  static constexpr uint8_t kDeviceConfig = 0x11U;
  static constexpr uint8_t kIntConfig    = 0x14U;
  static constexpr uint8_t kFifoConfig   = 0x16U;
  static constexpr uint8_t kAccelDataX1  = 0x1FU;
  static constexpr uint8_t kIntStatus    = 0x2DU;
  static constexpr uint8_t kPwrMgmt0    = 0x4EU;
  static constexpr uint8_t kGyroConfig0  = 0x4FU;
  static constexpr uint8_t kAccelConfig0 = 0x50U;
  static constexpr uint8_t kIntConfig0   = 0x63U;
  static constexpr uint8_t kIntSource0   = 0x65U;
  static constexpr uint8_t kWhoAmI       = 0x75U;
}

static constexpr uint8_t  kReadFlag   = 0x80U;
static constexpr uint32_t kSpiTimeout = 10U;   // ms

// Physical constants
static constexpr float kG = 9.80665f; // m/s² per g

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
  // Verify device identity.
  if (whoAmI() != kWhoAmIValue)
  {
    return false;
  }

  // Soft reset — wait ≥1 ms for reset to complete.
  if (!writeReg(Reg::kDeviceConfig, 0x01U))
  {
    return false;
  }
  HAL_Delay(2U);

  // Enable accel + gyro in low-noise mode (PWR_MGMT0: GYRO_MODE=3, ACCEL_MODE=3).
  if (!writeReg(Reg::kPwrMgmt0, 0x0FU))
  {
    return false;
  }
  HAL_Delay(1U);

  // Gyroscope: ±2000 °/s, 1 kHz ODR (GYRO_CONFIG0 = 0x06).
  if (!writeReg(Reg::kGyroConfig0, 0x06U))
  {
    return false;
  }
  m_gyroScale = 2000.0f / 32768.0f; // LSB → °/s

  // Accelerometer: ±16 g, 1 kHz ODR (ACCEL_CONFIG0 = 0x06).
  if (!writeReg(Reg::kAccelConfig0, 0x06U))
  {
    return false;
  }
  m_accelScale = (16.0f * kG) / 32768.0f; // LSB → m/s²

  // INT1: data-ready, active-high, push-pull (INT_CONFIG = 0x00, INT_SOURCE0 bit0 = 1).
  if (!writeReg(Reg::kIntConfig,  0x00U))
  {
    return false;
  }
  if (!writeReg(Reg::kIntSource0, 0x08U)) // UI_DRDY_INT1_EN
  {
    return false;
  }

  return true;
}

bool IMU::read(AccelGyro& out)
{
  uint8_t buf[12] = {};

  // Burst-read: ACCEL_DATA_X1 through GYRO_DATA_Z0 (12 bytes).
  if (!readBurst(Reg::kAccelDataX1, buf, 12U))
  {
    return false;
  }

  // Reconstruct 16-bit signed values (big-endian).
  auto toInt16 = [](uint8_t hi, uint8_t lo) -> int16_t
  {
    return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8U) | lo);
  };

  out.accelX = static_cast<float>(toInt16(buf[0],  buf[1]))  * m_accelScale;
  out.accelY = static_cast<float>(toInt16(buf[2],  buf[3]))  * m_accelScale;
  out.accelZ = static_cast<float>(toInt16(buf[4],  buf[5]))  * m_accelScale;
  out.gyroX  = static_cast<float>(toInt16(buf[6],  buf[7]))  * m_gyroScale;
  out.gyroY  = static_cast<float>(toInt16(buf[8],  buf[9]))  * m_gyroScale;
  out.gyroZ  = static_cast<float>(toInt16(buf[10], buf[11])) * m_gyroScale;

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
  if (!readReg(Reg::kIntStatus, status))
  {
    return false;
  }
  return (status & 0x08U) != 0U; // DATA_RDY_INT bit
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
  // First byte is the address byte; remaining bytes are dummy TX / real RX.
  uint8_t addrByte = static_cast<uint8_t>(reg | kReadFlag);
  csAssert();

  // Send address.
  HAL_StatusTypeDef s = HAL_SPI_Transmit(m_hspi, &addrByte, 1U, kSpiTimeout);
  if (s == HAL_OK)
  {
    // Receive data bytes (TX dummy bytes are irrelevant).
    uint8_t dummy[12] = {};
    s = HAL_SPI_TransmitReceive(m_hspi, dummy, buf, len, kSpiTimeout);
  }

  csDeassert();
  return (s == HAL_OK);
}

/* EOF -----------------------------------------------------------------------*/
