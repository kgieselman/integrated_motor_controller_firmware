/*******************************************************************************
 * @file EEPROM.cpp
 * @brief M24C64 I²C EEPROM driver implementation.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#include "EEPROM.hpp"

#include <algorithm>

static constexpr uint32_t kI2cTimeout = 20U; // ms per HAL call

EEPROM::EEPROM(I2C_HandleTypeDef* hi2c,
               uint8_t            devAddress,
               GPIO_TypeDef*      wpPort,
               uint16_t           wpPin)
  : m_hi2c(hi2c)
  , m_devAddress(static_cast<uint8_t>(devAddress << 1U)) // HAL uses 8-bit address
  , m_wpPort(wpPort)
  , m_wpPin(wpPin)
{}

bool EEPROM::readByte(uint16_t address, uint8_t& out)
{
  if (address >= kSizeBytes)
  {
    return false;
  }
  return (HAL_I2C_Mem_Read(m_hi2c, m_devAddress, address,
                            I2C_MEMADD_SIZE_16BIT, &out, 1U, kI2cTimeout) == HAL_OK);
}

bool EEPROM::readBytes(uint16_t address, uint8_t* buf, uint16_t len)
{
  if ((address + len) > kSizeBytes || buf == nullptr)
  {
    return false;
  }
  return (HAL_I2C_Mem_Read(m_hi2c, m_devAddress, address,
                            I2C_MEMADD_SIZE_16BIT, buf, len, kI2cTimeout) == HAL_OK);
}

bool EEPROM::writeByte(uint16_t address, uint8_t value)
{
  if (address >= kSizeBytes)
  {
    return false;
  }

  setWriteProtect(false);

  bool ok = (HAL_I2C_Mem_Write(m_hi2c, m_devAddress, address,
                                I2C_MEMADD_SIZE_16BIT, &value, 1U, kI2cTimeout) == HAL_OK);
  if (ok)
  {
    ok = pollAck();
  }

  setWriteProtect(true);
  return ok;
}

bool EEPROM::writePage(uint16_t address, const uint8_t* buf, uint8_t len)
{
  if (buf == nullptr || len == 0U || len > kPageBytes)
  {
    return false;
  }
  if ((address + len) > kSizeBytes)
  {
    return false;
  }

  setWriteProtect(false);

  // Cast away const for HAL (HAL_I2C_Mem_Write takes uint8_t*).
  bool ok = (HAL_I2C_Mem_Write(m_hi2c, m_devAddress, address,
                                I2C_MEMADD_SIZE_16BIT,
                                const_cast<uint8_t*>(buf), len,
                                kI2cTimeout) == HAL_OK);
  if (ok)
  {
    ok = pollAck();
  }

  setWriteProtect(true);
  return ok;
}

bool EEPROM::writeBytes(uint16_t address, const uint8_t* buf, uint16_t len)
{
  if (buf == nullptr || len == 0U)
  {
    return false;
  }
  if ((address + len) > kSizeBytes)
  {
    return false;
  }

  uint16_t       remaining = len;
  uint16_t       addr      = address;
  const uint8_t* ptr       = buf;

  while (remaining > 0U)
  {
    // How many bytes until the next page boundary?
    uint8_t pageOffset  = static_cast<uint8_t>(addr % kPageBytes);
    uint8_t spaceInPage = static_cast<uint8_t>(kPageBytes - pageOffset);
    uint8_t chunk       = static_cast<uint8_t>(
        std::min(static_cast<uint16_t>(spaceInPage), remaining));

    if (!writePage(addr, ptr, chunk))
    {
      return false;
    }

    addr      += chunk;
    ptr       += chunk;
    remaining -= chunk;
  }

  return true;
}

void EEPROM::setWriteProtect(bool protect)
{
  // ~WC: high = protected, low = write-enabled.
  HAL_GPIO_WritePin(m_wpPort, m_wpPin, protect ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool EEPROM::pollAck(uint32_t timeoutMs)
{
  uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < timeoutMs)
  {
    if (HAL_I2C_IsDeviceReady(m_hi2c, m_devAddress, 1U, 1U) == HAL_OK)
    {
      return true;
    }
  }
  return false;
}

/* EOF -----------------------------------------------------------------------*/
