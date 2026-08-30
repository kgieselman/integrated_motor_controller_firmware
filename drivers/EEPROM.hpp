/*******************************************************************************
 * @file EEPROM.hpp
 * @brief M24C64 I²C EEPROM driver (64 Kbit / 8 KB, 32-byte page).
 *
 * Communicates over I²C1 in blocking (polling) mode. The M24C64 address
 * pins A0/A1/A2 are set via solder jumpers on the board; default all-bridged
 * (GND) gives device address 0x50.
 *
 * Pin assignments (from integrated_motor_controller.ioc):
 *  - SCL : PB6 (I2C1 SCL)
 *  - SDA : PB7 (I2C1 SDA)
 *  - ~WC : PA9 (GPIO out — drive LOW to enable writes, HIGH to write-protect)
 *
 * Write operations require the caller to assert write-enable via
 * setWriteProtect(false) before writing and re-enable afterwards.
 *
 * @note The M24C64 has a 5 ms worst-case write cycle time. After any write
 *       operation, the device NAKs I²C transactions until the internal write
 *       cycle completes. poll() handles this with ACK polling.
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
 * @brief M24C64 8 KB I²C EEPROM driver.
 */
class EEPROM
{
public:
  static constexpr uint16_t kSizeBytes    = 8192U; ///< Total capacity (bytes)
  static constexpr uint8_t  kPageBytes    = 32U;   ///< Page size for page-write mode
  static constexpr uint32_t kWriteCycleMs = 6U;    ///< Max internal write cycle (ms)

  /**
   * @brief Construct an EEPROM driver.
   *
   * @param hi2c        Pointer to the HAL I²C handle (I2C2).
   * @param devAddress  7-bit I²C device address (default 0x50 for A2=A1=A0=GND).
   * @param wpPort      GPIO port for the ~WC write-protect pin.
   * @param wpPin       GPIO pin mask for the ~WC pin.
   */
  EEPROM(I2C_HandleTypeDef* hi2c,
         uint8_t            devAddress,
         GPIO_TypeDef*      wpPort,
         uint16_t           wpPin);

  /**
   * @brief Read one byte from EEPROM.
   *
   * @param address  Memory address to read (0–8191).
   * @param[out] out Byte read from the device.
   * @return true on success, false on I²C error or address out of range.
   */
  bool readByte(uint16_t address, uint8_t& out);

  /**
   * @brief Read multiple bytes from EEPROM (sequential read).
   *
   * @param address  Starting memory address.
   * @param buf      Output buffer.
   * @param len      Number of bytes to read.
   * @return true on success, false on error.
   */
  bool readBytes(uint16_t address, uint8_t* buf, uint16_t len);

  /**
   * @brief Write one byte to EEPROM.
   *
   * Automatically asserts and de-asserts write-protect. Blocks until the
   * internal write cycle completes (ACK polling, up to kWriteCycleMs).
   *
   * @param address Memory address to write (0–8191).
   * @param value   Byte to write.
   * @return true on success.
   */
  bool writeByte(uint16_t address, uint8_t value);

  /**
   * @brief Write up to one page (32 bytes) to EEPROM.
   *
   * The write must not cross a 32-byte page boundary. Split larger writes
   * into multiple page-write calls or use writeBytes() which handles this.
   *
   * @param address Starting address (must be page-aligned for best performance).
   * @param buf     Data to write.
   * @param len     Number of bytes (≤ kPageBytes).
   * @return true on success.
   */
  bool writePage(uint16_t address, const uint8_t* buf, uint8_t len);

  /**
   * @brief Write an arbitrary number of bytes, splitting across page boundaries.
   *
   * Handles page-boundary splitting automatically. Slower than manual page
   * management due to one HAL call per page, but safe for any address/length.
   *
   * @param address Starting memory address.
   * @param buf     Data to write.
   * @param len     Number of bytes to write.
   * @return true if all pages written successfully.
   */
  bool writeBytes(uint16_t address, const uint8_t* buf, uint16_t len);

  /**
   * @brief Control the hardware write-protect pin (~WC).
   *
   * @param protect true  → drive ~WC high (write-protected, default safe state).
   *                false → drive ~WC low  (writes enabled).
   */
  void setWriteProtect(bool protect);

private:
  I2C_HandleTypeDef* m_hi2c;       ///< HAL I²C handle.
  uint8_t            m_devAddress; ///< 8-bit address (7-bit << 1), as expected by HAL.
  GPIO_TypeDef*      m_wpPort;     ///< GPIO port for the ~WC write-protect pin.
  uint16_t           m_wpPin;      ///< GPIO pin mask for the ~WC pin.

  /**
   * @brief Poll until the device ACKs (write cycle complete) or timeout.
   *
   * @param timeoutMs Maximum wait in milliseconds.
   * @return true if ACK received within timeout.
   */
  bool pollAck(uint32_t timeoutMs = kWriteCycleMs + 2U);
};

/* EOF -----------------------------------------------------------------------*/
