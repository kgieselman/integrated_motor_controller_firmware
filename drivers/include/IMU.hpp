/*******************************************************************************
 * @file IMU.hpp
 * @brief ICM-42688-P 6-axis IMU driver over SPI.
 *
 * The ICM-42688-P communicates over SPI in mode 0 or mode 3 (CPOL=CPHA).
 * CubeMX should configure the SPI peripheral in full-duplex master mode at
 * ≤ 24 MHz (register access) or ≤ 24 MHz (sensor data burst).
 *
 * Pin assignments (from integrated_motor_controller.ioc):
 *  - SCK  : PB3  (SPI1 SCK)
 *  - MISO : PB4  (SPI1 MISO)
 *  - MOSI : PB5  (SPI1 MOSI)
 *  - CS   : PC12 (GPIO out, active-low)
 *  - INT1 : PD2  (EXTI2, data-ready interrupt)
 *  - INT2 : PB8  (EXTI8, optional FIFO watermark)
 *
 * Usage:
 * @code
 *   IMU imu(&hspi1, GPIOA, GPIO_PIN_15);
 *   imu.init();
 *   auto data = imu.read();
 * @endcode
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
 * @brief Struct holding one sample of accelerometer and gyroscope data.
 */
struct AccelGyro
{
  float accelX; ///< X-axis acceleration (m/s²)
  float accelY; ///< Y-axis acceleration (m/s²)
  float accelZ; ///< Z-axis acceleration (m/s²)
  float gyroX;  ///< X-axis angular rate (°/s)
  float gyroY;  ///< Y-axis angular rate (°/s)
  float gyroZ;  ///< Z-axis angular rate (°/s)
};

/**
 * @brief ICM-42688-P driver (SPI, polling mode).
 *
 * All register transactions are blocking (HAL_SPI_TransmitReceive with a
 * short timeout). For the tactical firmware, consider DMA-driven burst reads
 * triggered by the INT1 data-ready interrupt.
 */
class IMU
{
public:
  /// Expected WHO_AM_I register value for ICM-42688-P.
  static constexpr uint8_t kWhoAmIValue = 0x47U;

  /**
   * @brief Construct an IMU driver.
   *
   * @param hspi   Pointer to the HAL SPI handle.
   * @param csPort GPIO port for the chip-select pin.
   * @param csPin  GPIO pin mask for the chip-select pin (active-low).
   */
  IMU(SPI_HandleTypeDef* hspi,
      GPIO_TypeDef*      csPort,
      uint16_t           csPin);

  /**
   * @brief Initialise the ICM-42688-P.
   *
   * Verifies WHO_AM_I, soft-resets the device, and configures:
   *  - Accelerometer: ±16 g range, 1 kHz ODR
   *  - Gyroscope:     ±2000 °/s range, 1 kHz ODR
   *  - INT1 pin as data-ready (active-high, push-pull)
   *
   * @return true on success, false if WHO_AM_I does not match or SPI fails.
   */
  bool init();

  /**
   * @brief Read one accel + gyro sample via a burst SPI transaction.
   *
   * Reads 12 bytes starting at ACCEL_DATA_X1 and converts to physical units.
   *
   * @param[out] out  Struct to populate with calibrated sensor data.
   * @return true on success, false on SPI error.
   */
  bool read(AccelGyro& out);

  /**
   * @brief Read WHO_AM_I register.
   *
   * Useful as a bring-up sanity check without calling init().
   *
   * @return WHO_AM_I byte, or 0xFF on SPI error.
   */
  uint8_t whoAmI();

  /**
   * @brief Check whether a data-ready interrupt is pending.
   *
   * Reads the INT_STATUS register bit. Alternative to using the INT1 EXTI line.
   *
   * @return true if new data is available.
   */
  bool dataReady();

private:
  SPI_HandleTypeDef* m_hspi;       ///< HAL SPI handle.
  GPIO_TypeDef*      m_csPort;     ///< GPIO port for the chip-select pin.
  uint16_t           m_csPin;      ///< GPIO pin mask for the chip-select pin.

  float m_accelScale; ///< LSB → m/s² scale factor (set during init).
  float m_gyroScale;  ///< LSB → °/s  scale factor (set during init).

  /**
   * @brief Assert the CS pin (drive low).
   */
  void csAssert();

  /**
   * @brief Deassert the CS pin (drive high).
   */
  void csDeassert();

  /**
   * @brief Write one register byte.
   *
   * @param reg   7-bit register address.
   * @param value Byte to write.
   * @return true on HAL_OK.
   */
  bool writeReg(uint8_t reg, uint8_t value);

  /**
   * @brief Read one register byte.
   *
   * @param reg       7-bit register address.
   * @param[out] out  Byte read from the device.
   * @return true on HAL_OK.
   */
  bool readReg(uint8_t reg, uint8_t& out);

  /**
   * @brief Burst-read multiple consecutive registers.
   *
   * @param reg    Starting register address.
   * @param buf    Output buffer.
   * @param len    Number of bytes to read.
   * @return true on HAL_OK.
   */
  bool readBurst(uint8_t reg, uint8_t* buf, uint16_t len);
};

/* EOF -----------------------------------------------------------------------*/
