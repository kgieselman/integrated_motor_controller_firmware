/*******************************************************************************
 * @file IMU.hpp
 * @brief LSM6DSV 6-axis IMU driver over SPI.
 *
 * The LSM6DSV communicates over SPI in mode 3 (CPOL=1, CPHA=1) at a maximum
 * of 10 MHz. The CubeMX SPI1 peripheral must be configured with
 * BaudRatePrescaler ≥ 16 when sourced from PLL1Q (125 MHz) to stay at or
 * below 10 MHz.
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
 *   IMU imu(&hspi1, IMU_SPI_CS_GPIO_Port, IMU_SPI_CS_Pin);
 *   imu.init();
 *   AccelGyro sample{};
 *   imu.read(sample);
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
 * @brief Game rotation vector output from the SFLP sensor fusion block.
 *
 * The quaternion is a unit quaternion (x,y,z,w) representing orientation
 * relative to an arbitrary initial heading. w is computed from x,y,z as
 * √(1 − x²−y²−z²).
 */
struct Quaternion
{
  float x;
  float y;
  float z;
  float w;
};

/**
 * @brief LSM6DSV driver (SPI, polling mode).
 *
 * All register transactions are blocking (HAL_SPI_TransmitReceive with a
 * short timeout). For the tactical firmware, consider DMA-driven burst reads
 * triggered by the INT1 data-ready interrupt.
 */
class IMU
{
public:
  /// Expected WHO_AM_I register value for the LSM6DSV.
  static constexpr uint8_t kWhoAmIValue = 0x70U;

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
   * @brief Initialise the LSM6DSV.
   *
   * Verifies WHO_AM_I, soft-resets the device, and configures:
   *  - Accelerometer: ±8 g range, 960 Hz ODR, high-performance mode
   *  - Gyroscope:     ±2000 °/s range, 960 Hz ODR, high-performance mode
   *  - Block data update enabled for coherent register reads
   *
   * @return true on success, false if WHO_AM_I does not match or SPI fails.
   */
  bool init();

  /**
   * @brief Read one accel + gyro sample via a burst SPI transaction.
   *
   * Reads 12 bytes starting at OUTX_L_G and converts to physical units.
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
   * @return WHO_AM_I byte (0x70 expected), or 0xFF on SPI error.
   */
  uint8_t whoAmI();

  /**
   * @brief Check whether both accel and gyro data-ready flags are set.
   *
   * Reads STATUS_REG and checks the XLDA and GDA bits.
   *
   * @return true if new data is available for both sensors.
   */
  bool dataReady();

  /**
   * @brief Enable the SFLP sensor fusion block and route its output to the FIFO.
   *
   * Must be called after init(). Configures:
   *  - SFLP game rotation vector at 120 Hz
   *  - FIFO in continuous mode, batching game RV, gravity, and gyro bias entries
   *
   * @return true on success, false on SPI error.
   */
  bool enableFusion();

  /**
   * @brief Read the latest game rotation vector from the FIFO.
   *
   * Drains all pending FIFO entries and returns the most recent quaternion.
   *
   * @param[out] out  Quaternion to populate.
   * @return true if at least one game rotation vector entry was found.
   */
  bool readFusion(Quaternion& out);

private:
  SPI_HandleTypeDef* m_hspi;
  GPIO_TypeDef*      m_csPort;
  uint16_t           m_csPin;

  float m_accelScale; ///< LSB → m/s² scale factor (set during init).
  float m_gyroScale;  ///< LSB → °/s  scale factor (set during init).

  void csAssert();
  void csDeassert();

  bool writeReg(uint8_t reg, uint8_t value);
  bool readReg(uint8_t reg, uint8_t& out);
  bool readBurst(uint8_t reg, uint8_t* buf, uint16_t len);
};

/* EOF -----------------------------------------------------------------------*/
