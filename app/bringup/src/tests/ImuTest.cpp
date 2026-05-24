/*******************************************************************************
 * @file ImuTest.cpp
 * @brief Bringup tests for the ICM-42688-P IMU (SPI1).
 *
 * Subcommands:
 *   imu who     — Read WHO_AM_I register and compare against expected value.
 *   imu read    — Single accel/gyro sample.
 *   imu fusion  — Single SFLP game rotation vector (quaternion).
 *   imu stream  — Continuous accel/gyro output; any keypress stops the loop.
 ******************************************************************************/

extern "C"
{
#include "spi.h"
#include "main.h"
}

#include "tests/ImuTest.hpp"
#include "IMU.hpp"

#include <cstring>

static void handleImu(Console& c, int argc, char* argv[])
{
  if (argc < 2 || strcmp(argv[1], "help") == 0)
  {
    c.println("Usage: imu <subcommand>");
    c.println("  who     Read WHO_AM_I register");
    c.println("  read    Single accel/gyro sample");
    c.println("  fusion  Single SFLP game rotation vector (quaternion)");
    c.println("  stream  Continuous accel/gyro output  (any key to stop)");
    return;
  }

  if (strcmp(argv[1], "who") == 0)
  {
    IMU imu(&hspi1, IMU_SPI_CS_GPIO_Port, IMU_SPI_CS_Pin);
    uint8_t id = imu.whoAmI();
    c.printf("WHO_AM_I = 0x%02X  (expected 0x%02X)  %s\r\n",
             id, IMU::kWhoAmIValue,
             id == IMU::kWhoAmIValue ? "OK" : "MISMATCH");
    return;
  }

  if (strcmp(argv[1], "read") == 0)
  {
    IMU imu(&hspi1, IMU_SPI_CS_GPIO_Port, IMU_SPI_CS_Pin);
    if (!imu.init())
    {
      c.println("imu init FAILED");
      return;
    }
    // Wait for first sample — poll XLDA+GDA flags with a 100 ms timeout.
    const uint32_t deadline = HAL_GetTick() + 100U;
    while (!imu.dataReady() && HAL_GetTick() < deadline)
    {
    }
    if (!imu.dataReady())
    {
      c.println("imu data-ready timeout");
      return;
    }
    AccelGyro sample{};
    if (!imu.read(sample))
    {
      c.println("imu read FAILED");
      return;
    }
    c.printf("accelX: %+.4f m/s2\r\n", static_cast<double>(sample.accelX));
    c.printf("accelY: %+.4f m/s2\r\n", static_cast<double>(sample.accelY));
    c.printf("accelZ: %+.4f m/s2\r\n", static_cast<double>(sample.accelZ));
    c.printf("gyroX:  %+.3f d/s\r\n",  static_cast<double>(sample.gyroX));
    c.printf("gyroY:  %+.3f d/s\r\n",  static_cast<double>(sample.gyroY));
    c.printf("gyroZ:  %+.3f d/s\r\n",  static_cast<double>(sample.gyroZ));
    return;
  }

  if (strcmp(argv[1], "fusion") == 0)
  {
    IMU imu(&hspi1, IMU_SPI_CS_GPIO_Port, IMU_SPI_CS_Pin);
    if (!imu.init())
    {
      c.println("imu init FAILED");
      return;
    }
    if (!imu.enableFusion())
    {
      c.println("imu enableFusion FAILED");
      return;
    }
    // SFLP at 120 Hz — wait up to 500 ms for the first FIFO entry.
    Quaternion q{};
    const uint32_t deadline = HAL_GetTick() + 500U;
    bool ok = false;
    while (HAL_GetTick() < deadline)
    {
      if (imu.readFusion(q)) { ok = true; break; }
    }
    if (!ok)
    {
      c.println("imu fusion FIFO timeout");
      return;
    }
    c.printf("qx: %+.4f\r\n", static_cast<double>(q.x));
    c.printf("qy: %+.4f\r\n", static_cast<double>(q.y));
    c.printf("qz: %+.4f\r\n", static_cast<double>(q.z));
    c.printf("qw: %+.4f\r\n", static_cast<double>(q.w));
    return;
  }

  if (strcmp(argv[1], "stream") == 0)
  {
    IMU imu(&hspi1, IMU_SPI_CS_GPIO_Port, IMU_SPI_CS_Pin);
    if (!imu.init())
    {
      c.println("imu init FAILED");
      return;
    }
    c.println("imu stream  (any key to stop)");
    c.println("  accelX(m/s2)  accelY(m/s2)  accelZ(m/s2)   gyroX(d/s)   gyroY(d/s)   gyroZ(d/s)");
    while (!c.interrupted())
    {
      AccelGyro sample{};
      if (imu.read(sample))
      {
        c.printf("  %+10.4f    %+10.4f    %+10.4f    %+9.3f   %+9.3f   %+9.3f\r\n",
                 static_cast<double>(sample.accelX), static_cast<double>(sample.accelY),
                 static_cast<double>(sample.accelZ), static_cast<double>(sample.gyroX),
                 static_cast<double>(sample.gyroY),  static_cast<double>(sample.gyroZ));
      }
      else
      {
        c.println("  read FAILED");
      }
      HAL_Delay(100);
    }
    return;
  }

  c.printf("imu: unknown subcommand '%s'  (try 'imu help')\r\n", argv[1]);
}

void registerImuTests(Console& c)
{
  c.registerCommand("imu", "LSM6DSV tests: imu <who|read|fusion|stream>", handleImu);
}
