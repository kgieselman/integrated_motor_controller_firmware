/*******************************************************************************
 * @file ImuTest.cpp
 * @brief Bringup test stubs for the ICM-42688-P IMU (SPI1).
 *
 * Subcommands:
 *   imu who     — Read WHO_AM_I register and compare against expected value.
 *   imu stream  — Continuous accel/gyro output; any keypress stops the loop.
 ******************************************************************************/

#include "tests/ImuTest.hpp"

#include <cstring>

static void handleImu(Console& c, int argc, char* argv[])
{
  if (argc < 2 || strcmp(argv[1], "help") == 0)
  {
    c.println("Usage: imu <subcommand>");
    c.println("  who     Read WHO_AM_I register");
    c.println("  stream  Continuous accel/gyro output  (any key to stop)");
    return;
  }

  if (strcmp(argv[1], "who") == 0)
  {
    // TODO: construct IMU(&hspi1, CS_GPIO, CS_PIN), call init(), check whoAmI()
    c.println("imu who — not yet implemented");
    return;
  }

  if (strcmp(argv[1], "stream") == 0)
  {
    // Streaming skeleton — demonstrates the interrupted() stop pattern.
    // TODO: construct IMU instance and replace stub body with real reads.
    c.println("imu stream — not yet implemented  (any key to stop)");
    while (!c.interrupted())
    {
      // TODO: read IMU sample and c.printf(...)
    }
    return;
  }

  c.printf("imu: unknown subcommand '%s'  (try 'imu help')\r\n", argv[1]);
}

void registerImuTests(Console& c)
{
  c.registerCommand("imu", "ICM-42688-P tests: imu <who|stream>", handleImu);
}
