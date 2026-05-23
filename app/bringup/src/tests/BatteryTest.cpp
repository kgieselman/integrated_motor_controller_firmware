/*******************************************************************************
 * @file BatteryTest.cpp
 * @brief Bringup test stub for the battery voltage monitor (ADC1 / VBAT_SENSE).
 *
 * Subcommands:
 *   battery read — Read and print cell voltage via the ADC1 DMA slot.
 ******************************************************************************/

#include "tests/BatteryTest.hpp"

#include <cstring>

static void handleBattery(Console& c, int argc, char* argv[])
{
  if (argc < 2 || strcmp(argv[1], "help") == 0)
  {
    c.println("Usage: battery <subcommand>");
    c.println("  read   Print battery voltage (V)");
    return;
  }

  if (strcmp(argv[1], "read") == 0)
  {
    // TODO: ensure ADC1 DMA scan is running, construct Battery, call voltageV()
    c.println("battery read — not yet implemented");
    return;
  }

  c.printf("battery: unknown subcommand '%s'  (try 'battery help')\r\n", argv[1]);
}

void registerBatteryTests(Console& c)
{
  c.registerCommand("battery", "Battery voltage test: battery <read>", handleBattery);
}
