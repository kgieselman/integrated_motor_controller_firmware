/*******************************************************************************
 * @file CrsfTest.cpp
 * @brief Bringup test stub for the CRSF / ELRS receiver (UART4).
 *
 * Subcommands:
 *   crsf stats  — Print CRSF link quality and raw RC channel values.
 ******************************************************************************/

#include "tests/CrsfTest.hpp"

#include <cstring>

static void handleCrsf(Console& c, int argc, char* argv[])
{
  if (argc < 2 || strcmp(argv[1], "help") == 0)
  {
    c.println("Usage: crsf <subcommand>");
    c.println("  stats   Print link quality and RC channel values  (any key to stop)");
    return;
  }

  if (strcmp(argv[1], "stats") == 0)
  {
    // TODO: construct CRSFReceiver(&huart4), arm DMA RX, call update() + getLinkStats()
    c.println("crsf stats — not yet implemented  (any key to stop)");
    while (!c.interrupted())
    {
      // TODO: call update(), print channel values and RSSI
    }
    return;
  }

  c.printf("crsf: unknown subcommand '%s'  (try 'crsf help')\r\n", argv[1]);
}

void registerCrsfTests(Console& c)
{
  c.registerCommand("crsf", "CRSF/ELRS receiver test: crsf <stats>", handleCrsf);
}
