/*******************************************************************************
 * @file CrsfTest.cpp
 * @brief Bringup test for the CRSF / ELRS receiver (UART4).
 *
 * Subcommands:
 *   crsf stats  — Continuously print all 16 RC channel values and link age.
 *                 Press any key to stop.
 ******************************************************************************/

#include "tests/CrsfTest.hpp"
#include "CRSFReceiver.hpp"

extern "C"
{
#include "usart.h"
}

#include <cstring>

/* File-local receiver instance ----------------------------------------------*/

static CRSFReceiver s_crsf(&huart4);
static bool         s_initialized = false;

/* Public callback router (called from main_bringup.cpp) ---------------------*/

void crsfOnDmaRxEvent(uint16_t size)
{
  s_crsf.onDmaRxEvent(size);
}

/* Command handler -----------------------------------------------------------*/

static void handleCrsf(Console& c, int argc, char* argv[])
{
  if (argc < 2 || strcmp(argv[1], "help") == 0)
  {
    c.println("Usage: crsf <subcommand>");
    c.println("  stats   Print all 16 RC channel values and link age  (any key to stop)");
    return;
  }

  if (strcmp(argv[1], "stats") == 0)
  {
    if (!s_initialized)
    {
      if (!s_crsf.init())
      {
        c.println("crsf: ERROR — DMA init failed (check UART4 / GPDMA config)");
        return;
      }
      s_initialized = true;
    }

    c.println("CRSF stats — press any key to stop");
    c.println("Ch:  1    2    3    4    5    6    7    8    9   10   11   12   13   14   15   16   Age(ms)");
    c.println("---------------------------------------------------------------------------------------------");

    while (!c.interrupted())
    {
      s_crsf.update();

      c.printf("    %4u %4u %4u %4u %4u %4u %4u %4u %4u %4u %4u %4u %4u %4u %4u %4u   %lu\r\n",
               s_crsf.channelUs(0U),  s_crsf.channelUs(1U),
               s_crsf.channelUs(2U),  s_crsf.channelUs(3U),
               s_crsf.channelUs(4U),  s_crsf.channelUs(5U),
               s_crsf.channelUs(6U),  s_crsf.channelUs(7U),
               s_crsf.channelUs(8U),  s_crsf.channelUs(9U),
               s_crsf.channelUs(10U), s_crsf.channelUs(11U),
               s_crsf.channelUs(12U), s_crsf.channelUs(13U),
               s_crsf.channelUs(14U), s_crsf.channelUs(15U),
               s_crsf.lastFrameAgeMs());

      HAL_Delay(50U); // ~20 Hz refresh
    }
    return;
  }

  c.printf("crsf: unknown subcommand '%s'  (try 'crsf help')\r\n", argv[1]);
}

void registerCrsfTests(Console& c)
{
  c.registerCommand("crsf", "CRSF/ELRS receiver test: crsf <stats>", handleCrsf);
}
