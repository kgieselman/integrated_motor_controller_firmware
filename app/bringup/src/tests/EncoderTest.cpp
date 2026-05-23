/*******************************************************************************
 * @file EncoderTest.cpp
 * @brief Bringup test stubs for quadrature encoders (TIM2 left, TIM8 right).
 *
 * Subcommands:
 *   enc read    — Single snapshot of both encoder counts and velocities.
 *   enc stream  — Continuous output; any keypress stops the loop.
 ******************************************************************************/

#include "tests/EncoderTest.hpp"

#include <cstring>

static void handleEncoder(Console& c, int argc, char* argv[])
{
  if (argc < 2 || strcmp(argv[1], "help") == 0)
  {
    c.println("Usage: enc <subcommand>");
    c.println("  read    Snapshot of both encoder counts");
    c.println("  stream  Continuous encoder output  (any key to stop)");
    return;
  }

  if (strcmp(argv[1], "read") == 0)
  {
    // TODO: construct Encoder(&htim2, ...) and Encoder(&htim8, ...), call getCount()
    c.println("enc read — not yet implemented");
    return;
  }

  if (strcmp(argv[1], "stream") == 0)
  {
    // TODO: replace stub body with real encoder reads + HAL_Delay cadence.
    c.println("enc stream — not yet implemented  (any key to stop)");
    while (!c.interrupted())
    {
      // TODO: update encoders and c.printf(...)
    }
    return;
  }

  c.printf("enc: unknown subcommand '%s'  (try 'enc help')\r\n", argv[1]);
}

void registerEncoderTests(Console& c)
{
  c.registerCommand("enc", "Encoder tests: enc <read|stream>", handleEncoder);
}
