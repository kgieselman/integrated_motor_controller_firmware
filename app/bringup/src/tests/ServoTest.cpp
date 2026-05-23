/*******************************************************************************
 * @file ServoTest.cpp
 * @brief Bringup test stubs for RC servo PWM outputs (TIM4).
 *
 * Subcommands:
 *   servo set   <ch> <us> — Set channel <ch> (0-based) to <us> microseconds.
 *   servo sweep <ch>      — Sweep channel from 1000 to 2000 µs and back.
 ******************************************************************************/

#include "tests/ServoTest.hpp"

#include <cstring>
#include <cstdlib>

static void handleServo(Console& c, int argc, char* argv[])
{
  if (argc < 2 || strcmp(argv[1], "help") == 0)
  {
    c.println("Usage: servo <subcommand>");
    c.println("  set   <ch> <us>   Set channel pulse width in microseconds");
    c.println("  sweep <ch>        Sweep 1000-2000 µs  (any key to stop)");
    return;
  }

  if (strcmp(argv[1], "set") == 0)
  {
    if (argc < 4)
    {
      c.println("Usage: servo set <ch> <us>");
      return;
    }
    // TODO: parse argv[2]/argv[3], construct ServoChannel, call setPulseUs()
    c.printf("servo set ch=%lu us=%lu — not yet implemented\r\n",
             strtoul(argv[2], nullptr, 0),
             strtoul(argv[3], nullptr, 0));
    return;
  }

  if (strcmp(argv[1], "sweep") == 0)
  {
    if (argc < 3)
    {
      c.println("Usage: servo sweep <ch>");
      return;
    }
    // TODO: construct ServoChannel for argv[2], loop setPulseUs() with HAL_Delay
    c.printf("servo sweep ch=%lu — not yet implemented  (any key to stop)\r\n",
             strtoul(argv[2], nullptr, 0));
    while (!c.interrupted())
    {
      // TODO: sweep servo and c.printf(...)
    }
    return;
  }

  c.printf("servo: unknown subcommand '%s'  (try 'servo help')\r\n", argv[1]);
}

void registerServoTests(Console& c)
{
  c.registerCommand("servo", "Servo PWM tests: servo <set|sweep>", handleServo);
}
