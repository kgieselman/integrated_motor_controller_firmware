/*******************************************************************************
 * @file MotorTest.cpp
 * @brief Bringup test stubs for the DRV8874 motor channels (TIM3).
 *
 * Subcommands:
 *   motor left  <pct>  — Drive left channel at <pct> percent  (-100 to 100).
 *   motor right <pct>  — Drive right channel at <pct> percent (-100 to 100).
 *   motor stop         — Coast both channels (duty = 0).
 *   motor brake        — Active brake both channels.
 ******************************************************************************/

#include "tests/MotorTest.hpp"

#include <cstring>

static void handleMotor(Console& c, int argc, char* argv[])
{
  if (argc < 2 || strcmp(argv[1], "help") == 0)
  {
    c.println("Usage: motor <subcommand>");
    c.println("  left  <pct>   Drive left channel  -100..100 %");
    c.println("  right <pct>   Drive right channel -100..100 %");
    c.println("  stop          Coast both channels");
    c.println("  brake         Active brake both channels");
    return;
  }

  if (strcmp(argv[1], "left") == 0)
  {
    // TODO: parse argv[2] as float, construct Motor, call setDuty()
    c.println("motor left — not yet implemented");
    return;
  }

  if (strcmp(argv[1], "right") == 0)
  {
    // TODO: parse argv[2] as float, construct Motor, call setDuty()
    c.println("motor right — not yet implemented");
    return;
  }

  if (strcmp(argv[1], "stop") == 0)
  {
    // TODO: call coast() on both motors
    c.println("motor stop — not yet implemented");
    return;
  }

  if (strcmp(argv[1], "brake") == 0)
  {
    // TODO: call brake() on both motors
    c.println("motor brake — not yet implemented");
    return;
  }

  c.printf("motor: unknown subcommand '%s'  (try 'motor help')\r\n", argv[1]);
}

void registerMotorTests(Console& c)
{
  c.registerCommand("motor", "DRV8874 motor tests: motor <left|right|stop|brake>", handleMotor);
}
