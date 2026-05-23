/*******************************************************************************
 * @file LedTest.cpp
 * @brief Bringup test for debug LEDs 1 and 2 (PC0, PC1).
 *
 * LED 0 (PC15) is reserved for the heartbeat blink in main_bringup.cpp.
 *
 * Subcommands:
 *   led on  <1|2>   — Turn the specified LED on.
 *   led off <1|2>   — Turn the specified LED off.
 *   led toggle <1|2>— Toggle the specified LED.
 *   led blink <1|2> — Blink the LED at ~2 Hz until any key is pressed.
 ******************************************************************************/

extern "C"
{
#include "stm32h5xx_hal.h"
#include "main.h"
}

#include "tests/LedTest.hpp"

#include <cstring>
#include <cstdlib>

static void printUsage(Console& c)
{
  c.println("Usage: led <subcommand>");
  c.println("  on     <1|2>   Turn LED on");
  c.println("  off    <1|2>   Turn LED off");
  c.println("  toggle <1|2>   Toggle LED");
  c.println("  blink  <1|2>   Blink at 2 Hz  (any key to stop)");
}

static bool parseLed(Console& c, const char* arg,
                     GPIO_TypeDef*& port, uint16_t& pin)
{
  unsigned long id = strtoul(arg, nullptr, 10);
  if (id == 1)
  {
    port = DEBUG_LED_1_GPIO_Port;
    pin  = DEBUG_LED_1_Pin;
    return true;
  }
  if (id == 2)
  {
    port = DEBUG_LED_2_GPIO_Port;
    pin  = DEBUG_LED_2_Pin;
    return true;
  }
  c.printf("led: invalid LED '%s' — use 1 or 2\r\n", arg);
  return false;
}

static void handleLed(Console& c, int argc, char* argv[])
{
  if (argc < 2 || strcmp(argv[1], "help") == 0)
  {
    printUsage(c);
    return;
  }

  if (argc < 3)
  {
    printUsage(c);
    return;
  }

  GPIO_TypeDef* port;
  uint16_t      pin;
  if (!parseLed(c, argv[2], port, pin))
  {
    return;
  }

  if (strcmp(argv[1], "on") == 0)
  {
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    c.printf("LED %s on\r\n", argv[2]);
    return;
  }

  if (strcmp(argv[1], "off") == 0)
  {
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    c.printf("LED %s off\r\n", argv[2]);
    return;
  }

  if (strcmp(argv[1], "toggle") == 0)
  {
    HAL_GPIO_TogglePin(port, pin);
    c.printf("LED %s toggled\r\n", argv[2]);
    return;
  }

  if (strcmp(argv[1], "blink") == 0)
  {
    c.printf("LED %s blinking — any key to stop\r\n", argv[2]);
    while (!c.interrupted())
    {
      HAL_GPIO_TogglePin(port, pin);
      HAL_Delay(250);
    }
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    return;
  }

  c.printf("led: unknown subcommand '%s'  (try 'led help')\r\n", argv[1]);
}

void registerLedTests(Console& c)
{
  c.registerCommand("led", "Debug LED tests: led <on|off|toggle|blink> <1|2>", handleLed);
}
