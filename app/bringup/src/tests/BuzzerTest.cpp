/*******************************************************************************
 * @file BuzzerTest.cpp
 * @brief Bringup test for the AT-0927-TT-6-R buzzer on PC14 (DEBUG_BUZZER).
 *
 * The buzzer is a passive magnetic transducer (2–4 Vpp, resonant at 2730 Hz).
 * It requires an external toggling signal; the DWT cycle counter is used for
 * sub-millisecond half-period delays at 250 MHz SYSCLK.
 *
 * Subcommands:
 *   buzzer on               — Drive pin high (DC, no tone — for continuity check).
 *   buzzer off              — Drive pin low.
 *   buzzer beep [hz] [ms]   — Toggle at hz for ms  (defaults: 2730 Hz, 500 ms).
 *   buzzer test             — Three beeps at resonant frequency.
 ******************************************************************************/

extern "C"
{
#include "stm32h5xx_hal.h"
#include "main.h"
}

#include "tests/BuzzerTest.hpp"

#include <cstring>
#include <cstdlib>

static constexpr uint32_t kResonantFreqHz = 2730U;
static constexpr uint32_t kDefaultDurMs   = 500U;

static void dwtInit(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

/// Busy-wait for exactly halfPeriodCycles core clock cycles.
static void dwtWaitCycles(uint32_t halfPeriodCycles)
{
  uint32_t start = DWT->CYCCNT;
  while ((DWT->CYCCNT - start) < halfPeriodCycles) {}
}

/// Toggle the buzzer pin at freqHz for durMs, stopping early on keypress.
static void beep(Console& c, uint32_t freqHz, uint32_t durMs)
{
  dwtInit();
  const uint32_t halfPeriodCycles = SystemCoreClock / (2U * freqHz);
  const uint32_t endTick          = HAL_GetTick() + durMs;

  while (HAL_GetTick() < endTick && !c.interrupted())
  {
    HAL_GPIO_WritePin(DEBUG_BUZZER_GPIO_Port, DEBUG_BUZZER_Pin, GPIO_PIN_SET);
    dwtWaitCycles(halfPeriodCycles);
    HAL_GPIO_WritePin(DEBUG_BUZZER_GPIO_Port, DEBUG_BUZZER_Pin, GPIO_PIN_RESET);
    dwtWaitCycles(halfPeriodCycles);
  }

  HAL_GPIO_WritePin(DEBUG_BUZZER_GPIO_Port, DEBUG_BUZZER_Pin, GPIO_PIN_RESET);
}

static void handleBuzzer(Console& c, int argc, char* argv[])
{
  if (argc < 2 || strcmp(argv[1], "help") == 0)
  {
    c.println("Usage: buzzer <subcommand>");
    c.println("  on                Drive pin high (no tone — continuity check)");
    c.println("  off               Drive pin low");
    c.printf ("  beep [hz] [ms]    Toggle at hz for ms"
              "  (defaults: %lu Hz, %lu ms)\r\n",
              static_cast<unsigned long>(kResonantFreqHz),
              static_cast<unsigned long>(kDefaultDurMs));
    c.println("  test              Three beeps at resonant frequency");
    return;
  }

  if (strcmp(argv[1], "on") == 0)
  {
    HAL_GPIO_WritePin(DEBUG_BUZZER_GPIO_Port, DEBUG_BUZZER_Pin, GPIO_PIN_SET);
    c.println("buzzer on");
    return;
  }

  if (strcmp(argv[1], "off") == 0)
  {
    HAL_GPIO_WritePin(DEBUG_BUZZER_GPIO_Port, DEBUG_BUZZER_Pin, GPIO_PIN_RESET);
    c.println("buzzer off");
    return;
  }

  if (strcmp(argv[1], "beep") == 0)
  {
    uint32_t freq = (argc >= 3) ? static_cast<uint32_t>(strtoul(argv[2], nullptr, 10))
                                : kResonantFreqHz;
    uint32_t dur  = (argc >= 4) ? static_cast<uint32_t>(strtoul(argv[3], nullptr, 10))
                                : kDefaultDurMs;
    c.printf("buzzer beep %lu Hz %lu ms — any key to stop\r\n",
             static_cast<unsigned long>(freq), static_cast<unsigned long>(dur));
    beep(c, freq, dur);
    return;
  }

  if (strcmp(argv[1], "test") == 0)
  {
    c.printf("buzzer test — three beeps at %lu Hz\r\n",
             static_cast<unsigned long>(kResonantFreqHz));
    for (int i = 0; i < 3; ++i)
    {
      beep(c, kResonantFreqHz, 200U);
      HAL_Delay(150U);
    }
    c.println("PASS");
    return;
  }

  c.printf("buzzer: unknown subcommand '%s'  (try 'buzzer help')\r\n", argv[1]);
}

void registerBuzzerTests(Console& c)
{
  c.registerCommand("buzzer", "Buzzer tests (PC14): buzzer <on|off|beep|test>", handleBuzzer);
}
