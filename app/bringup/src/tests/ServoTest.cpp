/*******************************************************************************
 * @file ServoTest.cpp
 * @brief Bringup test commands for RC servo PWM outputs (TIM2 CH1-3).
 *
 * Subcommands:
 *   servo set   <ch> <us>              — Set channel to <us> µs (no clamping).
 *   servo sweep <ch> [min_us] [max_us] — Sweep min→max→min µs (any key stops).
 ******************************************************************************/

extern "C"
{
#include "tim.h"
}

#include "tests/ServoTest.hpp"
#include "ServoChannel.hpp"

#include <cstring>
#include <cstdlib>

static constexpr uint8_t kNumChannels = 3U;

static const uint32_t kTimChannels[kNumChannels] = {
  TIM_CHANNEL_1,
  TIM_CHANNEL_2,
  TIM_CHANNEL_3,
};

static bool parseChannel(Console& c, const char* arg, uint8_t& ch)
{
  unsigned long val = strtoul(arg, nullptr, 0);
  if (val >= kNumChannels)
  {
    c.printf("servo: channel must be 0-%u\r\n", kNumChannels - 1U);
    return false;
  }
  ch = static_cast<uint8_t>(val);
  return true;
}

// Write directly to the compare register, bypassing the driver's 1000-2000 µs
// safety clamp so bringup can probe the servo's actual hardware range.
static void setRawUs(uint8_t ch, uint16_t us)
{
  __HAL_TIM_SET_COMPARE(&htim2, kTimChannels[ch], static_cast<uint32_t>(us));
}

static void handleServo(Console& c, int argc, char* argv[])
{
  if (argc < 2 || strcmp(argv[1], "help") == 0)
  {
    c.println("Usage: servo <subcommand>");
    c.println("  set   <ch> <us>              Set channel pulse width (unclamped)");
    c.println("  sweep <ch> [min_us] [max_us] Sweep min-max us, default 1000-2000");
    return;
  }

  if (strcmp(argv[1], "set") == 0)
  {
    if (argc < 4)
    {
      c.println("Usage: servo set <ch> <us>");
      return;
    }
    uint8_t ch;
    if (!parseChannel(c, argv[2], ch))
    {
      return;
    }
    uint16_t us = static_cast<uint16_t>(strtoul(argv[3], nullptr, 0));
    ServoChannel servo(&htim2, kTimChannels[ch]);
    if (!servo.init())
    {
      c.println("servo: PWM start failed");
      return;
    }
    setRawUs(ch, us);
    c.printf("servo ch%u -> %u us\r\n", ch, us);
    return;
  }

  if (strcmp(argv[1], "sweep") == 0)
  {
    if (argc < 3)
    {
      c.println("Usage: servo sweep <ch> [min_us] [max_us]");
      return;
    }
    uint8_t ch;
    if (!parseChannel(c, argv[2], ch))
    {
      return;
    }
    uint16_t minUs = (argc >= 4) ? static_cast<uint16_t>(strtoul(argv[3], nullptr, 0))
                                 : ServoChannel::kPulseMinUs;
    uint16_t maxUs = (argc >= 5) ? static_cast<uint16_t>(strtoul(argv[4], nullptr, 0))
                                 : ServoChannel::kPulseMaxUs;
    if (minUs >= maxUs)
    {
      c.println("servo: min_us must be less than max_us");
      return;
    }
    ServoChannel servo(&htim2, kTimChannels[ch]);
    if (!servo.init())
    {
      c.println("servo: PWM start failed");
      return;
    }
    c.printf("servo ch%u sweeping %u-%u us  (any key to stop)\r\n", ch, minUs, maxUs);
    bool stopped = false;
    while (!stopped)
    {
      for (int32_t us = minUs; us <= static_cast<int32_t>(maxUs); us += 10)
      {
        setRawUs(ch, static_cast<uint16_t>(us));
        c.printf("\r  %u us  ", static_cast<uint16_t>(us));
        HAL_Delay(20U);
        if (c.interrupted()) { stopped = true; break; }
      }
      if (stopped) { break; }
      for (int32_t us = maxUs; us >= static_cast<int32_t>(minUs); us -= 10)
      {
        setRawUs(ch, static_cast<uint16_t>(us));
        c.printf("\r  %u us  ", static_cast<uint16_t>(us));
        HAL_Delay(20U);
        if (c.interrupted()) { stopped = true; break; }
      }
    }
    setRawUs(ch, ServoChannel::kPulseMidUs);
    c.printf("\r\nservo ch%u stopped at %u us\r\n", ch, ServoChannel::kPulseMidUs);
    return;
  }

  c.printf("servo: unknown subcommand '%s'  (try 'servo help')\r\n", argv[1]);
}

void registerServoTests(Console& c)
{
  c.registerCommand("servo", "Servo PWM tests: servo <set|sweep>", handleServo);
}
