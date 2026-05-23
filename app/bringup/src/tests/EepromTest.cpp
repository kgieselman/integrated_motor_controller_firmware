/*******************************************************************************
 * @file EepromTest.cpp
 * @brief Bringup tests for the M24C64 EEPROM (I2C1).
 *
 * Subcommands:
 *   eeprom about             — I2C address and device detection result.
 *   eeprom test              — Write/readback patterns at scratch addr 0x1FFF.
 *   eeprom read  <addr>      — Read one byte from <addr> (hex, e.g. 0x10).
 *   eeprom write <addr> <val>— Write <val> (hex) to <addr> and verify.
 ******************************************************************************/

extern "C"
{
#include "i2c.h"
#include "main.h"
}

#include "tests/EepromTest.hpp"
#include "EEPROM.hpp"

#include <cstring>
#include <cstdlib>

static constexpr uint8_t  kEepromAddr    = 0x50U;   // 7-bit address (A2=A1=A0=GND)
static constexpr uint16_t kScratchAddr   = 0x1FFFU; // top byte — reserved for test use

static void handleEeprom(Console& c, int argc, char* argv[])
{
  if (argc < 2 || strcmp(argv[1], "help") == 0)
  {
    c.println("Usage: eeprom <subcommand>");
    c.println("  about               I2C address and device presence");
    c.println("  test                Write/readback patterns at scratch addr 0x1FFF");
    c.println("  read  <addr>        Read one byte  (addr in hex, e.g. 0x10)");
    c.println("  write <addr> <val>  Write one byte (addr and val in hex)");
    return;
  }

  if (strcmp(argv[1], "about") == 0)
  {
    c.println("M24C64 EEPROM");
    c.println("  Bus         : I2C1  (PB6=SCL, PB7=SDA)");
    c.printf ("  7-bit addr  : 0x%02X\r\n", kEepromAddr);
    bool found = (HAL_I2C_IsDeviceReady(&hi2c1,
                                        static_cast<uint16_t>(kEepromAddr << 1U),
                                        3U, 10U) == HAL_OK);
    c.printf("  Found       : %s\r\n", found ? "YES" : "NO");
    return;
  }

  if (strcmp(argv[1], "test") == 0)
  {
    static constexpr uint8_t kPatterns[] = { 0xA5U, 0x5AU, 0xFFU, 0x00U };

    EEPROM eeprom(&hi2c1, kEepromAddr,
                  EEPROM_WRITE_PROTECT_GPIO_Port, EEPROM_WRITE_PROTECT_Pin);

    // Preserve original value so the test is non-destructive.
    uint8_t original = 0U;
    if (!eeprom.readByte(kScratchAddr, original))
    {
      c.println("FAIL  could not read scratch address");
      return;
    }

    bool passed = true;
    for (uint8_t pattern : kPatterns)
    {
      if (!eeprom.writeByte(kScratchAddr, pattern))
      {
        c.printf("FAIL  write 0x%02X\r\n", pattern);
        passed = false;
        break;
      }
      uint8_t readback = 0U;
      if (!eeprom.readByte(kScratchAddr, readback))
      {
        c.printf("FAIL  read after write 0x%02X\r\n", pattern);
        passed = false;
        break;
      }
      if (readback != pattern)
      {
        c.printf("FAIL  wrote 0x%02X, read back 0x%02X\r\n", pattern, readback);
        passed = false;
        break;
      }
      c.printf("  0x%02X -> 0x%02X  OK\r\n", pattern, readback);
    }

    // Restore original value regardless of pass/fail.
    eeprom.writeByte(kScratchAddr, original);

    c.println(passed ? "PASS" : "FAIL");
    return;
  }

  if (strcmp(argv[1], "read") == 0)
  {
    if (argc < 3)
    {
      c.println("Usage: eeprom read <addr>");
      return;
    }
    uint16_t addr = static_cast<uint16_t>(strtoul(argv[2], nullptr, 0));
    EEPROM eeprom(&hi2c1, kEepromAddr,
                  EEPROM_WRITE_PROTECT_GPIO_Port, EEPROM_WRITE_PROTECT_Pin);
    uint8_t val = 0U;
    if (eeprom.readByte(addr, val))
    {
      c.printf("eeprom[0x%04X] = 0x%02X (%u)\r\n", addr, val, val);
    }
    else
    {
      c.printf("eeprom read 0x%04X -- FAILED\r\n", addr);
    }
    return;
  }

  if (strcmp(argv[1], "write") == 0)
  {
    if (argc < 4)
    {
      c.println("Usage: eeprom write <addr> <val>");
      return;
    }
    uint16_t addr = static_cast<uint16_t>(strtoul(argv[2], nullptr, 0));
    uint8_t  val  = static_cast<uint8_t>(strtoul(argv[3], nullptr, 0));
    EEPROM eeprom(&hi2c1, kEepromAddr,
                  EEPROM_WRITE_PROTECT_GPIO_Port, EEPROM_WRITE_PROTECT_Pin);
    if (!eeprom.writeByte(addr, val))
    {
      c.printf("eeprom write 0x%04X -- WRITE FAILED\r\n", addr);
      return;
    }
    uint8_t readback = 0U;
    if (!eeprom.readByte(addr, readback))
    {
      c.printf("eeprom write 0x%04X -- VERIFY READ FAILED\r\n", addr);
      return;
    }
    if (readback == val)
    {
      c.printf("eeprom[0x%04X] = 0x%02X -- OK\r\n", addr, val);
    }
    else
    {
      c.printf("eeprom[0x%04X] wrote 0x%02X, read back 0x%02X -- MISMATCH\r\n",
               addr, val, readback);
    }
    return;
  }

  c.printf("eeprom: unknown subcommand '%s'  (try 'eeprom help')\r\n", argv[1]);
}

void registerEepromTests(Console& c)
{
  c.registerCommand("eeprom", "M24C64 EEPROM tests: eeprom <about|test|read|write>", handleEeprom);
}
