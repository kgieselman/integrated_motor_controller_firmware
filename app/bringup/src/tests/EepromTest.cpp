/*******************************************************************************
 * @file EepromTest.cpp
 * @brief Bringup test stubs for the M24C64 EEPROM (I2C1).
 *
 * Subcommands:
 *   eeprom read  <addr>       — Read one byte from <addr> (hex, e.g. 0x10).
 *   eeprom write <addr> <val> — Write <val> (hex) to <addr> and verify.
 ******************************************************************************/

#include "tests/EepromTest.hpp"

#include <cstring>
#include <cstdlib>

static void handleEeprom(Console& c, int argc, char* argv[])
{
  if (argc < 2 || strcmp(argv[1], "help") == 0)
  {
    c.println("Usage: eeprom <subcommand>");
    c.println("  read  <addr>        Read one byte  (addr in hex, e.g. 0x10)");
    c.println("  write <addr> <val>  Write one byte (addr and val in hex)");
    return;
  }

  if (strcmp(argv[1], "read") == 0)
  {
    if (argc < 3)
    {
      c.println("Usage: eeprom read <addr>");
      return;
    }
    // TODO: parse argv[2] with strtoul, construct EEPROM(&hi2c1), call readByte()
    c.printf("eeprom read 0x%04lX — not yet implemented\r\n",
             strtoul(argv[2], nullptr, 0));
    return;
  }

  if (strcmp(argv[1], "write") == 0)
  {
    if (argc < 4)
    {
      c.println("Usage: eeprom write <addr> <val>");
      return;
    }
    // TODO: parse argv[2]/argv[3], construct EEPROM(&hi2c1), call writeByte() + verify
    c.printf("eeprom write 0x%04lX 0x%02lX — not yet implemented\r\n",
             strtoul(argv[2], nullptr, 0),
             strtoul(argv[3], nullptr, 0));
    return;
  }

  c.printf("eeprom: unknown subcommand '%s'  (try 'eeprom help')\r\n", argv[1]);
}

void registerEepromTests(Console& c)
{
  c.registerCommand("eeprom", "M24C64 EEPROM tests: eeprom <read|write>", handleEeprom);
}
