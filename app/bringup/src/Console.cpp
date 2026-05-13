/*******************************************************************************
 * @file Console.cpp
 * @brief USB-CDC command console implementation.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#include "Console.hpp"

// CubeMX-generated USB CDC transmit function.
// Declared here to avoid pulling in the full USB Device header tree.
extern "C" uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

#include <cstring>
#include <cstdio>
#include <cstdarg>
#include <cctype>

Console::Console()
  : m_commandCount(0U)
  , m_lineIdx(0U)
  , m_lineReady(false)
{
  for (uint8_t i = 0U; i < kMaxCommands; ++i)
  {
    m_commands[i] = {nullptr, nullptr, nullptr};
  }
  m_lineBuf[0] = '\0';
}

bool Console::registerCommand(const char*    name,
                               const char*    help,
                               CommandHandler handler)
{
  if (m_commandCount >= kMaxCommands || name == nullptr || handler == nullptr)
  {
    return false;
  }
  m_commands[m_commandCount++] = {name, help, handler};
  return true;
}

void Console::feed(const uint8_t* buf, uint32_t len)
{
  for (uint32_t i = 0U; i < len; ++i)
  {
    char c = static_cast<char>(buf[i]);

    // Echo character back to the terminal.
    CDC_Transmit_FS(reinterpret_cast<uint8_t*>(&c), 1U);

    if (c == '\r' || c == '\n')
    {
      if (m_lineIdx > 0U)
      {
        m_lineBuf[m_lineIdx] = '\0';
        m_lineReady          = true;
        m_lineIdx            = 0U;
      }
    }
    else if (c == '\b' || c == 0x7FU)
    {
      // Backspace / DEL
      if (m_lineIdx > 0U)
      {
        m_lineIdx--;
      }
    }
    else if (m_lineIdx < (kMaxLineLen - 1U))
    {
      m_lineBuf[m_lineIdx++] = c;
    }
  }
}

void Console::poll()
{
  if (!m_lineReady)
  {
    return;
  }
  m_lineReady = false;

  // Work on a local copy so feed() can safely refill m_lineBuf while we parse.
  char local[kMaxLineLen];
  strncpy(local, m_lineBuf, kMaxLineLen - 1U);
  local[kMaxLineLen - 1U] = '\0';

  print("\r\n");
  dispatch(local);
  print("\r\n> ");
}

void Console::print(const char* str)
{
  if (str == nullptr)
  {
    return;
  }
  uint16_t len = static_cast<uint16_t>(strlen(str));
  if (len > 0U)
  {
    // CDC_Transmit_FS takes a non-const pointer.
    CDC_Transmit_FS(reinterpret_cast<uint8_t*>(const_cast<char*>(str)), len);
    // Brief busy-wait for USB to flush (acceptable for bring-up debug console).
    HAL_Delay(1U);
  }
}

void Console::printf(const char* fmt, ...)
{
  char buf[kMaxLineLen];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  print(buf);
}

void Console::println(const char* str)
{
  print(str);
  print("\r\n");
}

/* Private Helpers -----------------------------------------------------------*/

void Console::dispatch(char* line)
{
  // Tokenise: split on whitespace into argv[].
  char*  argv[kMaxArgs] = {};
  int    argc           = 0;
  char*  tok            = strtok(line, " \t");

  while (tok != nullptr && argc < kMaxArgs)
  {
    argv[argc++] = tok;
    tok          = strtok(nullptr, " \t");
  }

  if (argc == 0)
  {
    return; // blank line
  }

  // Built-in: help
  if (strncasecmp(argv[0], "help", 4U) == 0)
  {
    printHelp();
    return;
  }

  // Search registered commands (case-insensitive).
  for (uint8_t i = 0U; i < m_commandCount; ++i)
  {
    if (strcasecmp(argv[0], m_commands[i].name) == 0)
    {
      m_commands[i].handler(argc, argv);
      return;
    }
  }

  printf("Unknown command: '%s'  (type 'help' for list)\r\n", argv[0]);
}

void Console::printHelp()
{
  println("Integrated Motor Controller bring-up console — available commands:");
  println("  help             Show this message");
  for (uint8_t i = 0U; i < m_commandCount; ++i)
  {
    printf("  %-16s %s\r\n",
           m_commands[i].name ? m_commands[i].name : "?",
           m_commands[i].help ? m_commands[i].help : "");
  }
}

/* EOF -----------------------------------------------------------------------*/
