/*******************************************************************************
 * @file Console.cpp
 * @brief USB-CDC command console implementation.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#include "Console.hpp"

#include <cstring>
#include <cstdio>
#include <cstdarg>
#include <cctype>

// strcasecmp / strncasecmp are POSIX, not standard C++17.
// Provide portable replacements using <cctype>.
static int s_strcasecmp(const char* a, const char* b)
{
  while (*a && *b)
  {
    int diff = std::tolower(static_cast<unsigned char>(*a))
             - std::tolower(static_cast<unsigned char>(*b));
    if (diff != 0) return diff;
    ++a; ++b;
  }
  return std::tolower(static_cast<unsigned char>(*a))
       - std::tolower(static_cast<unsigned char>(*b));
}

static int s_strncasecmp(const char* a, const char* b, std::size_t n)
{
  for (std::size_t i = 0U; i < n; ++i)
  {
    int diff = std::tolower(static_cast<unsigned char>(a[i]))
             - std::tolower(static_cast<unsigned char>(b[i]));
    if (diff != 0) return diff;
    if (a[i] == '\0') return 0;
  }
  return 0;
}

Console::Console(UART_HandleTypeDef* uart)
  : m_uart(uart)
  , m_commandCount(0U)
  , m_lineIdx(0U)
  , m_lineReady(false)
  , m_stopFlag(false)
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
  m_stopFlag = true; // Any incoming byte signals the user; streaming loops check this.
  for (uint32_t i = 0U; i < len; ++i)
  {
    char c = static_cast<char>(buf[i]);

    // Echo character back to the terminal.
    HAL_UART_Transmit(m_uart, reinterpret_cast<uint8_t*>(&c), 1U, HAL_MAX_DELAY);

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

bool Console::interrupted()
{
  bool f  = m_stopFlag;
  m_stopFlag = false;
  return f;
}

void Console::poll()
{
  if (!m_lineReady)
  {
    return;
  }
  m_lineReady = false;
  m_stopFlag  = false; // Discard input that formed the command itself.

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
    HAL_UART_Transmit(m_uart,
                      reinterpret_cast<const uint8_t*>(str),
                      len,
                      HAL_MAX_DELAY);
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

  // Built-ins: help, about
  if (s_strncasecmp(argv[0], "help", 4U) == 0)
  {
    printHelp();
    return;
  }

  if (s_strncasecmp(argv[0], "about", 5U) == 0)
  {
    printAbout();
    return;
  }

  // Search registered commands (case-insensitive).
  for (uint8_t i = 0U; i < m_commandCount; ++i)
  {
    if (s_strcasecmp(argv[0], m_commands[i].name) == 0)
    {
      m_commands[i].handler(*this, argc, argv);
      return;
    }
  }

  printf("Unknown command: '%s'  (type 'help' for list)\r\n", argv[0]);
}

void Console::printHelp()
{
  println("Available commands:");
  println("  help             Show this message");
  println("  about            Board and firmware information");
  for (uint8_t i = 0U; i < m_commandCount; ++i)
  {
    printf("  %-16s %s\r\n",
           m_commands[i].name ? m_commands[i].name : "?",
           m_commands[i].help ? m_commands[i].help : "");
  }
}

void Console::printAbout()
{
  println("-----------------------------------");
  println("  Integrated Motor Controller");
  println("  MCU  : STM32H563  @ 250 MHz (Cortex-M33)");
  println("  Build: " __DATE__ " " __TIME__ " UTC");
  println("  Port : UART4 @ 420000 baud (CRSF connector)");
  println("-----------------------------------");
}

/* EOF -----------------------------------------------------------------------*/
