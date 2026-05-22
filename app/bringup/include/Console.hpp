/*******************************************************************************
 * @file Console.hpp
 * @brief UART command console for the bring-up firmware.
 *
 * Provides a minimal line-oriented command dispatcher over a HAL UART
 * peripheral. Commands are registered as function pointers so each test
 * module can self-register without touching the core console code.
 *
 * Currently wired to UART4 (PC10/PC11, 420000 baud) via the CRSF connector.
 * Swap to USART1 (PB14/PB15, 115200 baud) by changing the handle passed to
 * the constructor once that connector is populated.
 *
 * Usage:
 * @code
 *   Console console(&huart4);
 *   console.registerCommand("imu", "Test IMU WHO_AM_I and streaming", testImu);
 *   console.registerCommand("motor", "Test left/right motor channels", testMotors);
 *
 *   // In main loop:
 *   console.poll();
 * @endcode
 *
 * @note Receive bytes are fed via HAL_UART_RxCpltCallback — call
 *       Console::feed() from there after re-arming HAL_UART_Receive_IT().
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#pragma once

#include "stm32h5xx_hal.h"
#include <cstdint>

/// Maximum number of commands that can be registered.
static constexpr uint8_t kMaxCommands = 32U;

/// Maximum length of one input line (including null terminator).
static constexpr uint8_t kMaxLineLen  = 128U;

/// Maximum number of tokenised arguments per command (including command name).
static constexpr uint8_t kMaxArgs     = 8U;

/**
 * @brief Signature for a bring-up test command handler.
 *
 * @param argc Number of arguments (argv[0] is the command name).
 * @param argv Null-terminated argument strings.
 */
using CommandHandler = void (*)(int argc, char* argv[]);

/**
 * @brief Simple line-buffered USB-CDC command console.
 *
 * Not thread-safe — call from a single task or the main loop only.
 */
class Console
{
public:
  /**
   * @brief Construct the Console.
   *
   * @param uart HAL UART handle to use for TX and RX.
   *             Pass &huart4 (CRSF port) for now; swap to &huart1 (USART1
   *             growth connector) once that hardware is populated.
   *
   * Call registerCommand() to populate the command table, then poll()
   * in the main loop.
   */
  explicit Console(UART_HandleTypeDef* uart);

  /**
   * @brief Register a command with the console.
   *
   * Commands are matched case-insensitively against the first whitespace-
   * delimited token on each input line.
   *
   * @param name    Command keyword (e.g. "imu", "motor", "adc").
   * @param help    One-line description shown by the built-in "help" command.
   * @param handler Function called when the command is entered.
   * @return true on success, false if the command table is full.
   */
  bool registerCommand(const char*    name,
                       const char*    help,
                       CommandHandler handler);

  /**
   * @brief Feed received bytes from the CDC_Receive_FS() callback.
   *
   * Accumulates bytes into the internal line buffer. Complete lines
   * (terminated by \\r or \\n) are queued for processing by poll().
   *
   * @param buf   Pointer to received data.
   * @param len   Number of bytes received.
   */
  void feed(const uint8_t* buf, uint32_t len);

  /**
   * @brief Process any complete input lines and dispatch commands.
   *
   * Call from the main loop. Non-blocking; returns immediately if no
   * complete line is pending.
   */
  void poll();

  /**
   * @brief Print a null-terminated string to the USB-CDC console.
   *
   * Wraps CDC_Transmit_FS(). Safe to call from command handlers.
   *
   * @param str The string to transmit (no newline appended automatically).
   */
  void print(const char* str);

  /**
   * @brief Print a formatted string to the USB-CDC console.
   *
   * Uses a fixed internal buffer (kMaxLineLen bytes). Truncates on overflow.
   *
   * @param fmt printf-style format string.
   * @param ... Format arguments.
   */
  void printf(const char* fmt, ...) __attribute__((format(printf, 2, 3)));

  /**
   * @brief Print a string followed by "\\r\\n".
   *
   * @param str The string to print.
   */
  void println(const char* str);

private:
  /// A registered command entry.
  struct CommandEntry
  {
    const char*    name;
    const char*    help;
    CommandHandler handler;
  };

  UART_HandleTypeDef* m_uart;            ///< UART peripheral used for I/O.

  CommandEntry m_commands[kMaxCommands]; ///< Registered command table.
  uint8_t      m_commandCount;           ///< Number of registered commands.

  char    m_lineBuf[kMaxLineLen]; ///< Incoming character accumulator.
  uint8_t m_lineIdx;              ///< Current write position in m_lineBuf.
  bool    m_lineReady;            ///< True when a complete line is buffered.

  /**
   * @brief Dispatch a complete null-terminated command line.
   *
   * Tokenises the line, looks up the command, and calls the handler.
   * Prints an error message for unrecognised commands.
   *
   * @param line Null-terminated command line.
   */
  void dispatch(char* line);

  /**
   * @brief Built-in "help" command implementation.
   *
   * Lists all registered commands and their descriptions.
   */
  void printHelp();
};

/* EOF -----------------------------------------------------------------------*/
