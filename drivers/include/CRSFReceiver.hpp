/*******************************************************************************
 * @file CRSFReceiver.hpp
 * @brief CRSF (Crossfire Serial Protocol) receiver parser for ELRS radio links.
 *
 * Parses the CRSF byte stream from UART4 (PC11 RX / PC10 TX) connected to the
 * ELRS ESP32+SX1280 module on the WROOM footprint. From the STM32's perspective
 * the radio is a CRSF-speaking UART peer at 420 000 baud.
 *
 * RECEIVE MODEL — read this before changing anything here.
 *
 * UART4 RX DMA is configured **circular** in the .ioc
 * (GPDMA1.CIRCULARMODE_GPDMACH1=ENABLE on GPDMA1_REQUEST_UART4_RX). That has
 * two consequences that the previous implementation got wrong:
 *
 *  1. The transfer never stops, so it must NOT be re-armed. Re-arming restarts
 *     a transfer that was never meant to end.
 *  2. The @c Size reported by HAL_UARTEx_RxEventCallback is the current write
 *     position measured from the start of the buffer — not a count of new bytes
 *     since the last event. Treating it as a count re-copies every previously
 *     consumed byte on every idle event.
 *
 * So the ISR does the minimum possible: it records the head position and
 * nothing else. update(), running in task context, drains the ring from its own
 * tail index and does all parsing. The frame accumulator is therefore touched
 * by exactly one context, which removes every shared-state race in the parser
 * along with the critical sections that used to guard them.
 *
 * The DMA ring must be drained faster than it fills. At 250 Hz with ~26-byte
 * frames that is roughly 6.5 kB/s, so kDmaRxBufLen gives on the order of 40 ms
 * of slack — comfortable for a 200 Hz control cycle.
 *
 * CRSF frame format:
 * @code
 *   [SYNC 0xC8] [LEN] [TYPE] [PAYLOAD...] [CRC8]
 * @endcode
 *  - LEN = number of bytes from TYPE through CRC inclusive.
 *  - RC_CHANNELS_PACKED (type 0x16): 22-byte payload, 16 × 11-bit channels.
 *  - Channels are in the range [172, 1811], mapping to [1000, 2000] µs.
 *
 * @see ExpressLRS CRSF specification v2
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#pragma once

extern "C"
{
#include "stm32h5xx_hal.h"
}

#include <cstdint>
#include <cstring>

/**
 * @brief CRSF frame parser and RC channel decoder.
 */
class CRSFReceiver final
{
public:
  static constexpr uint8_t  kSyncByte       = 0xC8U; ///< CRSF frame sync byte
  static constexpr uint8_t  kTypeRcChannels = 0x16U; ///< RC_CHANNELS_PACKED frame type
  static constexpr uint8_t  kMaxFrameLen    = 64U;   ///< Max CRSF frame size (bytes)
  static constexpr uint16_t kDmaRxBufLen    = 256U;  ///< Circular DMA ring (~40 ms of slack)
  static constexpr uint8_t  kNumChannels    = 16U;   ///< Number of RC channels
  static constexpr uint16_t kCrsfMin        = 172U;  ///< CRSF channel minimum value
  static constexpr uint16_t kCrsfMax        = 1811U; ///< CRSF channel maximum value
  static constexpr uint16_t kPwmMin         = 1000U; ///< Equivalent PWM µs minimum
  static constexpr uint16_t kPwmMax         = 2000U; ///< Equivalent PWM µs maximum

  /**
   * @brief Construct a CRSFReceiver.
   *
   * @param huart Pointer to the HAL UART handle (UART4, 420 000 baud).
   */
  explicit CRSFReceiver(UART_HandleTypeDef* huart);

  /**
   * @brief Start the circular DMA receive with IDLE-line detection.
   *
   * Call once after MX_UART4_Init() and MX_GPDMA1_Init(). The transfer then
   * runs for the life of the program and is never re-armed.
   *
   * @return true on success.
   */
  bool init();

  /**
   * @brief Record a DMA Rx event (IDLE line, half-transfer or transfer-complete).
   *
   * Wire this to HAL_UARTEx_RxEventCallback in your application:
   * @code
   *   void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* h, uint16_t size) {
   *       if (h == crsfReceiver.uartHandle()) crsfReceiver.onDmaRxEvent(size);
   *   }
   * @endcode
   *
   * @param size Current DMA write position, measured in bytes from the start of
   *             the ring — this is what HAL reports in circular mode.
   *
   * @note ISR-safe and non-blocking: it performs one aligned 16-bit store and
   *       returns. It does not copy, parse, or re-arm.
   */
  void onDmaRxEvent(uint16_t size);

  /**
   * @brief Drain the DMA ring and process any complete frames.
   *
   * Decodes RC_CHANNELS_PACKED frames and updates the internal channel array.
   * Call from a task at a steady rate; must be called often enough that the
   * circular DMA cannot lap the tail (see the receive-model note above).
   *
   * @return true if at least one new RC frame was decoded.
   */
  bool update();

  /**
   * @brief Return the last received value for a channel in CRSF units.
   *
   * @param ch Channel index (0–15).
   * @return Channel value in [172, 1811], or kCrsfMin on invalid index.
   */
  uint16_t channel(uint8_t ch) const;

  /**
   * @brief Return the last received value for a channel in PWM microseconds.
   *
   * Linearly maps [kCrsfMin, kCrsfMax] → [kPwmMin, kPwmMax].
   *
   * @param ch Channel index (0–15).
   * @return Channel value in [1000, 2000] µs.
   */
  uint16_t channelUs(uint8_t ch) const;

  /**
   * @brief Return the last received value as a normalised float.
   *
   * @param ch Channel index (0–15).
   * @return Value in [-1.0, 1.0]; mid-stick = 0.0.
   */
  float channelNorm(uint8_t ch) const;

  /**
   * @brief Check whether a fresh RC frame has been received since last call.
   *
   * Clears the flag on read.
   *
   * @return true if new data is available.
   */
  bool hasNewData();

  /**
   * @brief Milliseconds since the last valid RC frame was received.
   *
   * Use this to detect link loss.
   *
   * @return Age in milliseconds.
   */
  uint32_t lastFrameAgeMs() const;

  /**
   * @brief Number of bytes discarded because the frame accumulator was full.
   *
   * A non-zero and growing value means the parser is being fed bytes it cannot
   * resynchronise on — a wiring, baud rate or framing problem, not a bug in the
   * caller. Exposed for telemetry rather than for control decisions.
   *
   * @return Cumulative dropped-byte count since reset.
   */
  uint32_t droppedBytes() const;

  /**
   * @brief Expose the HAL UART handle for use in the application Rx callback.
   *
   * @return Pointer to the HAL UART handle.
   */
  UART_HandleTypeDef* uartHandle() const;

private:
  /**
   * @brief Copy any bytes the DMA has written since the last drain.
   *
   * Handles the wrap at the end of the ring. Task context only.
   */
  void drainDmaRing();

  /**
   * @brief Append one byte to the frame accumulator, making room if needed.
   *
   * On overflow the oldest byte is dropped rather than the whole accumulator
   * being cleared, so a complete frame sitting at the tail is not thrown away
   * along with the junk in front of it.
   *
   * @param byte Byte to append.
   */
  void appendByte(uint8_t byte);

  /**
   * @brief Discard @p count bytes from the front of the accumulator.
   *
   * @param count Number of bytes to remove; clamped to the current fill.
   */
  void consume(uint16_t count);

  /**
   * @brief Validate CRC8/DVB-S2 over a received frame.
   *
   * @param frame Pointer to the frame bytes (starting at TYPE, not SYNC).
   * @param len   Number of bytes to check (TYPE + PAYLOAD, excluding CRC).
   * @param crc   Expected CRC byte.
   * @return true if CRC matches.
   */
  bool checkCrc(const uint8_t* frame, uint8_t len, uint8_t crc) const;

  /**
   * @brief Decode an RC_CHANNELS_PACKED payload into m_channels[].
   *
   * @param payload Pointer to the 22-byte RC payload.
   */
  void decodeRcChannels(const uint8_t* payload);

  UART_HandleTypeDef* m_huart; ///< HAL UART handle (UART4).

  /// Circular DMA ring. Written by the DMA controller, read in task context.
  volatile uint8_t m_dmaRxBuf[kDmaRxBufLen];

  /// Latest DMA write position, in bytes from the start of the ring. Written by
  /// the ISR, read by update(). Aligned 16-bit access is atomic on Cortex-M33,
  /// so no critical section is needed.
  volatile uint16_t m_dmaHead;

  /// Our read position in the ring. Task context only.
  uint16_t m_dmaTail;

  /// Frame accumulator and its fill level. Task context only — no ISR touches
  /// either, which is what makes the parser lock-free and race-free.
  uint8_t  m_rxBuf[kMaxFrameLen];
  uint16_t m_rxIdx;

  uint16_t      m_channels[kNumChannels]; ///< Last decoded channel values (CRSF units)
  volatile bool m_newData;                ///< Set when a fresh frame is decoded
  uint32_t      m_lastFrameTick;          ///< HAL_GetTick() at last valid frame
  uint32_t      m_droppedBytes;           ///< Bytes discarded on accumulator overflow
};

/* EOF -----------------------------------------------------------------------*/
