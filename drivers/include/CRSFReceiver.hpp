/*******************************************************************************
 * @file CRSFReceiver.hpp
 * @brief CRSF (Crossfire Serial Protocol) receiver parser for ELRS radio links.
 *
 * Parses the CRSF byte stream from UART4 (PC11 RX / PC10 TX) connected to the
 * ELRS ESP32+SX1280 module on the WROOM footprint. From the STM32's perspective
 * the radio is a CRSF-speaking UART peer at 420 000 baud.
 *
 * This implementation is DMA-driven with IDLE-line detection: HAL fills a DMA
 * staging buffer and fires HAL_UARTEx_RxEventCallback when the UART line goes
 * idle (or the buffer fills). Wire that callback to onDmaRxEvent(). Call
 * update() from your main loop or RTOS task to process complete frames without
 * blocking the callback.
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
class CRSFReceiver
{
public:
  static constexpr uint8_t  kSyncByte       = 0xC8U; ///< CRSF frame sync byte
  static constexpr uint8_t  kTypeRcChannels = 0x16U; ///< RC_CHANNELS_PACKED frame type
  static constexpr uint8_t  kMaxFrameLen    = 64U;   ///< Max CRSF frame size (bytes)
  static constexpr uint16_t kDmaRxBufLen    = 128U;  ///< DMA staging buffer (≥2 max frames)
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
   * @brief Start DMA receive with IDLE-line detection.
   *
   * Call once after MX_UART4_UART_Init() and MX_DMA_Init(). Arms
   * HAL_UARTEx_ReceiveToIdle_DMA into the internal DMA staging buffer.
   *
   * @return true on success.
   */
  bool init();

  /**
   * @brief Handle a completed DMA Rx transfer (IDLE-line or buffer-full event).
   *
   * Wire this to HAL_UARTEx_RxEventCallback in your application:
   * @code
   *   void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* h, uint16_t size) {
   *       if (h == crsfReceiver.uartHandle()) crsfReceiver.onDmaRxEvent(size);
   *   }
   * @endcode
   * Re-arms the DMA transfer internally. Safe to call from an ISR.
   *
   * @param size Number of bytes written to the DMA staging buffer since the
   *             last call (as reported by HAL).
   */
  void onDmaRxEvent(uint16_t size);

  /**
   * @brief Process any complete frames accumulated since the last call.
   *
   * Decodes RC_CHANNELS_PACKED frames and updates the internal channel array.
   * Safe to call from the main loop; frame processing is non-blocking.
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
   * Use this to detect link loss (e.g. > 500 ms = failsafe).
   *
   * @return Age in milliseconds.
   */
  uint32_t lastFrameAgeMs() const;

  /**
   * @brief Expose the HAL UART handle for use in the application Rx callback.
   *
   * @return Pointer to the HAL UART handle.
   */
  UART_HandleTypeDef* uartHandle() const;

private:
  UART_HandleTypeDef* m_huart; ///< HAL UART handle (UART4).

  // DMA staging buffer (written by DMA hardware, read in onDmaRxEvent)
  uint8_t m_dmaRxBuf[kDmaRxBufLen];

  // Parser state (m_rxBuf/m_rxIdx shared between onDmaRxEvent ISR and update main)
  volatile uint8_t m_rxBuf[kMaxFrameLen]; ///< Frame accumulator
  volatile uint8_t m_rxIdx;               ///< Current write position in m_rxBuf

  // Decoded channel data
  uint16_t     m_channels[kNumChannels]; ///< Last decoded channel values (CRSF units)
  volatile bool m_newData;               ///< Set when a fresh frame is decoded
  uint32_t      m_lastFrameTick;         ///< HAL_GetTick() at last valid frame

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
};

/* EOF -----------------------------------------------------------------------*/
