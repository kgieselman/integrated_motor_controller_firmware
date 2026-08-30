/*******************************************************************************
 * @file CRSFReceiver.cpp
 * @brief CRSF frame parser implementation.
 *
 * See the receive-model note at the top of CRSFReceiver.hpp — the split between
 * the ISR (records the DMA head, nothing else) and update() (drains and parses,
 * in task context) is load-bearing, not incidental.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#include "CRSFReceiver.hpp"

CRSFReceiver::CRSFReceiver(UART_HandleTypeDef* huart)
  : m_huart(huart)
  , m_dmaRxBuf{}
  , m_dmaHead(0U)
  , m_dmaTail(0U)
  , m_rxBuf{}
  , m_rxIdx(0U)
  , m_channels{}
  , m_newData(false)
  , m_lastFrameTick(0U)
  , m_droppedBytes(0U)
{
  for (uint8_t i = 0U; i < kNumChannels; ++i)
  {
    m_channels[i] = (kCrsfMin + kCrsfMax) / 2U; // centre
  }
}

bool CRSFReceiver::init()
{
  // Circular DMA with IDLE-line detection. This runs for the life of the
  // program — it is deliberately never re-armed. See the header note.
  return (HAL_UARTEx_ReceiveToIdle_DMA(m_huart,
                                       const_cast<uint8_t*>(m_dmaRxBuf),
                                       kDmaRxBufLen) == HAL_OK);
}

void CRSFReceiver::onDmaRxEvent(uint16_t size)
{
  // ISR context. In circular mode HAL reports the current write position from
  // the start of the ring, so this is a position, not a count. One aligned
  // 16-bit store; no copying, no parsing, no re-arm.
  if (size <= kDmaRxBufLen)
  {
    m_dmaHead = (size == kDmaRxBufLen) ? 0U : size;
  }
}

bool CRSFReceiver::update()
{
  drainDmaRing();

  bool decoded = false;

  while (m_rxIdx >= 3U) // SYNC + LEN + TYPE minimum before length is known
  {
    if (m_rxBuf[0] != kSyncByte)
    {
      consume(1U); // re-sync one byte at a time
      continue;
    }

    const uint8_t  frameLen = m_rxBuf[1]; // TYPE through CRC inclusive
    const uint16_t totalLen = 2U + static_cast<uint16_t>(frameLen);

    if ((frameLen < 2U) || (totalLen > kMaxFrameLen))
    {
      consume(1U); // implausible length — this was not really a frame start
      continue;
    }

    if (m_rxIdx < totalLen)
    {
      break; // incomplete frame; wait for the rest
    }

    const uint8_t type       = m_rxBuf[2];
    const uint8_t crc        = m_rxBuf[totalLen - 1U];
    const uint8_t payloadLen = static_cast<uint8_t>(frameLen - 2U);

    if (checkCrc(&m_rxBuf[2], static_cast<uint8_t>(frameLen - 1U), crc))
    {
      if ((type == kTypeRcChannels) && (payloadLen == 22U))
      {
        decodeRcChannels(&m_rxBuf[3]);
        m_newData       = true;
        m_lastFrameTick = HAL_GetTick();
        decoded         = true;
      }
      consume(totalLen);
    }
    else
    {
      // Bad CRC means this was probably not a frame boundary at all. Drop one
      // byte and rescan rather than trusting the length field we just read.
      consume(1U);
    }
  }

  return decoded;
}

uint16_t CRSFReceiver::channel(uint8_t ch) const
{
  if (ch >= kNumChannels)
  {
    return kCrsfMin;
  }
  return m_channels[ch];
}

uint16_t CRSFReceiver::channelUs(uint8_t ch) const
{
  const uint16_t raw = channel(ch);
  // Linear map: [kCrsfMin, kCrsfMax] → [kPwmMin, kPwmMax]
  int32_t val = kPwmMin
                + ((static_cast<int32_t>(raw - kCrsfMin) * (kPwmMax - kPwmMin))
                   / (kCrsfMax - kCrsfMin));
  if (val < kPwmMin) { val = kPwmMin; }
  if (val > kPwmMax) { val = kPwmMax; }
  return static_cast<uint16_t>(val);
}

float CRSFReceiver::channelNorm(uint8_t ch) const
{
  const uint16_t us    = channelUs(ch);
  const float    mid   = (kPwmMin + kPwmMax) / 2.0f;
  const float    range = (kPwmMax - kPwmMin) / 2.0f;
  return (static_cast<float>(us) - mid) / range;
}

bool CRSFReceiver::hasNewData()
{
  // Read-and-clear must be atomic. Save and restore PRIMASK rather than calling
  // __enable_irq(): an unconditional enable would silently break any outer
  // critical section this happens to be called from.
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();

  const bool val = m_newData;
  m_newData      = false;

  __set_PRIMASK(primask);
  return val;
}

uint32_t CRSFReceiver::lastFrameAgeMs() const
{
  return HAL_GetTick() - m_lastFrameTick;
}

uint32_t CRSFReceiver::droppedBytes() const
{
  return m_droppedBytes;
}

UART_HandleTypeDef* CRSFReceiver::uartHandle() const
{
  return m_huart;
}

/* Private Helpers -----------------------------------------------------------*/

void CRSFReceiver::drainDmaRing()
{
  // One atomic read of the head; the ISR may advance it again while we copy,
  // and those bytes are simply picked up on the next call.
  const uint16_t head = m_dmaHead;

  while (m_dmaTail != head)
  {
    appendByte(m_dmaRxBuf[m_dmaTail]);

    ++m_dmaTail;
    if (m_dmaTail >= kDmaRxBufLen)
    {
      m_dmaTail = 0U; // wrap
    }
  }
}

void CRSFReceiver::appendByte(uint8_t byte)
{
  if (m_rxIdx == 0U && byte != kSyncByte)
  {
    return; // nothing buffered and this cannot start a frame — ignore it
  }

  if (m_rxIdx >= kMaxFrameLen)
  {
    // Accumulator full without a decodable frame. Drop the oldest byte rather
    // than clearing everything: a complete frame may be sitting at the tail
    // behind the junk, and clearing would throw it away too.
    consume(1U);
    ++m_droppedBytes;
  }

  m_rxBuf[m_rxIdx] = byte;
  ++m_rxIdx;
}

void CRSFReceiver::consume(uint16_t count)
{
  if (count >= m_rxIdx)
  {
    m_rxIdx = 0U;
    return;
  }

  m_rxIdx = static_cast<uint16_t>(m_rxIdx - count);
  std::memmove(m_rxBuf, m_rxBuf + count, m_rxIdx);
}

bool CRSFReceiver::checkCrc(const uint8_t* data, uint8_t len, uint8_t expected) const
{
  // CRC8/DVB-S2 (poly 0xD5)
  uint8_t crc = 0U;
  for (uint8_t i = 0U; i < len; ++i)
  {
    crc ^= data[i];
    for (uint8_t bit = 0U; bit < 8U; ++bit)
    {
      if ((crc & 0x80U) != 0U)
      {
        crc = static_cast<uint8_t>((crc << 1U) ^ 0xD5U);
      }
      else
      {
        crc = static_cast<uint8_t>(crc << 1U);
      }
    }
  }
  return (crc == expected);
}

void CRSFReceiver::decodeRcChannels(const uint8_t* payload)
{
  // 16 channels × 11 bits = 176 bits = 22 bytes, packed LSB-first.
  m_channels[0]  = static_cast<uint16_t>( payload[0]         | ((payload[1]  & 0x07U) << 8U));
  m_channels[1]  = static_cast<uint16_t>((payload[1]  >> 3U) | ((payload[2]  & 0x3FU) << 5U));
  m_channels[2]  = static_cast<uint16_t>((payload[2]  >> 6U) | ( payload[3]           << 2U) | ((payload[4]  & 0x01U) << 10U));
  m_channels[3]  = static_cast<uint16_t>((payload[4]  >> 1U) | ((payload[5]  & 0x0FU) << 7U));
  m_channels[4]  = static_cast<uint16_t>((payload[5]  >> 4U) | ((payload[6]  & 0x7FU) << 4U));
  m_channels[5]  = static_cast<uint16_t>((payload[6]  >> 7U) | ( payload[7]           << 1U) | ((payload[8]  & 0x03U) << 9U));
  m_channels[6]  = static_cast<uint16_t>((payload[8]  >> 2U) | ((payload[9]  & 0x1FU) << 6U));
  m_channels[7]  = static_cast<uint16_t>((payload[9]  >> 5U) | ( payload[10]          << 3U));
  m_channels[8]  = static_cast<uint16_t>( payload[11]        | ((payload[12] & 0x07U) << 8U));
  m_channels[9]  = static_cast<uint16_t>((payload[12] >> 3U) | ((payload[13] & 0x3FU) << 5U));
  m_channels[10] = static_cast<uint16_t>((payload[13] >> 6U) | ( payload[14]          << 2U) | ((payload[15] & 0x01U) << 10U));
  m_channels[11] = static_cast<uint16_t>((payload[15] >> 1U) | ((payload[16] & 0x0FU) << 7U));
  m_channels[12] = static_cast<uint16_t>((payload[16] >> 4U) | ((payload[17] & 0x7FU) << 4U));
  m_channels[13] = static_cast<uint16_t>((payload[17] >> 7U) | ( payload[18]          << 1U) | ((payload[19] & 0x03U) << 9U));
  m_channels[14] = static_cast<uint16_t>((payload[19] >> 2U) | ((payload[20] & 0x1FU) << 6U));
  m_channels[15] = static_cast<uint16_t>((payload[20] >> 5U) | ( payload[21]          << 3U));
}

/* EOF -----------------------------------------------------------------------*/
