/*******************************************************************************
 * @file CRSFReceiver.cpp
 * @brief CRSF frame parser implementation.
 *
 * @author Integrated Motor Controller firmware team
 ******************************************************************************/

#include "CRSFReceiver.hpp"

CRSFReceiver::CRSFReceiver(UART_HandleTypeDef* huart)
  : m_huart(huart)
  , m_rxIdx(0U)
  , m_newData(false)
  , m_lastFrameTick(0U)
{
  for (uint8_t i = 0U; i < kNumChannels; ++i)
  {
    m_channels[i] = (kCrsfMin + kCrsfMax) / 2U; // centre
  }
}

bool CRSFReceiver::init()
{
  // Arm DMA receive with IDLE-line detection. HAL_UARTEx_RxEventCallback fires
  // on IDLE or buffer-full; the caller must route that to onDmaRxEvent().
  return (HAL_UARTEx_ReceiveToIdle_DMA(m_huart, m_dmaRxBuf, kDmaRxBufLen) == HAL_OK);
}

void CRSFReceiver::onDmaRxEvent(uint16_t size)
{
  // Called from HAL_UARTEx_RxEventCallback (ISR context).
  // Copy bytes from DMA staging buffer into the frame accumulator.
  for (uint16_t i = 0U; i < size; ++i)
  {
    uint8_t byte = m_dmaRxBuf[i];
    if (m_rxIdx == 0U && byte != kSyncByte)
    {
      continue; // skip until sync
    }
    if (m_rxIdx < kMaxFrameLen)
    {
      m_rxBuf[m_rxIdx++] = byte;
    }
    else
    {
      m_rxIdx = 0U; // overflow — drop and re-sync
    }
  }
  // Re-arm for the next transfer.
  HAL_UARTEx_ReceiveToIdle_DMA(m_huart, m_dmaRxBuf, kDmaRxBufLen);
}

bool CRSFReceiver::update()
{
  bool decoded = false;

  // We need at least 3 bytes (SYNC + LEN + TYPE) before we can know frame length.
  // Snapshot m_rxIdx once; the ISR may append beyond this point during the loop,
  // which is safe — we only consume bytes we already observed.
  while (true)
  {
    __disable_irq();
    uint8_t fill = m_rxIdx;
    __enable_irq();

    if (fill < 3U)
    {
      break;
    }

    if (m_rxBuf[0] != kSyncByte)
    {
      // Re-sync: shift buffer left by one under a critical section so the
      // ISR cannot append a byte mid-memmove.
      __disable_irq();
      m_rxIdx--;
      memmove(reinterpret_cast<void*>(const_cast<uint8_t*>(m_rxBuf)),
              reinterpret_cast<const void*>(m_rxBuf + 1U),
              m_rxIdx);
      __enable_irq();
      continue;
    }

    // Fix (issue 5): use uint16_t so 2 + frameLen cannot wrap at 254.
    uint8_t  frameLen = m_rxBuf[1]; // bytes from TYPE through CRC inclusive
    uint16_t totalLen = 2U + static_cast<uint16_t>(frameLen); // SYNC + LEN + ...

    if (frameLen < 2U || totalLen > kMaxFrameLen)
    {
      // Invalid length — re-sync.
      __disable_irq();
      m_rxIdx--;
      memmove(reinterpret_cast<void*>(const_cast<uint8_t*>(m_rxBuf)),
              reinterpret_cast<const void*>(m_rxBuf + 1U),
              m_rxIdx);
      __enable_irq();
      continue;
    }

    if (fill < static_cast<uint8_t>(totalLen))
    {
      // Incomplete frame — wait for more bytes.
      break;
    }

    // Full frame available. Reading m_rxBuf[0..totalLen-1] is safe here:
    // the ISR only appends at m_rxIdx ≥ totalLen, so these bytes are stable.
    uint8_t type       = m_rxBuf[2];
    uint8_t crc        = m_rxBuf[totalLen - 1U];
    uint8_t payloadLen = static_cast<uint8_t>(frameLen - 2U); // exclude TYPE and CRC

    if (checkCrc(reinterpret_cast<const uint8_t*>(&m_rxBuf[2]), frameLen - 1U, crc))
    {
      if (type == kTypeRcChannels && payloadLen == 22U)
      {
        decodeRcChannels(reinterpret_cast<const uint8_t*>(&m_rxBuf[3]));
        m_newData       = true;
        m_lastFrameTick = HAL_GetTick();
        decoded         = true;
      }
    }

    // Consume the frame atomically so the ISR cannot append mid-memmove.
    __disable_irq();
    m_rxIdx -= static_cast<uint8_t>(totalLen);
    memmove(reinterpret_cast<void*>(const_cast<uint8_t*>(m_rxBuf)),
            reinterpret_cast<const void*>(m_rxBuf + totalLen),
            m_rxIdx);
    __enable_irq();
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
  uint16_t raw = channel(ch);
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
  uint16_t us    = channelUs(ch);
  float    mid   = (kPwmMin + kPwmMax) / 2.0f;
  float    range = (kPwmMax - kPwmMin) / 2.0f;
  return (static_cast<float>(us) - mid) / range;
}

bool CRSFReceiver::hasNewData()
{
  // Protect the read-clear as an atomic pair. m_newData is written in update()
  // (main context) but callers may be in different task contexts in future.
  __disable_irq();
  bool val    = m_newData;
  m_newData   = false;
  __enable_irq();
  return val;
}

uint32_t CRSFReceiver::lastFrameAgeMs() const
{
  return HAL_GetTick() - m_lastFrameTick;
}

UART_HandleTypeDef* CRSFReceiver::uartHandle() const
{
  return m_huart;
}

/* Private Helpers -----------------------------------------------------------*/

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
        crc <<= 1U;
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
