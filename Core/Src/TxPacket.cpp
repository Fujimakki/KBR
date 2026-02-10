#include "TxPacket.hpp"

#include <string.h>
#include "fft_mag.h"

TxPacket::TxPacket(uint8_t type)
{
  switch (type)
  {
    case RAW:
    {
      m_pldBytesCount = FFT_SIZE * sizeof(uint16_t);
      break;
    }

    case FFT:
    {
      m_pldBytesCount = FFT_SIZE / 2 * sizeof(float32_t);
      break;
    }

    default:
    {
      m_pldBytesCount = 0;
      break;
    }
  }

  m_pldFloatsCount = m_pldBytesCount / sizeof(float32_t);

  m_packet = new uint8_t [m_pldBytesCount + 6];
  m_packet[0] = 0xAA;
  m_packet[1] = type;
}

void TxPacket::writeData(const uint16_t* const pldData)
{
  memcpy(m_packet, pldData, m_pldBytesCount);

  uint32_t crc = crcCalc((const uint32_t* const)pldData, m_pldFloatsCount);
  memcpy(m_packet, &crc, sizeof(uint32_t));

  LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_6, (uint32_t)m_packet);
  LL_DMA_SetMemorySize(DMA1, LL_DMA_STREAM_6, m_pldBytesCount + 6);
  LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_6);
}
