#include <RxPacket.hpp>
#include <string.h>


RxPacket::ParsedData RxPacket::parseData(uint8_t* pckt)
{
  ParsedData data;
  memcpy(&(data.type), pckt + 1, 1);

  memcpy(&(data.value), pckt + m_HEADER_BYTES_COUNT, m_AWS_PAYLOAD_BYTES_COUNT);

  uint32_t crc;
  memcpy(&crc, pckt + m_HEADER_BYTES_COUNT + m_AWS_PAYLOAD_BYTES_COUNT, m_CRC_BYTES_COUNT);

  uint32_t currentCrc = data.value;
  currentCrc = crcCalc(&currentCrc, 1);
  if (currentCrc != crc)
  {
    data.is_parsed = false;
  }
  else
  {
    data.is_parsed = true;
  }

  return data;
}
