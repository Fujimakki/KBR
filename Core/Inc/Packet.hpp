#ifndef INC_PACKET_HPP_
#define INC_PACKET_HPP_

#include "main.h"
#include <stdint.h>

class Packet
{
public:
  Packet() = default;
  ~Packet() = default;

protected:
  uint32_t m_pldBytesCount;
  uint8_t* m_packet;

  static uint32_t crcCalc(const uint32_t *const payload, uint16_t pldSize);

  static constexpr uint8_t m_HEADER_BYTES_COUNT = 2;
  static constexpr uint8_t m_CRC_BYTES_COUNT = 4;

};



#endif /* INC_PACKET_HPP_ */
