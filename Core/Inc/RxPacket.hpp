#ifndef INC_RXPACKET_HPP_
#define INC_RXPACKET_HPP_

#include <Packet.hpp>

class RxPacket : protected virtual Packet
{
public:

  enum RxPacketType : uint8_t
  {
    AWS = 0x31, // New AVRG_WINDOW_SIZE value
  };

  struct ParsedData
  {
    uint16_t value;
    RxPacketType type;
    bool is_parsed;
  };

  RxPacket() = delete;
  ~RxPacket() = delete;

  static ParsedData parseData(uint8_t* pckt);

private:

  static constexpr uint8_t m_AWS_PAYLOAD_BYTES_COUNT = 2;

};

#endif /* INC_RXPACKET_HPP_ */
