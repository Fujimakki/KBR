#ifndef INC_TXPACKET_HPP_
#define INC_TXPACKET_HPP_

#include <Packet.hpp>

class TxPacket : public virtual Packet
{
  enum TxPacketType : uint8_t
  {
    RAW = 0x51, // Raw data from ADC
    FFT = 0x52  // Calculated FFT magnitudes
  };

public:

  TxPacket() = delete;
  TxPacket(uint8_t type);
  inline ~TxPacket() { delete m_packet; };

  void writeData(const uint16_t* const pldData);

private:
  uint16_t m_pldFloatsCount;

  void writeUart(const uint8_t* const buffer, const uint16_t size);

};

#endif /* INC_TXPACKET_HPP_ */
