#include <Packet.hpp>

uint32_t Packet::crcCalc(const uint32_t *const payload, uint16_t pldSize) {
  LL_CRC_ResetCRCCalculationUnit(CRC);

  for (uint16_t i = 0; i < pldSize; i++) {
    uint32_t data = payload[i];
    LL_CRC_FeedData32(CRC, __RBIT(data));
  }

  uint32_t crc = ~(__RBIT(LL_CRC_ReadData32(CRC)));
  return crc;
}
