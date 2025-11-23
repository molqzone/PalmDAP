#include "xrdap.hpp"

#include "dap_constants.hpp"

namespace DAP
{

uint64_t GenerateSpiTransaction(uint8_t request, uint32_t write_data)
{
  uint64_t frame = 0;

  // XRDAP 48-bit SPI frame format (LSB first):
  // bits 0-1: PADDING (always 0)
  // bits 2-9: SWD request byte
  // bit 10: TURN1 (bus turnaround)
  // bits 11-13: ACK window (target drives)
  // bit 14: TURN2/data boundary (WRITE: host can drive, READ: target drives)
  // bits 15-46: DATA phase (32 bits + parity)
  // bit 47+: IDLE/padding

  // bits 0-1: PADDING (always 0 for alignment)
  // frame = 0;  // Already 0

  // bits 2-9: SWD request byte
  frame |= (static_cast<uint64_t>(request) & 0xFF) << 2;

  bool is_read = (request & DAP_TRANSFER_RnW);

  if (!is_read)
  {
    // WRITE operation: include data in frame
    // bits 15-46: 32-bit write data (LSB first)
    frame |= (static_cast<uint64_t>(write_data) << 15);

    // bit 46: Data parity bit
    uint8_t data_parity = 0;
    uint32_t temp_data = write_data;
    for (int i = 0; i < 32; i++)
    {
      data_parity ^= (temp_data & 1);
      temp_data >>= 1;
    }
    frame |= (static_cast<uint64_t>(data_parity) << 46);
  }
  // READ operation: no data to place, hardware will capture response
  // bits 15-47 remain 0 for read operations, target will drive data+parity

  return frame;
}

}  // namespace DAP