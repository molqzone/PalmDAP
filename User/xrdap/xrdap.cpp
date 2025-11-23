#include "xrdap.hpp"

#include "dap_constants.hpp"

namespace DAP
{

uint64_t GenerateSpiTransaction(uint8_t request, uint32_t write_data)
{
  uint64_t frame = 0;

  // SWD request byte (bit 0 is start, always 0; bit 1 is APnDP; bit 2 is RnW; bits 3-4
  // are A3:A2; bit 5 is parity; bit 6 is stop; bit 7 is park)
  frame = static_cast<uint64_t>(request) & 0xFF;  // Bits 0-7: SWD request byte

  // Write data for SWD write operations (bits 8-39)
  bool is_read = (request & DAP_TRANSFER_RnW);
  if (!is_read)
  {
    frame |= (static_cast<uint64_t>(write_data) << 8);  // Bits 8-39: 32-bit write data
  }

  // Parity for write data (bit 40)
  if (!is_read)
  {
    uint8_t data_parity = 0;
    uint32_t temp_data = write_data;
    for (int i = 0; i < 32; i++)
    {
      data_parity ^= (temp_data & 1);
      temp_data >>= 1;
    }
    frame |= (static_cast<uint64_t>(data_parity) << 40);  // Bit 40: data parity
  }

  // Read operation: no data to place, hardware will capture response
  // Bits 15-47 remain 0 for read operations

  return frame;
}

}  // namespace DAP