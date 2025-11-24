#include "xrdap.hpp"

#include "dap_constants.hpp"

namespace DAP
{

uint64_t GenerateSpiTransaction(uint8_t request, uint32_t write_data)
{
  // XRDAP 48-bit SPI frame format (LSB first):
  // bits 0-1: PADDING (always 0)
  // bits 2-9: SWD request byte
  // bit 10: TURN1 (bus turnaround)
  // bits 11-13: ACK window (target drives - standard SWD 3-bit ACK)
  // bit 14: TURN2/data boundary (XRDAP hardware controls based on ACK)
  // bits 15-46: DATA phase (32 bits + parity) - XRDAP handles SWD protocol compliance
  // bit 47+: IDLE/padding

  // SWD Protocol Compliance (XRDAP hardware behavior):
  // - ACK bits 11-13: 001 (OK), 100 (WAIT), 111 (FAULT), others (ILLEGAL)
  // - When ACK != 001: bit 14+ remains high-impedance, no data driven
  // - After non-OK ACK: XRDAP automatically sends SWCLK with MOSI=0 for recovery
  // - Or pulls reset_n low for raw mode line reset

  // XRDAP SPI frame bit field positions
  constexpr uint8_t PADDING_POS = 0;
  constexpr uint8_t REQUEST_POS = PADDING_POS + 2;
  constexpr uint8_t TURN1_POS = REQUEST_POS + 8;
  constexpr uint8_t ACK_POS = TURN1_POS + 1;
  constexpr uint8_t TURN2_POS = ACK_POS + 3;
  constexpr uint8_t DATA_POS = TURN2_POS + 1;
  // Note: PARITY position depends on READ vs WRITE operation
  // READ: PARITY at bit 46, WRITE: PARITY at bit 47

  // Bit masks
  constexpr uint8_t REQUEST_MASK = 0xFF;      // 8-bit request field
  constexpr uint32_t DATA_MASK = 0xFFFFFFFF;  // 32-bit data field
  constexpr uint8_t PARITY_MASK = 0x01;       // 1-bit parity field

  // Data field constants
  constexpr uint8_t DATA_BITS = 32;  // 32 bits for data field

  uint64_t frame = 0;

  // bits 0-1: PADDING (always 0 for alignment)
  // frame = 0;  // Already 0

  // bits 2-9: SWD request byte (standard SWD format)
  frame |= (static_cast<uint64_t>(request) & REQUEST_MASK) << REQUEST_POS;

  bool is_read = (request & DAP_TRANSFER_RnW);

  // Note: XRDAP hardware handles SWD protocol compliance automatically:
  // - For initial transmission, include data phase
  // - If ACK != 001, XRDAP will:
  //   * Keep bit 14+ in high-impedance (no data driven)
  //   * Send only SWCLK with MOSI=0 for recovery cycles
  //   * Or perform line reset by pulling reset_n low
  //   * This matches standard SWD protocol behavior
  if (!is_read)
  {
    // WRITE operation: include data for initial transmission
    // XRDAP hardware will conditionally drive based on ACK response
    // bits 15-46: 32-bit write data (LSB first)
    frame |= (static_cast<uint64_t>(write_data) & DATA_MASK) << DATA_POS;

    // bit 47: Data parity bit (WRITE parity at bit 47 per XRDAP spec)
    uint8_t data_parity = 0;
    uint32_t temp_data = write_data;
    for (int i = 0; i < DATA_BITS; i++)
    {
      data_parity ^= (temp_data & 1);
      temp_data >>= 1;
    }
    frame |= (static_cast<uint64_t>(data_parity) & 0x01) << 47;  // WRITE parity at bit 47
  }
  // READ operation: no data to place, hardware will capture response
  // bits 15-46: data, bit 46: parity (READ parity at bit 46 per XRDAP spec)

  return frame;
}

bool GenerateDapTransferResponse(uint8_t request, const uint8_t rx_data[6],
                                 uint8_t response[7], size_t& response_size)
{
  // XRDAP SPI response bit positions
  constexpr uint8_t PADDING_BITS = 2;
  constexpr uint8_t REQUEST_BITS = 8;
  constexpr uint8_t TURN1_BIT = 1;
  constexpr uint8_t ACK_BITS = 3;
  constexpr uint8_t TURN2_BIT = 1;
  constexpr uint8_t DATA_BITS = 32;
  constexpr uint8_t PARITY_BIT = 1;

  // XRDAP SPI response bit field positions
  constexpr uint8_t PADDING_POS = 0;
  constexpr uint8_t REQUEST_POS = PADDING_POS + PADDING_BITS;
  constexpr uint8_t TURN1_POS = REQUEST_POS + REQUEST_BITS;
  constexpr uint8_t ACK_POS = TURN1_POS + TURN1_BIT;
  constexpr uint8_t TURN2_POS = ACK_POS + ACK_BITS;
  constexpr uint8_t DATA_POS = TURN2_POS + TURN2_BIT;
  // Note: PARITY position depends on READ vs WRITE operation
  // READ: PARITY at bit 46, WRITE: PARITY at bit 47

  // XRDAP ACK response values
  constexpr uint8_t ACK_OK = 0x01;
  constexpr uint8_t ACK_WAIT = 0x04;
  constexpr uint8_t ACK_FAULT = 0x07;

  // DAP Transfer response sizes
  constexpr size_t RESPONSE_HEADER_SIZE = 3;  // Command + Count + Status
  constexpr size_t RESPONSE_DATA_SIZE = 4;    // 32-bit data for reads
  constexpr size_t RESPONSE_MAX_SIZE = RESPONSE_HEADER_SIZE + RESPONSE_DATA_SIZE;

  // Bit masks
  constexpr uint8_t ACK_MASK = 0x07;          // 3-bit ACK field
  constexpr uint32_t DATA_MASK = 0xFFFFFFFF;  // 32-bit data field

  if (!rx_data || !response)
  {
    return false;
  }

  // Reconstruct 48-bit response from 6-byte little-endian array
  uint64_t rx_frame = 0;
  for (int i = 0; i < 6; i++)
  {
    rx_frame |= (static_cast<uint64_t>(rx_data[i]) << (i * 8));
  }

  // Extract ACK bits (bits 11-13)
  uint8_t ack = static_cast<uint8_t>((rx_frame >> ACK_POS) & ACK_MASK);

  // Check for protocol errors
  if (ack == ACK_WAIT)
  {
    response[0] = static_cast<uint8_t>(CommandId::Transfer);
    response[1] = 1;
    response[2] = DAP_TRANSFER_WAIT;
    response_size = RESPONSE_HEADER_SIZE;
    return true;
  }
  else if (ack == ACK_FAULT)
  {
    response[0] = static_cast<uint8_t>(CommandId::Transfer);
    response[1] = 1;
    response[2] = DAP_TRANSFER_FAULT;
    response_size = RESPONSE_HEADER_SIZE;
    return true;
  }
  else if (ack != ACK_OK)
  {
    response[0] = static_cast<uint8_t>(CommandId::Transfer);
    response[1] = 1;
    response[2] = DAP_TRANSFER_ERROR;
    response_size = RESPONSE_HEADER_SIZE;
    return true;
  }

  // OK response - build full DAP Transfer response
  bool is_read = (request & DAP_TRANSFER_RnW);

  response[0] = static_cast<uint8_t>(CommandId::Transfer);
  response[1] = 1;
  response[2] = DAP_TRANSFER_OK;

  if (is_read)
  {
    // Extract 32-bit read data (bits 15-46)
    uint32_t read_data = static_cast<uint32_t>((rx_frame >> DATA_POS) & DATA_MASK);

    // Extract data parity bit (bit 46 for READ per XRDAP spec)
    uint8_t rx_parity = static_cast<uint8_t>((rx_frame >> 46) & 0x01);

    // Verify parity of received data
    uint8_t calculated_parity = 0;
    uint32_t temp_data = read_data;
    for (int i = 0; i < DATA_BITS; i++)
    {
      calculated_parity ^= (temp_data & 1);
      temp_data >>= 1;
    }

    if (rx_parity != calculated_parity)
    {
      // Parity error
      response[2] = DAP_TRANSFER_ERROR;
      response_size = RESPONSE_HEADER_SIZE;
      return true;
    }

    // Include read data in response (little-endian)
    for (int i = 0; i < RESPONSE_DATA_SIZE; i++)
    {
      response[RESPONSE_HEADER_SIZE + i] =
          static_cast<uint8_t>((read_data >> (i * 8)) & 0xFF);
    }
    response_size = RESPONSE_MAX_SIZE;
  }
  else
  {
    // Write operation - no data to return
    response_size = RESPONSE_HEADER_SIZE;
  }

  return true;
}

}  // namespace DAP