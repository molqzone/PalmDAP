#include "dap_protocol.hpp"

#include <cstring>
#include <functional>

#include "dap_config.hpp"
#include "dap_io.hpp"
namespace DAP
{

DapProtocol::DapProtocol(DapIo& io) : io_(io) { Setup(); }

void DapProtocol::Setup()
{
  state_ = {};
  state_.debug_port = DapPort::DISABLED;
}

void DapProtocol::Reset() { Setup(); }

uint32_t DapProtocol::ExecuteCommand(const uint8_t* request, uint8_t* response)
{
  DapProtocol::CommandResult r = ProcessCommand(request, response);
  return r.response_generated;
}

DapProtocol::CommandResult DapProtocol::ProcessCommand(const uint8_t* request,
                                                       uint8_t* response)
{
  const auto command = static_cast<CommandId>(request[0]);  ///< Command ID

  // Debug: Mark that we received a command
  response[0] = request[0];                                 // Echo ID

  // Set debug pattern in bytes 62-63 to verify this function is called
  if (response[0] == 0x00 || response[0] == 0x02) {  // DAP_Info or DAP_Connect
    // We'll set this later after processing
  }

  // We consumed the command byte.
  const uint8_t* payload = request + 1;      ///< Pointer to request payload area
  uint8_t* response_payload = response + 1;  ///< Pointer to response payload area

  CommandResult result;  ///< Command processing result

  switch (command)
  {
    case CommandId::Info:
      result = HandleInfo(payload, response_payload);
      break;
    case CommandId::Connect:
      result = HandleConnect(payload, response_payload);
      break;
    case CommandId::Disconnect:
      result = HandleDisconnect(response_payload);
      break;

    // Essential SWD commands for OpenOCD
    case CommandId::SWJ_Pins:
      result = HandleSwjPins(payload, response_payload);
      break;
    case CommandId::SWJ_Clock:
      result = HandleSwjClock(payload, response_payload);
      break;
    case CommandId::SWJ_Sequence:
      result = HandleSwjSequence(payload, response_payload);
      break;
    case CommandId::SWD_Configure:
      result = HandleSwdConfigure(payload, response_payload);
      break;
    case CommandId::SWD_Sequence:
      result = HandleSwdSequence(payload, response_payload);
      break;
    case CommandId::TransferConfigure:
      result = HandleTransferConfigure(payload, response_payload);
      break;
    case CommandId::Transfer:
      result = HandleTransfer(payload, response_payload);
      break;
    case CommandId::TransferBlock:
      result = HandleTransferBlock(payload, response_payload);
      break;
    case CommandId::ResetTarget:
      result = HandleResetTarget(response_payload);
      break;

    default:
      // Overwrite echoed command ID with Invalid for unsupported commands
      response[0] = static_cast<uint8_t>(CommandId::Invalid);
      result.response_generated = 1;  // Only the Invalid command ID
      result.request_consumed = 1;    // Only consume command byte
      break;
  }

  // Total consumed = command byte + payload consumed
  result.request_consumed += 1;
  return result;
}

/**
 * @brief Copy a null-terminated string into a byte array.
 *
 * Copies the string pointed to by `str` into the byte array pointed to by `data_ptr`.
 * If `str` is a null pointer, returns 0.
 *
 * @param str The null-terminated string to copy.
 * @param data_ptr The byte array to copy the string into.
 * @return The number of bytes copied into `data_ptr`.
 */
static uint8_t HandleStringInfo(const char* str, uint8_t* data_ptr)
{
  if (!str) return 0;
  uint8_t len = strlen(str);
  std::memcpy(data_ptr, str, len);
  return len;
}

DapProtocol::CommandResult DapProtocol::HandleInfo(const uint8_t* req, uint8_t* res)
{
  const auto info_id = static_cast<InfoId>(*req);
  uint8_t* data_ptr = res + 1;  // Leave space for the length byte
  uint8_t data_length = 0;

  switch (info_id)
  {
    // Strings type info
    case InfoId::Vendor:
      data_length = HandleStringInfo(DAP::VENDOR_STRING, data_ptr);
      break;
    case InfoId::Product:
      data_length = HandleStringInfo(DAP::PRODUCT_STRING, data_ptr);
      break;
    case InfoId::SerialNumber:
      data_length = HandleStringInfo(DAP::SERIAL_NUMBER_STRING, data_ptr);
      break;
    case InfoId::FirmwareVersion:
      data_length = HandleStringInfo(DAP::FIRMWARE_VERSION_STRING, data_ptr);
      break;

    // Aliases
    case InfoId::DeviceVendor:
      data_length = HandleStringInfo(DAP::VENDOR_STRING, data_ptr);
      break;
    case InfoId::DeviceName:
      data_length = HandleStringInfo(DAP::PRODUCT_STRING, data_ptr);
      break;
    case InfoId::BoardVendor:
      data_length = HandleStringInfo(DAP::VENDOR_STRING, data_ptr);
      break;
    case InfoId::BoardName:
      data_length = HandleStringInfo(DAP::PRODUCT_STRING, data_ptr);
      break;
    case InfoId::ProductFirmwareVersion:
      data_length = HandleStringInfo(DAP::FIRMWARE_VERSION_STRING, data_ptr);
      break;

    // Special cases
    case InfoId::Capabilities:
    {
      uint8_t capabilities = (1U << 4);
      capabilities |= (1U << 0);  // SWD support
      // capabilities |= (1U << 1);   // JTAG support - NOT IMPLEMENTED
      // TODO - No JTAG support for OpenOCD detection yet
      data_ptr[0] = capabilities;
      data_length = 1;
      break;
    }
    case InfoId::PacketSize:
    {
      constexpr uint16_t max_packet_size = 64;  // TODO: Support user configuration
      data_ptr[0] = static_cast<uint8_t>(max_packet_size & 0xFF);
      data_ptr[1] = static_cast<uint8_t>((max_packet_size >> 8) & 0xFF);
      data_length = 2;

      break;
    }
    case InfoId::PacketCount:
    {
      constexpr uint16_t packet_count = 1;  // NOTE: Usually 1 for USB HID
      data_ptr[0] = static_cast<uint8_t>(packet_count & 0xFF);
      data_length = 1;

      break;
    }
    default:
      // If ID is unknown, length is 0.
      data_length = 0;
      break;
  }

  res[0] = data_length;  // Write the actual length

  return {1, static_cast<uint16_t>(
                 1 + data_length)};  // Consumed 1 payload byte (info_id), produced 1
                                     // (length) + n (data) bytes.
}

DapProtocol::CommandResult DapProtocol::HandleConnect(const uint8_t* req, uint8_t* res)
{
  const auto port = static_cast<Port>(req[0]);
  LibXR::ErrorCode success = LibXR::ErrorCode::FAILED;
  Port selected_port = port;

  // Handle port selection with autodetect (like DAPLink)
  if (port == Port::AutoDetect || port == Port::Disabled)
  {
    // Autodetect: default to SWD since that's what we support
    selected_port = Port::SWD;
    success = SetupSwd();
  }
  else if (port == Port::SWD)
  {
    success = SetupSwd();
  }
  else if (port == Port::JTAG)
  {
    success = SetupJtag();
  }

  // Set response based on success (CMSIS-DAP V1 spec: 1 byte response)
  if (success == LibXR::ErrorCode::OK)
  {
    state_.debug_port = static_cast<DapPort>(selected_port);
    res[0] = static_cast<uint8_t>(selected_port);
  }
  else
  {
    PortOff();
    state_.debug_port = DapPort::DISABLED;
    res[0] = static_cast<uint8_t>(Port::Disabled);  // 0x00 indicates failure
  }

  return {1, 1};  // Consumed 1 payload byte (port), produced 1 response byte
}

DapProtocol::CommandResult DapProtocol::HandleDisconnect(uint8_t* res)
{
  state_.debug_port = DapPort::DISABLED;
  PortOff();

  res[0] = static_cast<uint8_t>(Status::OK);
  return {0, 1};  // Consumed 0 payload bytes, produced 1 response byte
}

LibXR::ErrorCode DapProtocol::SetupSwd()
{
  LibXR::Semaphore sem;
  LibXR::WriteOperation op_block(sem, 100);  // 100 ms timeout
  LibXR::ErrorCode err;

  // Configure SPI to generate SWCLK (SPI Mode 0 is typical for SWD)
  // NOTE - The actual clock frequency should be set via HandleSwjClock.
  err =
      io_.spi.SetConfig({LibXR::SPI::ClockPolarity::LOW, LibXR::SPI::ClockPhase::EDGE_1});
  if (err != LibXR::ErrorCode::OK)
  {
    return err;
  }

  // Configure GPIOs for SWDIO and nRESET as needed
  // nRESET is open-drain, kept high (de-asserted)
  err = io_.gpio_nreset.SetConfig({
      LibXR::GPIO::Direction::OUTPUT_OPEN_DRAIN,
      LibXR::GPIO::Pull::NONE  // NOTE - Assuming external pull-up
  });
  if (err != LibXR::ErrorCode::OK)
  {
    return err;
  }
  io_.gpio_nreset.Write(true);  // Deassert nRESET

  // SWDIO is used for both input and output. We'll manage its direction.
  // Start with it as an output for the switching sequence.
  err = io_.gpio_swdio.SetConfig({
      LibXR::GPIO::Direction::OUTPUT_PUSH_PULL,
      LibXR::GPIO::Pull::NONE  // NOTE - Assuming external pull-up
  });
  if (err != LibXR::ErrorCode::OK)
  {
    return err;
  }

  // Execute the JTAG-to-SWD Switching Sequence

  // Send > 50 SWCLK cycles with SWDIO (TMS) high to reset JTAG state machine.
  io_.gpio_swdio.Write(true);  // SWDIO high
  // We send 8 bytes of 0xFF. MOSI (connected to SWDIO) is held high,
  // and SCK generates 64 clock cycles.
  const uint8_t high_bits[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  err = io_.spi.Write({high_bits, sizeof(high_bits)}, op_block);
  if (err != LibXR::ErrorCode::OK)
  {
    return err;
  }

  // Send the 16-bit JTAG-to-SWD sequence (0xE79E), MSB first.
  const uint8_t swd_seq[2] = {0xE7, 0x9E};
  err = io_.spi.Write({swd_seq, sizeof(swd_seq)}, op_block);
  if (err != LibXR::ErrorCode::OK)
  {
    return err;
  }

  // Finalize with > 50 SWCLK cycles with SWDIO (TMS) high.
  err = io_.spi.Write({high_bits, sizeof(high_bits)}, op_block);
  if (err != LibXR::ErrorCode::OK)
  {
    return err;
  }

  // Now the target is in SWD mode. The port is ready.
  // We can leave SWDIO as output, it will be switched to input as needed
  // during SWD transfers.

  return LibXR::ErrorCode::OK;
}

LibXR::ErrorCode DapProtocol::SetupJtag()
{
  LibXR::Semaphore sem;
  LibXR::WriteOperation op_block(sem, 100);  // 100 ms timeout
  LibXR::ErrorCode err;

  // Configure SPI for JTAG clocking (Mode 0 is common)
  err =
      io_.spi.SetConfig({LibXR::SPI::ClockPolarity::LOW, LibXR::SPI::ClockPhase::EDGE_1});
  if (err != LibXR::ErrorCode::OK)
  {
    return err;
  }

  // Configure GPIOs for JTAG
  err = io_.gpio_nreset.SetConfig({
      LibXR::GPIO::Direction::OUTPUT_OPEN_DRAIN,
      LibXR::GPIO::Pull::NONE  // NOTE - Assuming external pull-up
  });
  if (err != LibXR::ErrorCode::OK)
  {
    return err;
  }
  io_.gpio_nreset.Write(true);  // Deassert nRESET

  // TDO is a dedicated input
  err = io_.gpio_tdo.SetConfig({
      LibXR::GPIO::Direction::INPUT,
      LibXR::GPIO::Pull::UP  // NOTE - often pulled up
  });
  if (err != LibXR::ErrorCode::OK)
  {
    return err;
  }

  // TODO - TMS is now shared with SWDIO, make it configurable in future
  // TMS is controlled by the SWDIO pin in our shared-pin setup
  err = io_.gpio_swdio.SetConfig(
      {LibXR::GPIO::Direction::OUTPUT_PUSH_PULL, LibXR::GPIO::Pull::NONE});
  if (err != LibXR::ErrorCode::OK)
  {
    return err;
  }

  // Reset the JTAG TAP controller to Test-Logic-Reset state
  // by sending at least 5 TCK cycles with TMS high.
  io_.gpio_swdio.Write(true);                 // TMS high
  const uint8_t high_bits_reset[1] = {0xFF};  // 8 bits of 0xFF = 8 TCK cycles
  err = io_.spi.Write({high_bits_reset, sizeof(high_bits_reset)}, op_block);
  if (err != LibXR::ErrorCode::OK)
  {
    return err;
  }

  return LibXR::ErrorCode::OK;
}

void DapProtocol::PortOff()
{
  LibXR::Semaphore sem;
  LibXR::WriteOperation op_block(sem, 100);  // 100 ms timeout

  // Configure all relevant GPIOs as high-impedance inputs.
  // This prevents the debug probe from driving any lines when disconnected.
  io_.gpio_swdio.SetConfig({LibXR::GPIO::Direction::INPUT, LibXR::GPIO::Pull::NONE});
  io_.gpio_tdo.SetConfig({LibXR::GPIO::Direction::INPUT, LibXR::GPIO::Pull::NONE});
  io_.gpio_nreset.SetConfig({LibXR::GPIO::Direction::INPUT, LibXR::GPIO::Pull::UP});
}

// Basic SWD command implementations for OpenOCD compatibility

DapProtocol::CommandResult DapProtocol::HandleSwjPins(const uint8_t* req, uint8_t* res)
{
  // Basic implementation - just return success
  // TODO: Implement actual pin control if needed
  res[0] = 0x00;  // Status: OK
  return {1, 1};  // Consume 1 byte, produce 1 byte response
}

DapProtocol::CommandResult DapProtocol::HandleSwjClock(const uint8_t* req, uint8_t* res)
{
  // Basic implementation - just return success
  // TODO: Implement actual clock frequency control if needed
  res[0] = 0x00;  // Status: OK
  return {1, 1};  // Consume 1 byte, produce 1 byte response
}

DapProtocol::CommandResult DapProtocol::HandleSwjSequence(const uint8_t* req, uint8_t* res)
{
  // Basic implementation - just return success
  // TODO: Implement actual SWJ sequence if needed
  res[0] = 0x00;  // Status: OK
  return {1, 1};  // Consume 1 byte, produce 1 byte response
}

DapProtocol::CommandResult DapProtocol::HandleSwdConfigure(const uint8_t* req, uint8_t* res)
{
  // Basic implementation - just return success
  // TODO: Implement actual SWD configuration if needed
  res[0] = 0x00;  // Status: OK
  return {1, 1};  // Consume 1 byte, produce 1 byte response
}

DapProtocol::CommandResult DapProtocol::HandleSwdSequence(const uint8_t* req, uint8_t* res)
{
  // Basic implementation - just return success
  // TODO: Implement actual SWD sequence if needed
  res[0] = 0x00;  // Status: OK
  return {1, 1};  // Consume 1 byte, produce 1 byte response
}

DapProtocol::CommandResult DapProtocol::HandleTransferConfigure(const uint8_t* req, uint8_t* res)
{
  // Basic implementation - just return success
  // TODO: Implement actual transfer configuration if needed
  res[0] = 0x00;  // Status: OK
  return {1, 1};  // Consume 1 byte, produce 1 byte response
}

DapProtocol::CommandResult DapProtocol::HandleTransfer(const uint8_t* req, uint8_t* res)
{
  // Basic implementation - return error for now
  // This is a critical command that needs full implementation for actual debugging
  res[0] = 0xFF;  // Status: Error
  res[1] = 0x00;  // No data transferred
  return {5, 2};  // Consume 5 bytes (standard for Transfer), produce 2 bytes
}

DapProtocol::CommandResult DapProtocol::HandleTransferBlock(const uint8_t* req, uint8_t* res)
{
  // Basic implementation - return error for now
  // This is a critical command that needs full implementation for actual debugging
  res[0] = 0xFF;  // Status: Error
  res[1] = 0x00;  // No data transferred
  return {5, 2};  // Consume 5 bytes (standard for TransferBlock), produce 2 bytes
}

DapProtocol::CommandResult DapProtocol::HandleResetTarget(uint8_t* res)
{
  // Basic implementation - just return success
  // TODO: Implement actual target reset if needed
  res[0] = 0x00;  // Status: OK
  return {0, 1};  // Consume 0 bytes, produce 1 byte response
}

}  // namespace DAP
