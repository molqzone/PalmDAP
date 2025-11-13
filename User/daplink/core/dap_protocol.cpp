#include "dap_protocol.hpp"

#include <cstring>
#include <functional>

#include "dap_config.hpp"
#include "dap_io.hpp"
namespace DAP
{

DapProtocol::DapProtocol(DapIo& io) : io_(io), spi_sem_(0) { Setup(); }

void DapProtocol::Setup()
{
  state_ = {};
  state_.debug_port = DapPort::DISABLED;
}

void DapProtocol::Reset() { Setup(); }

uint32_t DapProtocol::ExecuteCommand(
    const uint8_t* request, LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  DapProtocol::CommandResult r = ProcessCommand(request, response_callback);
  return r.response_generated;
}

DapProtocol::CommandResult DapProtocol::ProcessCommand(
    const uint8_t* request, LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  const auto command = static_cast<CommandId>(request[0]);  ///< Command ID
  const uint8_t* payload = request + 1;  ///< Pointer to request payload area

  CommandResult result;  ///< Command processing result

  switch (command)
  {
    case CommandId::Info:
      result = HandleInfo(payload, response_callback);
      break;
    case CommandId::LED:
      result = HandleHostStatus(payload, response_callback);
      break;
    case CommandId::Connect:
      result = HandleConnect(payload, response_callback);
      break;
    case CommandId::Disconnect:
      result = HandleDisconnect(response_callback);
      break;

    // Essential SWD commands for OpenOCD
    case CommandId::SWJ_Pins:
      result = HandleSwjPins(payload, response_callback);
      break;
    case CommandId::SWJ_Clock:
      result = HandleSwjClock(payload, response_callback);
      break;
    case CommandId::SWJ_Sequence:
      result = HandleSwjSequence(payload, response_callback);
      break;
    case CommandId::SWD_Configure:
      result = HandleSwdConfigure(payload, response_callback);
      break;
    case CommandId::SWD_Sequence:
      result = HandleSwdSequence(payload, response_callback);
      break;
    case CommandId::TransferConfigure:
      result = HandleTransferConfigure(payload, response_callback);
      break;
    case CommandId::Transfer:
      result = HandleTransfer(payload, response_callback);
      break;
    case CommandId::TransferBlock:
      result = HandleTransferBlock(payload, response_callback);
      break;
    case CommandId::ResetTarget:
      result = HandleResetTarget(response_callback);
      break;

    default:
      // Send Invalid command response
      static uint8_t invalid_response[] = {static_cast<uint8_t>(CommandId::Invalid)};
      response_callback.Run(true, invalid_response, sizeof(invalid_response));
      result.response_generated = 1;  // Only the Invalid command ID
      result.request_consumed = 1;    // Only consume command byte
      break;
  }

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

DapProtocol::CommandResult DapProtocol::HandleInfo(
    const uint8_t* req, LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  const auto info_id = static_cast<InfoId>(*req);

  // Create response buffer: command byte + length byte + data
  static uint8_t response[64];
  response[0] = static_cast<uint8_t>(CommandId::Info);

  uint8_t* data_ptr = response + 2;  // Leave space for command and length bytes
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
      capabilities |= (1U << 1);  // JTAG support
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

  response[1] = data_length;  // Write the actual length

  // Send response via callback
  response_callback.Run(true, response, 2 + data_length);

  return {
      1,
      static_cast<uint16_t>(
          2 + data_length)};  // Consumed 1 payload byte (info_id), produced 2 + n bytes
}

DapProtocol::CommandResult DapProtocol::HandleConnect(
    const uint8_t* req, LibXR::Callback<const uint8_t*, size_t> response_callback)
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

  // Create response: command byte + status byte
  static uint8_t response[2];
  response[0] = static_cast<uint8_t>(CommandId::Connect);

  // Set response based on success (CMSIS-DAP V1 spec: 1 byte response)
  if (success == LibXR::ErrorCode::OK)
  {
    state_.debug_port = static_cast<DapPort>(selected_port);
    response[1] = static_cast<uint8_t>(selected_port);
  }
  else
  {
    PortOff();
    state_.debug_port = DapPort::DISABLED;
    response[1] = static_cast<uint8_t>(Port::Disabled);  // 0x00 indicates failure
  }

  // Send response via callback
  response_callback.Run(true, response, 2);

  return {1, 2};  // Consumed 1 payload byte (port), produced 2 response bytes
}

DapProtocol::CommandResult DapProtocol::HandleDisconnect(
    LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  state_.debug_port = DapPort::DISABLED;
  PortOff();

  // Create response: command byte + status byte
  static uint8_t response[2];
  response[0] = static_cast<uint8_t>(CommandId::Disconnect);
  response[1] = static_cast<uint8_t>(Status::OK);

  // Send response via callback
  response_callback.Run(true, response, 2);

  return {0, 2};  // Consumed 0 payload bytes, produced 2 response bytes
}

LibXR::ErrorCode DapProtocol::SetupSwd()
{
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

  // Execute the JTAG-to-SWD Switching Sequence as single pack

  // Send > 50 SWCLK cycles with SWDIO (TMS) high to reset JTAG state machine.
  io_.gpio_swdio.Write(true);  // SWDIO high

  // Create single pack: [64 high bits] + [JTAG-to-SWD sequence 0xE79E] + [64 high bits]
  static const uint8_t swd_sequence_pack[] = {
      // 64 high bits to reset JTAG state machine (8 bytes * 8 = 64 bits)
      0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
      // 16-bit JTAG-to-SWD sequence (0xE79E), MSB first
      0xE7, 0x9E,
      // 64 high bits to finalize SWD mode
      0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  // Use callback style for SPI write operation
  auto spi_callback = LibXR::Callback<LibXR::ErrorCode>::Create(
      [](bool in_isr, int context, LibXR::ErrorCode ec)
      {
        // SPI write completion callback - no action needed for now
        UNUSED(in_isr);
        UNUSED(context);
        UNUSED(ec);
      },
      0);  // context value not used

  LibXR::WriteOperation spi_op(spi_callback);
  err = io_.spi.Write({swd_sequence_pack, sizeof(swd_sequence_pack)}, spi_op);
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
  io_.gpio_swdio.Write(true);  // TMS high

  // Use single byte for JTAG reset - 8 bits of 0xFF = 8 TCK cycles
  static const uint8_t jtag_reset_pack[] = {0xFF};

  // Use callback style for SPI write operation
  auto spi_callback = LibXR::Callback<LibXR::ErrorCode>::Create(
      [](bool in_isr, int context, LibXR::ErrorCode ec)
      {
        // SPI write completion callback - no action needed for now
        UNUSED(in_isr);
        UNUSED(context);
        UNUSED(ec);
      },
      0);  // context value not used

  LibXR::WriteOperation spi_op(spi_callback);
  err = io_.spi.Write({jtag_reset_pack, sizeof(jtag_reset_pack)}, spi_op);
  if (err != LibXR::ErrorCode::OK)
  {
    return err;
  }

  return LibXR::ErrorCode::OK;
}

void DapProtocol::PortOff()
{
  // Configure all relevant GPIOs as high-impedance inputs.
  // This prevents the debug probe from driving any lines when disconnected.
  io_.gpio_swdio.SetConfig({LibXR::GPIO::Direction::INPUT, LibXR::GPIO::Pull::NONE});
  io_.gpio_tdo.SetConfig({LibXR::GPIO::Direction::INPUT, LibXR::GPIO::Pull::NONE});
  io_.gpio_nreset.SetConfig({LibXR::GPIO::Direction::INPUT, LibXR::GPIO::Pull::UP});
}

// Basic SWD command implementations for OpenOCD compatibility

DapProtocol::CommandResult DapProtocol::HandleSwjPins(
    const uint8_t* req, LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  // Basic implementation - just return success
  // TODO: Implement actual pin control if needed
  static uint8_t response[2];
  response[0] = static_cast<uint8_t>(CommandId::SWJ_Pins);
  response[1] = 0x00;  // Status: OK

  response_callback.Run(true, response, 2);
  return {1, 2};  // Consume 1 byte, produce 2 byte response
}

DapProtocol::CommandResult DapProtocol::HandleSwjClock(
    const uint8_t* req, LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  // Basic implementation - just return success
  // TODO: Implement actual clock frequency control if needed
  static uint8_t response[2];
  response[0] = static_cast<uint8_t>(CommandId::SWJ_Clock);
  response[1] = 0x00;  // Status: OK

  response_callback.Run(true, response, 2);
  return {1, 2};  // Consume 1 byte, produce 2 byte response
}

DapProtocol::CommandResult DapProtocol::HandleSwjSequence(
    const uint8_t* req, LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  // Basic implementation - just return success
  // TODO: Implement actual SWJ sequence if needed
  static uint8_t response[2];
  response[0] = static_cast<uint8_t>(CommandId::SWJ_Sequence);
  response[1] = 0x00;  // Status: OK

  response_callback.Run(true, response, 2);
  return {1, 2};  // Consume 1 byte, produce 2 byte response
}

DapProtocol::CommandResult DapProtocol::HandleSwdConfigure(
    const uint8_t* req, LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  // Basic implementation - just return success
  // TODO: Implement actual SWD configuration if needed
  static uint8_t response[2];
  response[0] = static_cast<uint8_t>(CommandId::SWD_Configure);
  response[1] = 0x00;  // Status: OK

  response_callback.Run(true, response, 2);
  return {1, 2};  // Consume 1 byte, produce 2 byte response
}

DapProtocol::CommandResult DapProtocol::HandleSwdSequence(
    const uint8_t* req, LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  // Basic implementation - just return success
  // TODO: Implement actual SWD sequence if needed
  static uint8_t response[2];
  response[0] = static_cast<uint8_t>(CommandId::SWD_Sequence);
  response[1] = 0x00;  // Status: OK

  response_callback.Run(true, response, 2);
  return {1, 2};  // Consume 1 byte, produce 2 byte response
}

DapProtocol::CommandResult DapProtocol::HandleTransferConfigure(
    const uint8_t* req, LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  // Basic implementation - just return success
  // TODO: Implement actual transfer configuration if needed
  static uint8_t response[2];
  response[0] = static_cast<uint8_t>(CommandId::TransferConfigure);
  response[1] = 0x00;  // Status: OK

  response_callback.Run(true, response, 2);
  return {1, 2};  // Consume 1 byte, produce 2 byte response
}

DapProtocol::CommandResult DapProtocol::HandleTransfer(
    const uint8_t* req, LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  // Basic implementation - return error for now
  // This is a critical command that needs full implementation for actual debugging
  static uint8_t response[3];
  response[0] = static_cast<uint8_t>(CommandId::Transfer);
  response[1] = 0xFF;  // Status: Error
  response[2] = 0x00;  // No data transferred

  response_callback.Run(true, response, 3);
  return {5, 3};  // Consume 5 bytes (standard for Transfer), produce 3 bytes
}

DapProtocol::CommandResult DapProtocol::HandleTransferBlock(
    const uint8_t* req, LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  // Basic implementation - return error for now
  // This is a critical command that needs full implementation for actual debugging
  static uint8_t response[3];
  response[0] = static_cast<uint8_t>(CommandId::TransferBlock);
  response[1] = 0xFF;  // Status: Error
  response[2] = 0x00;  // No data transferred

  response_callback.Run(true, response, 3);
  return {5, 3};  // Consume 5 bytes (standard for TransferBlock), produce 3 bytes
}

DapProtocol::CommandResult DapProtocol::HandleResetTarget(
    LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  // Basic implementation - just return success
  // TODO: Implement actual target reset if needed
  static uint8_t response[2];
  response[0] = static_cast<uint8_t>(CommandId::ResetTarget);
  response[1] = 0x00;  // Status: OK

  response_callback.Run(true, response, 2);
  return {0, 2};  // Consume 0 bytes, produce 2 byte response
}

DapProtocol::CommandResult DapProtocol::HandleHostStatus(
    const uint8_t* req, LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  // DAP_LED / DAP_HostStatus command handler
  // Command format: [0x01] [LED_number] [LED_state] for LED control
  //                 [0x01] [status] [param]   for HostStatus (V2)

  static uint8_t response[2];
  response[0] = static_cast<uint8_t>(CommandId::LED);
  response[1] = 0x00;  // Status: OK (DAP_OK)

  // For LED control, we'll accept the command but don't actually control LEDs
  // This prevents OpenOCD from showing "not implemented" errors
  response_callback.Run(true, response, 2);
  return {2, 2};  // Consume 2 bytes (LED_number + LED_state), produce 2 bytes
}

}  // namespace DAP
