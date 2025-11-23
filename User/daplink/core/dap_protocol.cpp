#include "dap_protocol.hpp"

#include <cstring>
#include <functional>

#include "dap_config.hpp"
#include "dap_io.hpp"
namespace DAP
{

DapProtocol::DapProtocol(DapIo& io, TransferMethod transfer_method)
    : io_(io), swd_transfer_method_(transfer_method)
{
  Setup();
}

void DapProtocol::Setup()
{
  state_ = {};
  state_.debug_port = DapPort::DISABLED;
}

void DapProtocol::Reset() { Setup(); }

void DapProtocol::ExecuteCommand(
    const uint8_t* request, LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  ProcessCommand(request, response_callback);
}

void DapProtocol::ProcessCommand(
    const uint8_t* request, LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  const auto command = static_cast<CommandId>(request[0]);  ///< Command ID
  const uint8_t* payload = request + 1;  ///< Pointer to request payload area

  switch (command)
  {
    case CommandId::Info:
      HandleInfo(payload, response_callback);
      break;
    case CommandId::HostStatus:
      HandleHostStatus(payload, response_callback);
      break;
    case CommandId::Connect:
      HandleConnect(payload, response_callback);
      break;
    case CommandId::Disconnect:
      HandleDisconnect(response_callback);
      break;

    // Essential SWD commands for OpenOCD
    case CommandId::SWJ_Pins:
      HandleSwjPins(payload, response_callback);
      break;
    case CommandId::SWJ_Clock:
      HandleSwjClock(payload, response_callback);
      break;
    case CommandId::SWJ_Sequence:
      HandleSwjSequence(payload, response_callback);
      break;
    case CommandId::SWD_Configure:
      HandleSwdConfigure(payload, response_callback);
      break;
    case CommandId::SWD_Sequence:
      HandleSwdSequence(payload, response_callback);
      break;
    case CommandId::TransferConfigure:
      HandleTransferConfigure(payload, response_callback);
      break;
    case CommandId::Transfer:
      HandleTransfer(payload, response_callback);
      break;
    case CommandId::TransferBlock:
      HandleTransferBlock(payload, response_callback);
      break;
    case CommandId::ResetTarget:
      HandleResetTarget(response_callback);
      break;

    default:
      // Send Invalid command response
      static uint8_t invalid_response[] = {static_cast<uint8_t>(CommandId::Invalid)};
      response_callback.Run(true, invalid_response, sizeof(invalid_response));
      break;
  }
}

static uint8_t HandleStringInfo(const char* str, uint8_t* data_ptr)
{
  if (!str) return 0;
  uint8_t len = strlen(str);
  std::memcpy(data_ptr, str, len);
  return len;
}

void DapProtocol::HandleInfo(const uint8_t* req,
                             LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  const auto info_id = static_cast<InfoId>(*req);

  static uint8_t response[64];
  response[0] = static_cast<uint8_t>(CommandId::Info);

  uint8_t* data_ptr = response + 2;
  uint8_t data_length = 0;

  switch (info_id)
  {
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
      constexpr uint16_t max_packet_size = 64;
      data_ptr[0] = static_cast<uint8_t>(max_packet_size & 0xFF);
      data_ptr[1] = static_cast<uint8_t>((max_packet_size >> 8) & 0xFF);
      data_length = 2;
      break;
    }
    case InfoId::PacketCount:
    {
      constexpr uint16_t packet_count = 1;
      data_ptr[0] = static_cast<uint8_t>(packet_count & 0xFF);
      data_length = 1;
      break;
    }
    default:
      data_length = 0;
      break;
  }

  response[1] = data_length;
  response_callback.Run(true, response, 2 + data_length);
}

void DapProtocol::HandleConnect(const uint8_t* req,
                                LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  const auto port = static_cast<Port>(req[0]);
  LibXR::ErrorCode success = LibXR::ErrorCode::FAILED;
  Port selected_port = port;

  if (port == Port::AutoDetect || port == Port::Disabled)
  {
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

  static uint8_t response[2];
  response[0] = static_cast<uint8_t>(CommandId::Connect);

  if (success == LibXR::ErrorCode::OK)
  {
    state_.debug_port = static_cast<DapPort>(selected_port);
    response[1] = static_cast<uint8_t>(selected_port);
  }
  else
  {
    PortOff();
    state_.debug_port = DapPort::DISABLED;
    response[1] = static_cast<uint8_t>(Port::Disabled);
  }

  response_callback.Run(true, response, 2);
}

void DapProtocol::HandleDisconnect(
    LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  state_.debug_port = DapPort::DISABLED;
  PortOff();

  static uint8_t response[2];
  response[0] = static_cast<uint8_t>(CommandId::Disconnect);
  response[1] = static_cast<uint8_t>(Status::OK);

  response_callback.Run(true, response, 2);
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
  err = io_.gpio_swdio.SetConfig(
      {LibXR::GPIO::Direction::OUTPUT_PUSH_PULL, LibXR::GPIO::Pull::NONE});
  if (err != LibXR::ErrorCode::OK)
  {
    return err;
  }

  // Reset the JTAG TAP controller to Test-Logic-Reset state
  io_.gpio_swdio.Write(true);  // TMS high

  static const uint8_t jtag_reset_pack[] = {0xFF};

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
  io_.gpio_swdio.SetConfig({LibXR::GPIO::Direction::INPUT, LibXR::GPIO::Pull::NONE});
  io_.gpio_tdo.SetConfig({LibXR::GPIO::Direction::INPUT, LibXR::GPIO::Pull::NONE});
  io_.gpio_nreset.SetConfig({LibXR::GPIO::Direction::INPUT, LibXR::GPIO::Pull::UP});
}

void DapProtocol::HandleSwjPins(const uint8_t* req,
                                LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  // TODO: Implement actual pin control if needed
  static uint8_t response[2];
  response[0] = static_cast<uint8_t>(CommandId::SWJ_Pins);
  response[1] = 0x00;  // Status: OK

  response_callback.Run(true, response, 2);
}

void DapProtocol::HandleSwjClock(
    const uint8_t* req, LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  // TODO: Implement actual clock frequency control if needed
  static uint8_t response[2];
  response[0] = static_cast<uint8_t>(CommandId::SWJ_Clock);
  response[1] = 0x00;  // Status: OK

  response_callback.Run(true, response, 2);
}

void DapProtocol::HandleSwjSequence(
    const uint8_t* req, LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  // TODO: Implement actual SWJ sequence if needed
  static uint8_t response[2];
  response[0] = static_cast<uint8_t>(CommandId::SWJ_Sequence);
  response[1] = 0x00;  // Status: OK

  response_callback.Run(true, response, 2);
}

void DapProtocol::HandleSwdConfigure(
    const uint8_t* req, LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  // TODO: Implement actual SWD configuration if needed
  static uint8_t response[2];
  response[0] = static_cast<uint8_t>(CommandId::SWD_Configure);
  response[1] = 0x00;  // Status: OK

  response_callback.Run(true, response, 2);
}

void DapProtocol::HandleSwdSequence(
    const uint8_t* req, LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  // TODO: Implement actual SWD sequence if needed
  static uint8_t response[2];
  response[0] = static_cast<uint8_t>(CommandId::SWD_Sequence);
  response[1] = 0x00;  // Status: OK

  response_callback.Run(true, response, 2);
}

void DapProtocol::HandleTransferConfigure(
    const uint8_t* req, LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  // TODO: Implement actual transfer configuration if needed
  static uint8_t response[2];
  response[0] = static_cast<uint8_t>(CommandId::TransferConfigure);
  response[1] = 0x00;  // Status: OK

  response_callback.Run(true, response, 2);
}

void DapProtocol::HandleTransfer(
    const uint8_t* req, LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  if (state_.debug_port == DapPort::DISABLED || !swd_transfer_method_)
  {
    static uint8_t response[2];
    response[0] = static_cast<uint8_t>(CommandId::Transfer);
    response[1] = DAP_TRANSFER_NO_TARGET;
    response_callback.Run(true, response, 2);
    return;
  }

  const uint8_t dap_index = req[0];
  const uint8_t transfer_count = req[1];
  const uint8_t* transfer_requests = req + 2;
  const uint8_t* write_data_ptr =
      transfer_requests + transfer_count;  // Write data follows requests

  static uint8_t response[256];
  response[0] = static_cast<uint8_t>(CommandId::Transfer);
  response[1] = transfer_count;  // Echo transfer count

  size_t response_data_pos = 2;
  uint8_t transfer_status = 0;
  size_t write_data_offset = 0;

  // Process each transfer request
  for (uint8_t i = 0; i < transfer_count; i++)
  {
    const uint8_t request = transfer_requests[i];
    bool is_read = (request & DAP_TRANSFER_RnW);
    uint32_t write_data = 0;

    // Extract write data for write operations
    if (!is_read && write_data_ptr != nullptr)
    {
      write_data = static_cast<uint32_t>(write_data_ptr[write_data_offset]) |
                   (static_cast<uint32_t>(write_data_ptr[write_data_offset + 1]) << 8) |
                   (static_cast<uint32_t>(write_data_ptr[write_data_offset + 2]) << 16) |
                   (static_cast<uint32_t>(write_data_ptr[write_data_offset + 3]) << 24);
      write_data_offset += 4;
    }

    // Call the transfer method with parsed request data and the original response
    // callback
    LibXR::ErrorCode result =
        swd_transfer_method_(request, write_data, response_callback);

    // NOTE - Unknown how to handle multiple transfers properly here.
    if (result != LibXR::ErrorCode::OK)
    {
      // If transfer method failed, send error response immediately
      static uint8_t error_response[2];
      error_response[0] = static_cast<uint8_t>(CommandId::Transfer);
      error_response[1] = DAP_TRANSFER_ERROR;
      response_callback.Run(true, error_response, 2);
    }

    // For single transfers, the transfer method should call response_callback directly
    // For multiple transfers, we need to handle aggregation
    if (transfer_count == 1)
    {
      // Single transfer - response will be handled by transfer_method callback
      return;
    }
  }

  // For multiple transfers, send aggregated response
  // TODO: Implement proper multiple transfer handling with response aggregation
  static uint8_t multi_response[3];
  multi_response[0] = static_cast<uint8_t>(CommandId::Transfer);
  multi_response[1] = transfer_count;
  multi_response[2] = DAP_TRANSFER_OK;  // (simplified)

  response_callback.Run(true, multi_response, 3);
}

void DapProtocol::HandleTransferBlock(
    const uint8_t* req, LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  // TODO: This is a critical command that needs full implementation for actual debugging
  static uint8_t response[3];
  response[0] = static_cast<uint8_t>(CommandId::TransferBlock);
  response[1] = 0xFF;  // Status: Error
  response[2] = 0x00;  // No data transferred

  response_callback.Run(true, response, 3);
}

void DapProtocol::HandleResetTarget(
    LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  // TODO: Implement actual target reset if needed
  static uint8_t response[2];
  response[0] = static_cast<uint8_t>(CommandId::ResetTarget);
  response[1] = 0x00;  // Status: OK

  response_callback.Run(true, response, 2);
}

void DapProtocol::HandleHostStatus(
    const uint8_t* req, LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  uint8_t status = req[0];  // Status bitmask
  (void)req[1];             // Reserved for future use (was target_state)

  // LED Control based on DAPLink implementation
  // bit 0: Connected status - controls LED when debugger is connected
  // bit 1: Running status - controls LED when target is running
  uint8_t connected_status = status & 0x01;
  uint8_t running_status = (status >> 1) & 0x01;

  // Control LED based on status (active-low like DAPLink)
  // LED ON when NOT connected and NOT running (idle state)
  bool led_on = !(connected_status || running_status);

  if (led_on)
  {
    // LED On - Set output low (active low)
    io_.gpio_led.Write(false);
  }
  else
  {
    // LED Off - Set output high (active low)
    io_.gpio_led.Write(true);
  }

  static uint8_t response[2];
  response[0] = static_cast<uint8_t>(CommandId::HostStatus);  // Echo command ID
  response[1] = 0x00;                                         // Status: OK (DAP_OK)

  response_callback.Run(true, response, 2);
}

}  // namespace DAP
