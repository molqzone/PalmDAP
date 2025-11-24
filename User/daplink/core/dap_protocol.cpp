#include "dap_protocol.hpp"

#include <cstring>
#include <functional>

#include "dap_config.hpp"
#include "dap_io.hpp"
namespace DAP
{

DapProtocol::DapProtocol(DapIo& io, TransferMethod transfer_method)
    : io_(io),
      swd_transfer_method_(transfer_method),
      spi_callback_(LibXR::Callback<LibXR::ErrorCode>::Create(
          [](bool in_isr, DapProtocol* self, LibXR::ErrorCode ec)
          { self->HandleSpiWriteComplete(in_isr, 0, ec); }, this))
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
  // Initialize XRDAP SPI for SWD communication
  io_.InitializeXrdapSpi();

  // Set initial GPIO states (configuration already done in app_main)
  io_.gpio_nreset.Write(true);  // Deassert nRESET
  io_.gpio_led.Write(false);    // LED off initially

  // For XRDAP, the actual SWD initialization is handled by the hardware
  // when normal SWD transfers begin. We just need to ensure the hardware
  // is in the correct state for XRDAP operation.

  // Set rnw to a known state (WRITE mode initially)
  io_.SetReadWriteDirection(false);

  // Ensure we're in normal XRDAP operation mode (not RAW mode)
  io_.ExitRawMode();

  // Turn on LED to indicate SWD mode is ready
  io_.gpio_led.Write(true);

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

  LibXR::WriteOperation spi_op(spi_callback_);
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

void DapProtocol::HandleSpiWriteComplete(bool in_isr, int context, LibXR::ErrorCode ec)
{
  UNUSED(in_isr);
  UNUSED(context);
  UNUSED(ec);
  // Currently this is a dummy callback - sequence transmission completes synchronously
  // through the SPI hardware, so no additional processing is needed here
}

void DapProtocol::HandleSwjPins(const uint8_t* req,
                                LibXR::Callback<const uint8_t*, size_t> response_callback)
{
  // DAP_SWJ_Pins command format: [0x10] [Pin_select] [Pin_values] [Wait_time(L)]
  // [Wait_time(H)]
  const uint8_t pin_select = req[0];
  const uint8_t pin_values = req[1];
  const uint16_t wait_time = static_cast<uint16_t>(req[2] | (req[3] << 8));

  static uint8_t response[2];
  response[0] = static_cast<uint8_t>(CommandId::SWJ_Pins);

  // Initialize XRDAP SPI if needed
  io_.InitializeXrdapSpi();

  // Control SWJ pins based on selection mask
  uint8_t actual_output = 0;

  // Control nRESET pin if selected
  if (pin_select & DAP_SWJ_nRESET)
  {
    bool nreset_state = (pin_values & DAP_SWJ_nRESET) != 0;
    io_.gpio_nreset.Write(nreset_state);
    if (nreset_state)
    {
      actual_output |= DAP_SWJ_nRESET;
    }
  }

  // Control TDI pin if selected (for JTAG mode)
  if (pin_select & DAP_SWJ_TDI)
  {
    bool tdi_state = (pin_values & DAP_SWJ_TDI) != 0;
    // For XRDAP, TDI can be controlled via SWDIO in RAW mode or GPIO
    io_.gpio_swdio.Write(tdi_state);
    if (tdi_state)
    {
      actual_output |= DAP_SWJ_TDI;
    }
  }

  // Control SWDIO/TMS pin if selected
  if (pin_select & DAP_SWJ_SWDIO_TMS)
  {
    bool swdio_state = (pin_values & DAP_SWJ_SWDIO_TMS) != 0;
    io_.gpio_swdio.Write(swdio_state);
    if (swdio_state)
    {
      actual_output |= DAP_SWJ_SWDIO_TMS;
    }
  }

  // Wait for specified time (in microseconds) if requested
  if (wait_time > 0)
  {
    LibXR::Thread::Sleep(wait_time / 1000);  // Convert microseconds to milliseconds
  }

  // Read actual pin states for response
  uint8_t pin_status = actual_output;

  // Read TDO pin if supported
  if (pin_select & DAP_SWJ_TDO)
  {
    bool tdo_state = io_.gpio_tdo.Read();
    if (tdo_state)
    {
      pin_status |= DAP_SWJ_TDO;
    }
  }

  response[1] = pin_status;
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
  // DAP_SWJ_Sequence command format: [0x12] [Bit_count] [Sequence_data...]
  const uint8_t bit_count = req[0];
  const uint8_t* sequence_data = &req[1];

  static uint8_t response[2];
  response[0] = static_cast<uint8_t>(CommandId::SWJ_Sequence);

  // Validate bit count (must be non-zero)
  if (bit_count == 0)
  {
    response[1] = static_cast<uint8_t>(DAP_TRANSFER_ERROR);
    response_callback.Run(true, response, 2);
    return;
  }

  // Calculate number of bytes needed for the sequence
  const uint8_t byte_count = (bit_count + 7) / 8;  // Round up to nearest byte

  // Initialize SPI in XRDAP mode
  io_.InitializeXrdapSpi();

  // Enter RAW mode for SWJ sequence transmission
  io_.EnterRawMode();

  LibXR::WriteOperation spi_op(spi_callback_);

  // Send the sequence data via SPI (MOSI → SWDIO direct pass-through in RAW mode)
  LibXR::ErrorCode err = io_.spi.Write({sequence_data, byte_count}, spi_op);

  // Exit RAW mode after sequence transmission
  io_.ExitRawMode();

  if (err == LibXR::ErrorCode::OK)
  {
    response[1] = static_cast<uint8_t>(Status::OK);
  }
  else
  {
    response[1] = static_cast<uint8_t>(DAP_TRANSFER_ERROR);
  }

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
