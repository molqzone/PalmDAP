#include "dap_protocol.hpp"

#include <cstring>
#include <functional>

#include "dap_config.hpp"

namespace DAP
{

DapProtocol::DapProtocol() { Setup(); }

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
  response[0] = request[0];                                 // Echo ID

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

      // TODO: Implement other command handlers here

    default:
      response[0] = static_cast<uint8_t>(CommandId::Invalid);
      result.response_generated = 1;
      result.request_consumed = 0;
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
      capabilities |= (1U << 0);   // SWD support
      capabilities |= (1U << 1);   // JTAG support
      data_ptr[0] = capabilities;  // TODO - Support user configuration
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
  DapProtocol::CommandResult result{};
  const auto port = static_cast<Port>(req[0]);

  if (port == Port::SWD || port == Port::JTAG)
  {
    state_.debug_port = static_cast<DapPort>(port);
    res[0] = static_cast<uint8_t>(Status::OK);
    res[1] = static_cast<uint8_t>(port);
  }
  else
  {
    res[0] = static_cast<uint8_t>(Status::Error);
    res[1] = static_cast<uint8_t>(Port::Disabled);
  }

  result.request_consumed = 1;
  result.response_generated = 2;
  return result;
}

DapProtocol::CommandResult DapProtocol::HandleDisconnect(uint8_t* res)
{
  DapProtocol::CommandResult result{};
  state_.debug_port = DapPort::DISABLED;
  res[0] = static_cast<uint8_t>(Status::OK);
  result.request_consumed = 0;
  result.response_generated = 1;
  return result;
}

}  // namespace DAP
