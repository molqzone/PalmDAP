#pragma once

#include <array>
#include <cstring>

#include "dap_protocol.hpp"
#include "hid.hpp"

namespace LibXR::USB
{

// CMSIS-DAP V1 HID Report Descriptor (matching DAPLink format)
static constexpr uint8_t CMSIS_DAP_REPORT_DESC[] = {
    0x06, 0x00, 0xFF,  // Usage Page (Vendor Defined 0xFF00)
    0x09, 0x01,        // Usage (Vendor Usage 1)
    0xA1, 0x01,        // Collection (Application)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8 bits)
    0x95, 0x40,        //   Report Count (64 bytes)
    0x09, 0x01,        //   Usage (Vendor Usage 1)
    0x81, 0x02,        //   Input (Data,Var,Abs)
    0x95, 0x40,        //   Report Count (64 bytes)
    0x09, 0x01,        //   Usage (Vendor Usage 1)
    0x91, 0x02,        //   Output (Data,Var,Abs)
    0x95, 0x40,        //   Report Count (64 bytes)
    0x09, 0x01,        //   Usage (Vendor Usage 1)
    0xB1, 0x02,        //   Feature (Data,Var,Abs)
    0xC0               // End Collection
};

class HIDCmsisDap : public HID<sizeof(CMSIS_DAP_REPORT_DESC), 64, 64>
{
 public:
  HIDCmsisDap(DAP::DapIo& io, uint8_t in_ep_interval = 1, uint8_t out_ep_interval = 1)
      : HID(false, in_ep_interval, out_ep_interval, Endpoint::EPNumber::EP_AUTO,
            Endpoint::EPNumber::EP_AUTO),  // Use control transfers only (true CMSIS-DAP
                                           // v1)
        dap_engine_(io)
  {
  }

 private:
  DAP::DapProtocol dap_engine_;

  std::array<uint8_t, 64> response_buffer_{};

 protected:
  ConstRawData GetReportDesc() override
  {
    return ConstRawData(CMSIS_DAP_REPORT_DESC, sizeof(CMSIS_DAP_REPORT_DESC));
  }

  // Accept SET_REPORT requests - required for CMSIS-DAP V1
  ErrorCode OnSetReport(uint8_t report_id, DeviceClass::RequestResult& result) override
  {
    // CMSIS-DAP doesn't use report IDs, but accept any for compatibility
    (void)report_id;

    result.read_data = {response_buffer_.data(), response_buffer_.size()};

    return ErrorCode::OK;
  }

  // Process DAP commands received via HID SET_REPORT
  ErrorCode OnSetReportData(bool in_isr, ConstRawData& data) override
  {
    UNUSED(in_isr);

    // Validate input data length - CMSIS-DAP requires at least 1 byte (command ID)
    if (data.size_ == 0 || data.addr_ == nullptr)
    {
      static uint8_t error_response[64] = {0};
      SendInputReport(ConstRawData{error_response, 64});
      return ErrorCode::OK;
    }

    const auto* request = static_cast<const uint8_t*>(data.addr_);

    // Handle special TransferAbort command (like DAPLink does)
    if (request[0] == 0x18)  // ID_DAP_TransferAbort
    {
      // NOTE - For now, just acknowledge the command
      static uint8_t abort_response[64] = {0};
      abort_response[0] = request[0];  // Echo command ID
      abort_response[1] = 0x00;        // Status: OK
      SendInputReport(ConstRawData{abort_response, 64});
      return ErrorCode::OK;
    }

    // Execute DAP command using the CMSIS-DAP protocol engine
    size_t response_len =
        dap_engine_.ExecuteCommand(request, response_buffer_.data(), in_isr);

    // Send response via interrupt IN endpoint (CMSIS-DAP V1 standard method)
    if (response_len > 0)
    {
      // Create 64-byte response buffer padded with zeros (CMSIS-DAP standard)
      static uint8_t response_packet[64];
      std::memset(response_packet, 0, sizeof(response_packet));

      // Copy response data (max 64 bytes) - preserve the actual DAP response format
      size_t copy_len = (response_len > 64) ? 64 : response_len;
      std::memcpy(response_packet, response_buffer_.data(), copy_len);

      // Send via interrupt IN endpoint
      SendInputReport(ConstRawData{response_packet, 64});
    }
    else
    {
      // Send minimal valid response
      static uint8_t empty_response[64] = {0};
      empty_response[0] = request[0];  // Echo command ID
      SendInputReport(ConstRawData{empty_response, 64});
    }

    return ErrorCode::OK;
  }
};

}  // namespace LibXR::USB
