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

  // Buffer for storing responses to be returned via GET_REPORT
  std::array<uint8_t, 64> current_response_{};
  size_t current_response_len_ = 0;

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
    (void)result;

    return ErrorCode::OK;
  }

  // Process DAP commands received via HID SET_REPORT
  ErrorCode OnSetReportData(bool in_isr, ConstRawData& data) override
  {
    UNUSED(in_isr);

    // Execute DAP command using the CMSIS-DAP protocol engine
    const auto* request = static_cast<const uint8_t*>(data.addr_);
    size_t response_len = dap_engine_.ExecuteCommand(request, response_buffer_.data());

    // Store response for the next GET_REPORT request
    current_response_len_ = response_len;
    if (response_len > 0)
    {
      memcpy(current_response_.data(), response_buffer_.data(), response_len);
    }

    return ErrorCode::OK;
  }

  // Send DAP responses via HID GET_REPORT
  ErrorCode OnGetInputReport(uint8_t report_id,
                             DeviceClass::RequestResult& result) override
  {
    (void)report_id;

    // Return the most recent DAP response
    if (current_response_len_ > 0)
    {
      result.write_data = ConstRawData{current_response_.data(), current_response_len_};
    }
    else
    {
      // Empty response when no data is available
      result.write_data = ConstRawData{current_response_.data(), 1};
      current_response_[0] = 0x00;
    }

    return ErrorCode::OK;
  }
};

}  // namespace LibXR::USB
