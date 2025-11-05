#pragma once

#include <array>

#include "dap_protocol.hpp"
#include "hid.hpp"

namespace LibXR::USB
{

// CMSIS-DAP V1 HID Report Descriptor
// It's a simple vendor-defined device with one IN and one OUT report.
static constexpr uint8_t CMSIS_DAP_REPORT_DESC[] = {
    0x06, 0x00, 0xFF,  // Usage Page (Vendor Defined)
    0x09, 0x01,        // Usage (Vendor Defined)
    0xA1, 0x01,        // Collection (Application)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8 bits)
    0x95, 64,          //   Report Count (64 bytes, must match packet size)
    0x09, 0x01,        //   Usage (Vendor Defined)
    0x81, 0x02,        //   Input (Data,Var,Abs) -> To Host
    0x95, 64,          //   Report Count (64 bytes, must match packet size)
    0x09, 0x01,        //   Usage (Vendor Defined)
    0x91, 0x02,        //   Output (Data,Var,Abs) -> From Host
    0xC0               // End Collection
};

class HIDCmsisDap : public HID<sizeof(CMSIS_DAP_REPORT_DESC), 64, 64>
{
 public:
  HIDCmsisDap(DAP::DapIo& io, uint8_t in_ep_interval = 1, uint8_t out_ep_interval = 1)
      : HID(true, in_ep_interval, out_ep_interval, Endpoint::EPNumber::EP_AUTO,
            Endpoint::EPNumber::EP_AUTO),
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

  void OnDataOutComplete(bool in_isr, ConstRawData& data) override
  {
    UNUSED(in_isr);

    // A request from the host. Process it.
    const auto* request = static_cast<const uint8_t*>(data.addr_);
    size_t response_len = dap_engine_.ExecuteCommand(request, response_buffer_.data());

    // Send the generated response back to the host.
    SendInputReport({response_buffer_.data(), response_len});
  }

  void OnDataInComplete(bool in_isr, ConstRawData& data) override
  {
    UNUSED(in_isr);
    UNUSED(data);

    GetOutEndpoint()->Transfer(64);
  }
};

}  // namespace LibXR::USB
