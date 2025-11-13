#pragma once

#include <array>
#include <cstring>

#include "dap_protocol.hpp"
#include "hid.hpp"

namespace LibXR::USB
{

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
  /**
   * @brief CMSIS-DAP V1 HID Interface
   * @param io DAP I/O interface reference
   * @param in_ep_interval IN endpoint polling interval (ms)
   * @param out_ep_interval OUT endpoint polling interval (ms)
   */
  HIDCmsisDap(DAP::DapIo& io, uint8_t in_ep_interval = 1, uint8_t out_ep_interval = 1)
      : HID(false, in_ep_interval, out_ep_interval, Endpoint::EPNumber::EP_AUTO,
            Endpoint::EPNumber::EP_AUTO),
        dap_engine_(io)
  {
  }

 private:
  DAP::DapProtocol dap_engine_;

 protected:
  /**
   * @brief Get HID report descriptor
   * @return ConstRawData containing CMSIS-DAP report descriptor
   */
  ConstRawData GetReportDesc() override
  {
    return ConstRawData(CMSIS_DAP_REPORT_DESC, sizeof(CMSIS_DAP_REPORT_DESC));
  }

  /**
   * @brief Handle HID SET_REPORT request
   * @param report_id Report ID (unused for CMSIS-DAP)
   * @param result Request result structure
   * @return ErrorCode indicating operation status
   */
  ErrorCode OnSetReport(uint8_t report_id, DeviceClass::RequestResult& result) override
  {
    (void)report_id;

    static uint8_t dummy_buffer[64];
    result.read_data = {dummy_buffer, sizeof(dummy_buffer)};

    return ErrorCode::OK;
  }

  /**
   * @brief Process DAP commands received via HID SET_REPORT
   * @param in_isr Whether called from interrupt context
   * @param data Command data received
   * @return ErrorCode indicating operation status
   */
  ErrorCode OnSetReportData(bool in_isr, ConstRawData& data) override
  {
    UNUSED(in_isr);

    if (data.size_ == 0 || data.addr_ == nullptr)
    {
      static uint8_t error_response[64] = {0};
      SendInputReport(ConstRawData{error_response, 64});
      return ErrorCode::OK;
    }

    const auto* request = static_cast<const uint8_t*>(data.addr_);

    // Handle TransferAbort command
    if (request[0] == 0x18)  // ID_DAP_TransferAbort
    {
      static uint8_t abort_response[64] = {0};
      abort_response[0] = request[0];
      abort_response[1] = 0x00;
      SendInputReport(ConstRawData{abort_response, 64});
      return ErrorCode::OK;
    }

    auto response_callback = LibXR::Callback<const uint8_t*, size_t>::Create(
        [](bool in_isr, HIDCmsisDap* self, const uint8_t* response_data, size_t response_len) {
          UNUSED(in_isr);

          static uint8_t response_packet[64];
          std::memset(response_packet, 0, sizeof(response_packet));

          size_t copy_len = (response_len > 64) ? 64 : response_len;
          std::memcpy(response_packet, response_data, copy_len);

          self->SendInputReport(ConstRawData{response_packet, 64});
        },
        this);

    dap_engine_.ExecuteCommand(request, response_callback);

    return ErrorCode::OK;
  }
};

}  // namespace LibXR::USB
