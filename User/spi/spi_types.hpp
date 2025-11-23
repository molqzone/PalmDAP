#pragma once

#include <cstdint>
#include "libxr.hpp"

namespace DAP
{

// Queue item for SPI transfer requests
// Contains the 48-bit SPI sequence and the USB response callback
struct SpiTransferRequest {
  uint64_t spi_frame;                                     // 48-bit SPI frame for XRDAP
  LibXR::Callback<const uint8_t*, size_t> response_callback;  // USB response callback

  SpiTransferRequest() : spi_frame(0) {}
  SpiTransferRequest(uint64_t frame, LibXR::Callback<const uint8_t*, size_t> callback)
    : spi_frame(frame), response_callback(callback) {}
};

// Transfer method function pointer - embedded-friendly, no dynamic allocation
typedef LibXR::ErrorCode (*TransferMethod)(uint8_t request, uint32_t write_data,
                                           LibXR::Callback<const uint8_t*, size_t> response_callback);

}  // namespace DAP