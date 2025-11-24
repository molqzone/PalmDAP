#pragma once

#include <cstdint>

#include "libxr.hpp"

namespace DAP
{

struct SpiTransferRequest
{
  uint64_t spi_frame;  // 48-bit SPI frame for XRDAP (8 bytes)
  LibXR::Callback<const uint8_t*, size_t>
      response_callback;  // Response callback (8 bytes)
                          // Total: 16 bytes per request
};

// TransferMethod function pointer for DAP transfers
typedef LibXR::ErrorCode (*TransferMethod)(
    uint8_t request, uint32_t write_data,
    LibXR::Callback<const uint8_t*, size_t> response_callback);

}  // namespace DAP