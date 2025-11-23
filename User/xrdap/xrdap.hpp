#pragma once

#include <cstdint>

#include "libxr.hpp"
#include "spi_types.hpp"

namespace DAP
{

/**
 * @brief Generates 48-bit SPI transaction data for XRDAP-SWD-Probe.
 *
 * @param request DAP transfer request byte (RnW, APnDP, address bits).
 * @param write_data 32-bit data for write operations (ignored for reads).
 * @return 48-bit frame to send via SPI to the XRDAP probe.
 */
uint64_t GenerateSpiTransaction(uint8_t request, uint32_t write_data = 0);

}  // namespace DAP