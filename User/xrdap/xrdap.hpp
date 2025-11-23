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

/**
 * @brief Converts XRDAP SPI response data into DAP_TRANSFER response format.
 *
 * This function parses the 48-bit SPI response from the XRDAP probe and converts it
 * into the standard DAP Transfer response format used by CMSIS-DAP.
 *
 * @param request The original DAP request byte for context.
 * @param rx_data 6-byte SPI response data received from XRDAP probe.
 * @param[out] response Output buffer for DAP response data.
 * @param[out] response_size Size of the generated DAP response.
 * @return True if response was successfully generated, false on error.
 */
bool GenerateDapTransferResponse(uint8_t request, const uint8_t rx_data[6],
                                 uint8_t response[7], size_t& response_size);

}  // namespace DAP