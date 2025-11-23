#pragma once

#include "gpio.hpp"
#include "spi.hpp"

namespace DAP
{

/**
 * @brief XRDAP-specific I/O interface for PalmDAP
 *
 * This structure provides direct access to XRDAP-SWD-Probe specific GPIO signals
 * and hardware control. It's designed specifically for XRDAP implementation
 * without unnecessary abstraction layers.
 */
struct DapIo
{
  LibXR::SPI& spi;           ///< SPI interface for XRDAP communication
  LibXR::GPIO& gpio_swdio;   ///< SWDIO pin (for compatibility)
  LibXR::GPIO& gpio_tdo;     ///< JTAG TDO pin (optional)
  LibXR::GPIO& gpio_nreset;  ///< Target reset pin
  LibXR::GPIO& gpio_led;     ///< DAP status LED
  LibXR::GPIO& gpio_rst_n;   ///< XRDAP frame reset signal (rst_n)
  LibXR::GPIO& gpio_rnw;     ///< XRDAP read/write control (rnw)

  /**
   * @brief Construct XRDAP DAP I/O interface
   *
   * @param spi_bus SPI interface for XRDAP communication
   * @param swdio_pin SWDIO pin (for compatibility)
   * @param tdo_pin JTAG TDO pin (optional)
   * @param nreset_pin Target reset pin
   * @param led_pin Status LED pin
   * @param rst_n_pin XRDAP frame reset signal (rst_n)
   * @param rnw_pin XRDAP read/write control signal (rnw)
   */
  DapIo(LibXR::SPI& spi_bus, LibXR::GPIO& swdio_pin, LibXR::GPIO& tdo_pin,
        LibXR::GPIO& nreset_pin, LibXR::GPIO& led_pin, LibXR::GPIO& rst_n_pin,
        LibXR::GPIO& rnw_pin)
      : spi(spi_bus),
        gpio_swdio(swdio_pin),
        gpio_tdo(tdo_pin),
        gpio_nreset(nreset_pin),
        gpio_led(led_pin),
        gpio_rst_n(rst_n_pin),
        gpio_rnw(rnw_pin)
  {
  }

  /**
   * @brief Initialize SPI for XRDAP communication
   *
   * Configures SPI with XRDAP-required settings:
   * - CPOL = 0, CPHA = 0
   * - LSB first transmission
   */
  void InitializeXrdapSpi()
  {
    LibXR::SPI::Configuration config = {LibXR::SPI::ClockPolarity::LOW,
                                        LibXR::SPI::ClockPhase::EDGE_1};
    spi.SetConfig(config);
  }

  /**
   * @brief Enter RAW mode for line reset operations
   *
   * In RAW mode: rst_n=0, MOSI→SWDIO direct
   * Used for line reset and SWJ sequences.
   */
  void EnterRawMode() { gpio_rst_n.Write(0); }

  /**
   * @brief Exit RAW mode and return to normal operation
   */
  void ExitRawMode() { gpio_rst_n.Write(1); }

  /**
   * @brief Set read/write direction for XRDAP transaction
   *
   * @param is_read True for READ operation (rnw=1), false for WRITE (rnw=0)
   */
  void SetReadWriteDirection(bool is_read) { gpio_rnw.Write(is_read ? 1 : 0); }

  /**
   * @brief Perform frame reset before XRDAP transaction
   *
   * According to XRDAP protocol: "每帧开始前短暂拉低一次 rst_n，作为该帧的逻辑复位"
   * This should be a very brief pulse to reset the frame counter.
   */
  void ResetFrame()
  {
    gpio_rst_n.Write(0);
    // Very brief reset pulse - hardware needs just a few cycles
    // Could use a small delay or NOP cycles for precise timing
    for (volatile int i = 0; i < 10; i++) { __asm__("nop"); }  // Brief delay
    gpio_rst_n.Write(1);
  }
};

}  // namespace DAP
