#pragma once
#include "gpio.hpp"
#include "spi.hpp"

namespace DAP
{

/**
 * @struct DapIo
 * @brief
 * A template-based container for injecting concrete LibXR hardware drivers
 * into DapProtocol. This allows for static dispatch and compile-time selection
 * of hardware implementations.
 *
 * @tparam SpiType The concrete SPI driver class (e.g., CH32_SPI)
 * @tparam GpioType The concrete GPIO driver class (e.g., CH32_GPIO)
 */
/**
 * @struct DapIo
 * @brief
 * A container for injecting LibXR hardware resources into DapProtocol.
 * It holds references to the abstract base classes (SPI&, GPIO&),
 * allowing for dynamic polymorphism.
 */
struct DapIo
{
  // --- Abstract Hardware Driver References ---
  LibXR::SPI& spi;
  LibXR::GPIO& gpio_swdio;  // Combined read/write pin
  LibXR::GPIO& gpio_tdo;    // JTAG TDO, if separate
  LibXR::GPIO& gpio_nreset;

  // --- Constructor for Dependency Injection ---
  DapIo(LibXR::SPI& spi_bus, LibXR::GPIO& swdio_pin, LibXR::GPIO& tdo_pin,
        LibXR::GPIO& nreset_pin)
      : spi(spi_bus), gpio_swdio(swdio_pin), gpio_tdo(tdo_pin), gpio_nreset(nreset_pin)
  {
  }
};

}  // namespace DAP
