#pragma once
#include "gpio.hpp"
#include "spi.hpp"

namespace DAP
{

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
