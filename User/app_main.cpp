#include <cmath>

#include "cdc_uart.hpp"
#include "ch32_gpio.hpp"
#include "ch32_spi.hpp"
#include "ch32_timebase.hpp"
#include "ch32_uart.hpp"
#include "ch32_usb.hpp"
#include "ch32_usb_dev.hpp"
#include "ch32v30x_gpio.h"
#include "dap/hid_dap.hpp"
#include "dap_io.hpp"
#include "libxr.hpp"

// EP0: Control, 64 bytes
static uint8_t ep0_buffer_hs[64];
// EP1: HID IN (64 bytes report size)
static uint8_t ep1_buffer_tx_hs[1024];
// EP2: HID OUT (64 bytes report size)
static uint8_t ep2_buffer_rx_hs[1024];
// EP3: CDC Data IN
static uint8_t ep3_buffer_tx_hs[1024];
// EP4: CDC Data OUT
static uint8_t ep4_buffer_rx_hs[1024];
// EP5: CDC Notification IN
static uint8_t ep5_buffer_tx_hs[8];

uint8_t spi_dma_tx_buffer[64], spi_dma_rx_buffer[64];

extern "C" void app_main()
{
  LibXR::CH32SPI spi1(CH32_SPI1, {spi_dma_rx_buffer, 64}, {spi_dma_tx_buffer, 64}, GPIOA,
                      GPIO_Pin_5, GPIOA, GPIO_Pin_6, GPIOA, GPIO_Pin_7);

  LibXR::CH32GPIO gpio_swdio(GPIOA, GPIO_Pin_8);
  LibXR::CH32GPIO gpio_tdo(GPIOA, GPIO_Pin_9);
  LibXR::CH32GPIO gpio_nreset(GPIOA, GPIO_Pin_10);

  DAP::DapIo dap_io_instance(spi1, gpio_swdio, gpio_tdo, gpio_nreset);

  LibXR::USB::HIDCmsisDap dap_interface(dap_io_instance);

  static constexpr auto LANG_PACK_EN_US = LibXR::USB::DescriptorStrings::MakeLanguagePack(
      LibXR::USB::DescriptorStrings::Language::EN_US, "XRobot", "CDC Demo", "123456789");

  LibXR::USB::CDCUart cdc(4096, 4096, 8), cdc1;
  ;
  LibXR::CH32USBDeviceFS usb_device(
      /* EP */
      {
          {ep0_buffer_hs},            // EP0: Control
          {ep1_buffer_tx_hs, true},   // EP1: HID IN
          {ep2_buffer_rx_hs, false},  // EP2: HID OUT
          {ep3_buffer_tx_hs, true},   // EP3: CDC Data IN
          {ep4_buffer_rx_hs, false},  // EP4: CDC Data OUT
          {ep5_buffer_tx_hs, true},   // EP5: CDC Notification IN
      },
      /* packet size */
      LibXR::USB::DeviceDescriptor::PacketSize0::SIZE_64,
      /* vid pid bcd */
      0x1209, 0x0001, 0x0100,
      /* language */
      {&LANG_PACK_EN_US},
      /* config */
      // A compound device with two CDC interfaces and one HID interface
      {
          {&dap_interface},
          {&cdc1},
      });

  usb_device.Init();

  usb_device.Start();

  // Initialize GPIO pins used by SPI and DAP
  LibXR::CH32GPIO spi_sck(GPIOA, GPIO_Pin_5);   // SPI1 SCK
  LibXR::CH32GPIO spi_miso(GPIOA, GPIO_Pin_6);  // SPI1 MISO
  LibXR::CH32GPIO spi_mosi(GPIOA, GPIO_Pin_7);  // SPI1 MOSI

  LibXR::CH32Timebase timebase;

  LibXR::PlatformInit(3, 8192);

  while (1)
  {
    LibXR::Thread::Sleep(1000);
  }
}
