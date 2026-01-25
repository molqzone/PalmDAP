#include "app_main.h"

#include "cdc_uart.hpp"
#include "libxr.hpp"
#include "main.h"
#include "stm32_adc.hpp"
#include "stm32_can.hpp"
#include "stm32_canfd.hpp"
#include "stm32_dac.hpp"
#include "stm32_flash.hpp"
#include "stm32_gpio.hpp"
#include "stm32_i2c.hpp"
#include "stm32_power.hpp"
#include "stm32_pwm.hpp"
#include "stm32_spi.hpp"
#include "stm32_timebase.hpp"
#include "stm32_uart.hpp"
#include "stm32_usb_dev.hpp"
#include "stm32_watchdog.hpp"
#include "flash_map.hpp"

using namespace LibXR;

/* User Code Begin 1 */
/* User Code End 1 */
// NOLINTBEGIN
// clang-format off
/* External HAL Declarations */
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* No DMA Resources generated. */

extern "C" void app_main(void) {
  // clang-format on
  // NOLINTEND
  /* User Code Begin 2 */
  
  /* User Code End 2 */
  // clang-format off
  // NOLINTBEGIN
  STM32Timebase timebase;
  PlatformInit(2, 1024);
  STM32PowerManager power_manager;

  /* GPIO Configuration */




  /* Terminal Configuration */

  // clang-format on
  // NOLINTEND
  /* User Code Begin 3 */
  while(true) {
    Thread::Sleep(UINT32_MAX);
  }
  /* User Code End 3 */
}