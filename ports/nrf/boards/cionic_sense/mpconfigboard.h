#include "nrfx/hal/nrf_gpio.h"

#define MICROPY_HW_BOARD_NAME       "Cionic Sense nRF52840"
#define MICROPY_HW_MCU_NAME         "nRF52840"

#define MICROPY_HW_LED_STATUS       (&pin_P1_02)

#if QSPI_FLASH_FILESYSTEM
#define MICROPY_QSPI_DATA0                NRF_GPIO_PIN_MAP(0, 21)
#define MICROPY_QSPI_DATA1                NRF_GPIO_PIN_MAP(0, 22)
#define MICROPY_QSPI_DATA2                NRF_GPIO_PIN_MAP(0, 23)
#define MICROPY_QSPI_DATA3                NRF_GPIO_PIN_MAP(1, 0)
#define MICROPY_QSPI_SCK                  NRF_GPIO_PIN_MAP(0, 19)
#define MICROPY_QSPI_CS                   NRF_GPIO_PIN_MAP(0, 20)
#endif

// Board does not have a 32kHz crystal. It does have a 32MHz crystal, in the module.
#define BOARD_HAS_32KHZ_XTAL (0)

#define DEFAULT_I2C_BUS_SCL         (&pin_P0_00)
#define DEFAULT_I2C_BUS_SDA         (&pin_P0_01)
