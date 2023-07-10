#include "nrfx/hal/nrf_gpio.h"

#define MICROPY_HW_BOARD_NAME       "Cionic Sense nRF52840"
#define MICROPY_HW_MCU_NAME         "nRF52840"

#define MICROPY_HW_LED_STATUS       (&pin_P0_03)

#if QSPI_FLASH_FILESYSTEM
#define MICROPY_QSPI_DATA0                NRF_GPIO_PIN_MAP(0, 21)
#define MICROPY_QSPI_DATA1                NRF_GPIO_PIN_MAP(0, 7)
#define MICROPY_QSPI_DATA2                NRF_GPIO_PIN_MAP(1, 9)
#define MICROPY_QSPI_DATA3                NRF_GPIO_PIN_MAP(1, 0)
#define MICROPY_QSPI_SCK                  NRF_GPIO_PIN_MAP(0, 11)
#define MICROPY_QSPI_CS                   NRF_GPIO_PIN_MAP(0, 25)
#endif

#if SPI_FLASH_FILESYSTEM
#define SPI_FLASH_MOSI_PIN &pin_P0_06
#define SPI_FLASH_MISO_PIN &pin_P1_08
#define SPI_FLASH_SCK_PIN &pin_P0_08
#define SPI_FLASH_CS_PIN &pin_P0_05
#endif

// Board does not have a 32kHz crystal. It does have a 32MHz crystal, in the module.
#define BOARD_HAS_32KHZ_XTAL (0)

#define DEFAULT_I2C_BUS_SCL         (&pin_P0_13)
#define DEFAULT_I2C_BUS_SDA         (&pin_P0_14)
