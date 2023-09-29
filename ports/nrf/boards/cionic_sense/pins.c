#include "shared-bindings/board/__init__.h"

STATIC const mp_rom_map_elem_t board_module_globals_table[] = {
    CIRCUITPYTHON_BOARD_DICT_STANDARD_ITEMS

    { MP_ROM_QSTR(MP_QSTR_LED), MP_ROM_PTR(&pin_P1_02) },
    { MP_ROM_QSTR(MP_QSTR_LED_RED), MP_ROM_PTR(&pin_P1_03) },
    { MP_ROM_QSTR(MP_QSTR_LED_BLUE), MP_ROM_PTR(&pin_P1_01) },
    { MP_ROM_QSTR(MP_QSTR_LED_GREEN), MP_ROM_PTR(&pin_P1_02) },

    { MP_ROM_QSTR(MP_QSTR_BNO_CS), MP_ROM_PTR(&pin_P0_14) },
    { MP_ROM_QSTR(MP_QSTR_BNO_INT), MP_ROM_PTR(&pin_P0_12) },
    { MP_ROM_QSTR(MP_QSTR_BNO_RST), MP_ROM_PTR(&pin_P0_11) },
    { MP_ROM_QSTR(MP_QSTR_BNO_PS0), MP_ROM_PTR(&pin_P1_05) },

    { MP_ROM_QSTR(MP_QSTR_SCL_INT), MP_ROM_PTR(&pin_P0_04) },
    { MP_ROM_QSTR(MP_QSTR_SDA_INT), MP_ROM_PTR(&pin_P0_05) },
    
    { MP_ROM_QSTR(MP_QSTR_SCK), MP_ROM_PTR(&pin_P0_13) },
    { MP_ROM_QSTR(MP_QSTR_MOSI), MP_ROM_PTR(&pin_P0_15) },
    { MP_ROM_QSTR(MP_QSTR_MISO), MP_ROM_PTR(&pin_P0_17) },
    
    { MP_ROM_QSTR(MP_QSTR_SCL), MP_ROM_PTR(&pin_P0_00) },
    { MP_ROM_QSTR(MP_QSTR_SDA), MP_ROM_PTR(&pin_P0_01) },

    { MP_ROM_QSTR(MP_QSTR_I2C), MP_ROM_PTR(&board_i2c_obj) },
    { MP_ROM_QSTR(MP_QSTR_SPI), MP_ROM_PTR(&board_spi_obj) },
    { MP_ROM_QSTR(MP_QSTR_STEMMA_I2C), MP_ROM_PTR(&board_i2c_obj) },
};

MP_DEFINE_CONST_DICT(board_module_globals, board_module_globals_table);
