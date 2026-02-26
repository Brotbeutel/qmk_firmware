#pragma once

/* KEY MATRIX SIZE */
#define MATRIX_ROWS 8
#define MATRIX_COLS 18

/*
 * WIRING — J80-3000 with I2C1 on PB6/PB7
 *
 * Column index → source:
 *  0: MCU PC14    1: MCU PA3
 *  2: MCP B0      3: MCP B1      4: MCP B2      5: MCP B3
 *  6: MCU PB0     7: MCU PB1
 *  8: MCP B4
 *  9: MCU PB10
 * 10: MCP B5     11: MCP B6     12: MCP B7
 * 13: MCU PB12   14: MCU PB13   15: MCU PB14   16: MCU PB15
 * 17: MCP A0
 *
 * Row index → source:
 *  0: MCU PB5     1: MCU PA1     2: MCU PA0
 *  3: MCU PB8     4: MCU PB9
 *  5: MCP A2      6: MCP A3      7: MCP A4
 *
 * LED:
 *  LED1 (NumLock)   → MCU PA15
 *  LED2 (CapsLock)  → MCU PB3
 *  LED3 (ScrollLock)→ MCP A1
 */

/* I2C1 on PB6=SCL, PB7=SDA — both AF4, well-tested in QMK */
#define I2C_DRIVER          I2CD1
#define I2C1_SCL_PIN        B6
#define I2C1_SDA_PIN        B7
#define I2C1_SCL_PAL_MODE   4
#define I2C1_SDA_PAL_MODE   4
#define I2C1_TIMINGR_PRESC  2U
#define I2C1_TIMINGR_SCLDEL 1U
#define I2C1_TIMINGR_SDADEL 0U
#define I2C1_TIMINGR_SCLH   9U
#define I2C1_TIMINGR_SCLL   26U

/* Dynamic keymap / VIA */
#ifndef DYNAMIC_KEYMAP_LAYER_COUNT
#define DYNAMIC_KEYMAP_LAYER_COUNT 3
#endif
#ifndef DYNAMIC_KEYMAP_EEPROM_ADDR
#define DYNAMIC_KEYMAP_EEPROM_ADDR (EECONFIG_SIZE)
#endif
