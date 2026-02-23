#pragma once

/* KEY MATRIX SIZE */
#define MATRIX_ROWS 8
#define MATRIX_COLS 18

/*
 * WIRING — matches J80-3000 Pinout spreadsheet exactly
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
 *  0: MCU PB5     1: MCU PB6     2: MCU PB7     3: MCU PB8     4: MCU PB9
 *  5: MCP A2      6: MCP A3      7: MCP A4
 *
 * LED:
 *  LED1 (NumLock)   → MCU PA15
 *  LED2 (CapsLock)  → MCU PB3
 *  LED3 (ScrollLock)→ MCP A1
 */

/* MCU rows (indices 0-4) */
#define MATRIX_ROW_PINS_MCU \
    { B5, B6, B7, B8, B9 }

/* MCU columns — only the MCU-driven ones, in col-index order */
/* Used by matrix.c directly with the col_map below */
/* (not used as a flat array in config.h — see matrix.c col_map) */

/* MCP23017 pin indices (0..15) */
#define MCP_A0   0
#define MCP_A1   1
#define MCP_A2   2
#define MCP_A3   3
#define MCP_A4   4
#define MCP_A5   5
#define MCP_A6   6
#define MCP_A7   7
#define MCP_B0   8
#define MCP_B1   9
#define MCP_B2  10
#define MCP_B3  11
#define MCP_B4  12
#define MCP_B5  13
#define MCP_B6  14
#define MCP_B7  15

/* MCP rows (indices 5-7) → GPIOA bits A2, A3, A4 */
#define MATRIX_ROW_PINS_MCP \
    { MCP_A2, MCP_A3, MCP_A4 }

/* LED pins on MCU */
#define LED_NUM_LOCK_PIN    A15
#define LED_CAPS_LOCK_PIN   B3
/* LED3 (ScrollLock) is on MCP A1 — handled in matrix.c */

/* I2C Configuration for MCP23017 on I2C3 (PA8=SCL, PB4=SDA) */
#define I2C_DRIVER      I2CD3
#define I2C1_SCL_PIN    A8
#define I2C1_SDA_PIN    B4
#define I2C1_SCL_PAL_MODE 4
#define I2C1_SDA_PAL_MODE 4
#define I2C1_TIMINGR_PRESC  2U
#define I2C1_TIMINGR_SCLDEL 1U
#define I2C1_TIMINGR_SDADEL 0U
#define I2C1_TIMINGR_SCLH   9U
#define I2C1_TIMINGR_SCLL  26U

/* Dynamic keymap / VIA EEPROM sizing */
#ifndef DYNAMIC_KEYMAP_LAYER_COUNT
#define DYNAMIC_KEYMAP_LAYER_COUNT 3
#endif

#ifndef DYNAMIC_KEYMAP_EEPROM_ADDR
#define DYNAMIC_KEYMAP_EEPROM_ADDR (EECONFIG_SIZE)
#endif
