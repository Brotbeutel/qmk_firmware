#pragma once

/* ── Matrix size ─────────────────────────────────────────────────────── */
#define MATRIX_ROWS 8
#define MATRIX_COLS 18

/*
 * WIRING REFERENCE — sourced from Connections sheet (authoritative)
 *
 * Rows (driven LOW to select):
 *   0: MCU PB5    1: MCU PB6    2: MCU PB7    3: MCU PB8    4: MCU PB9
 *   5: MCP GPA2   6: MCP GPA3   7: MCP GPA4
 *
 * Columns (read — active LOW = pressed):
 *   0: MCU PC14   1: MCU PA3
 *   2: MCP GPB3   3: MCP GPB4   4: MCP GPB5   5: MCP GPB6
 *   6: MCU PB0    7: MCU PB1
 *   8: MCP GPB7
 *   9: MCU PB10
 *  10: MCP GPB0  11: MCP GPB1  12: MCP GPB2
 *  13: MCU PB12  14: MCU PB13  15: MCU PB14  16: MCU PB15
 *  17: MCP GPA0
 *
 * LEDs (active LOW):
 *   PA15 → NumLock    PB3 → CapsLock    MCP GPA1 → ScrollLock
 *
 * I²C: PA8 = SCL (I2C3, AF4)    PB4 = SDA (I2C3, AF4)
 */

/* ── I²C driver ─────────────────────────────────────────────────────────
 *
 * STM32F4xx uses I2Cv1 (NOT I2Cv2 — no TIMINGR register!).
 * I2Cv1 is configured via OPMODE, CLOCK_SPEED, DUTY_CYCLE.
 *
 * QMK always uses the I2C1_* prefix for these, regardless of which
 * I2C peripheral is selected via I2C_DRIVER.
 *
 * For 400 kHz Fast Mode on STM32F411 (APB1 = 48 MHz):
 *   CLOCK_SPEED  = 400000   (Hz)
 *   DUTY_CYCLE   = FAST_DUTY_CYCLE_2  (Tlow/Thigh = 16/9, better signal)
 *
 * Pin alternate function: PA8=SCL, PB4=SDA both use AF4 for I2C3.
 */
#define I2C_DRIVER          I2CD3
#define I2C3_SCL_PIN        A8
#define I2C3_SDA_PIN        B4
#define I2C3_SCL_PAL_MODE   4
#define I2C3_SDA_PAL_MODE   4

/* I2Cv1 timing — prefix is always I2C1_ regardless of peripheral */
#define I2C1_OPMODE         OPMODE_I2C
#define I2C1_CLOCK_SPEED    400000
#define I2C1_DUTY_CYCLE     FAST_DUTY_CYCLE_2

/* ── VIA ─────────────────────────────────────────────────────────────── */
#ifndef DYNAMIC_KEYMAP_LAYER_COUNT
#  define DYNAMIC_KEYMAP_LAYER_COUNT 3
#endif

#ifndef DYNAMIC_KEYMAP_EEPROM_ADDR
#  define DYNAMIC_KEYMAP_EEPROM_ADDR (EECONFIG_SIZE)
#endif
