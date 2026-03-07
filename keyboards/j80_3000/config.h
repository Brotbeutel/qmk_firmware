// Copyright 2025 Brotbeutel (@Brotbeutel)
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

/* KEY MATRIX SIZE */
#define MATRIX_ROWS 8
#define MATRIX_COLS 18

/*
 * WIRING — J80-3000 with I2C1 on PB6/PB7
 *
 * Column index -> source:
 *  0: MCU PC14    1: MCU PA3
 *  2: MCP B0      3: MCP B1      4: MCP B2      5: MCP B3
 *  6: MCU PB0     7: MCU PB1
 *  8: MCP B4
 *  9: MCU PB10
 * 10: MCP B5     11: MCP B6     12: MCP B7
 * 13: MCU PB12   14: MCU PB13   15: MCU PB14   16: MCU PB15
 * 17: MCP A0
 *
 * Row index -> source:
 *  0: MCU PB5     1: MCU PA1     2: MCU PA0
 *  3: MCU PB8     4: MCU PB9
 *  5: MCP A2      6: MCP A3      7: MCP A4
 *
 * LED (active-high):
 *  NumLock    -> MCU PA15
 *  CapsLock   -> MCU PB3
 *  ScrollLock -> MCU PA2
 */

/* I2C1 on PB6=SCL, PB7=SDA — both AF4
 *
 * Speed: ~1.3MHz (Fast Mode Plus)
 * APB1 = 48MHz -> t_CLK = 20.83ns
 *   PRESC=0  -> t_PRESC = 20.83ns
 *   SCLL=23  -> t_SCLL  = 24 * 20.83ns = 500ns  (FM+ min: 500ns)
 *   SCLH=12  -> t_SCLH  = 13 * 20.83ns = 271ns  (FM+ min: 260ns)
 *   SCLDEL=2 -> t_SCLDEL = 3 * 20.83ns =  63ns  (FM+ min:  50ns)
 *   SDADEL=0
 * MCP23017 max SCL: 1.7MHz — this is within spec.
 */
#define I2C_DRIVER          I2CD1
#define I2C1_SCL_PIN        B6
#define I2C1_SDA_PIN        B7
#define I2C1_SCL_PAL_MODE   4
#define I2C1_SDA_PAL_MODE   4
#define I2C1_TIMINGR_PRESC  0U
#define I2C1_TIMINGR_SCLDEL 2U
#define I2C1_TIMINGR_SDADEL 0U
#define I2C1_TIMINGR_SCLH   12U
#define I2C1_TIMINGR_SCLL   23U

/* Debounce: 3ms, eager on keydown for snappy response */
#define DEBOUNCE 3
#define DEBOUNCE_TYPE asym_eager_defer_pk

/* Caps Word: deactivates after 5 seconds of inactivity */
#define CAPS_WORD_IDLE_TIMEOUT 5000
