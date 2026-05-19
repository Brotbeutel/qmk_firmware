// Copyright 2025 Brotbeutel (@Brotbeutel)
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

/* KEY MATRIX SIZE */
#define MATRIX_ROWS 8
#define MATRIX_COLS 18

/*
 * WIRING — J80-3000 with bit-bang SPI on PA5/PA6/PA7, CS=PB6
 *
 * Column index -> source:
 *  0: MCU PC14    1: MCU PA3
 *  2: MCP B0      3: MCP B1      4: MCP B2      5: MCP B3
 *  6: MCU PB7     7: MCU PB1
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
 * SPI (bit-bang): SCK=PA5, MOSI=PA7, MISO=PA6, CS=PB6
 * MISO requires external 10kΩ pull-up to 3.3V (MCP23S17 SO is open-drain)
 *
 * LED (active-high):
 *  NumLock    -> MCU PA15
 *  CapsLock   -> MCU PB3
 *  ScrollLock -> MCU PB4
 */

/* Debounce: 3ms, eager on keydown for snappy response */
#define DEBOUNCE 3
#define DEBOUNCE_TYPE asym_eager_defer_pk

/* Caps Word: deactivates after 5 seconds of inactivity */
#define CAPS_WORD_IDLE_TIMEOUT 5000
