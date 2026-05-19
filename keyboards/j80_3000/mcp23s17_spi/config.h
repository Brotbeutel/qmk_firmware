// Copyright 2025 Brotbeutel (@Brotbeutel)
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

/* KEY MATRIX SIZE */
#define MATRIX_ROWS 8
#define MATRIX_COLS 18

/*
 * WIRING — J80-3000 with SPI1 on PA5/PA6/PA7, CS=PB6
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
 * LED (active-high):
 *  NumLock    -> MCU PA15
 *  CapsLock   -> MCU PB3
 *  ScrollLock -> MCU PB4
 *
 * SPI1 (hardware, ChibiOS SPID1 via QMK spi_master):
 *  SCK  -> PA5  (AF5)
 *  MOSI -> PA7  (AF5)
 *  MISO -> PA6  (AF5) — 1kΩ external pull-up to 3.3V (MCP SO is open-drain)
 *  CS   -> PB6  (GPIO, managed by spi_master)
 *  ⚠️  PA4 = SPI1 hardware-NSS AND onboard W25Q64 flash CS — must stay HIGH
 */

/* SPI1 pin definitions for QMK spi_master / ChibiOS */
#define SPI_DRIVER        SPID1
#define SPI_SCK_PIN       A5
#define SPI_SCK_PAL_MODE  5
#define SPI_MOSI_PIN      A7
#define SPI_MOSI_PAL_MODE 5
#define SPI_MISO_PIN      A6
#define SPI_MISO_PAL_MODE 5

/* Debounce — configured in keyboard.json (debounce: 3, debounce_type: asym_eager_defer_pk) */

/* ── USB performance ──────────────────────────────────────────────────────── */
/* 1ms polling interval = 1000Hz — reduces input latency from ~10ms to <5ms   */
#define USB_POLLING_INTERVAL_MS 1
/* Process up to 12 key events per scan cycle (default: 4).                   */
/* Prevents key event queuing during fast typing or combo use.                 */
#define QMK_KEYS_PER_SCAN 12

/* ── Scan rate measurement (debug only) ──────────────────────────────────── */
/* Uncomment + add CONSOLE_ENABLE = yes in rules.mk, then run: qmk console    */
/* Output example:  > matrix scan frequency: 1840                             */
// #define DEBUG_MATRIX_SCAN_RATE

/* ── KITT startup sequence ────────────────────────────────────────────────── */
/* Total duration of all runs combined, in milliseconds.                       */
#define KITT_DURATION_MS   4000
/* Number of complete left → right → left sweeps.                             */
#define KITT_RUNS          3
/* Extra pause at each end (left-most and right-most LED), in milliseconds.   */
#define KITT_END_DWELL_MS  300

/* Caps Word: deactivates after 5 seconds of inactivity */
#define CAPS_WORD_IDLE_TIMEOUT 5000
