// Copyright 2025 Brotbeutel (@Brotbeutel)
// SPDX-License-Identifier: GPL-2.0-or-later

#include "matrix.h"
#include "wait.h"
#include "gpio.h"
#include "debounce.h"
#include "spi_master.h"
#include "j80_3000.h"

/* ── MCP23S17 register addresses ─────────────────────────────────────────── */
#define MCP_IODIRA  0x00
#define MCP_IODIRB  0x01
#define MCP_GPPUA   0x0C
#define MCP_GPPUB   0x0D
#define MCP_GPIOA   0x12
#define MCP_GPIOB   0x13
/* MCP_OLATA (0x14) defined in j80_3000.h */

/*
 * GPA0       = input  (COL17)
 * GPA1       = input  (unused)
 * GPA2–GPA4  = output (ROW5–ROW7)
 * GPA5–GPA7  = input  (unused)
 * GPB0–GPB7  = input  (COL2–COL5, COL8, COL10–COL12)
 */
#define IODIRA_VALUE 0xE3
#define IODIRB_VALUE 0xFF
#define GPPUA_VALUE  0xE3
#define GPPUB_VALUE  0xFF
#define OLATA_IDLE   0xFF

/* ── Row/column definitions ───────────────────────────────────────────────── */

static const pin_t mcu_row_pins[] = { B5, A1, A0, B8, B9 };
#define MCU_ROW_COUNT 5

static const uint8_t mcp_row_bits[] = { (1 << 2), (1 << 3), (1 << 4) };

/* MCU column pins in matrix order — only used for init (input_high setup).
 * read_cols() accesses the GPIO ports directly for speed. */
static const pin_t mcu_col_pins[] = { C14, A3, B7, B1, B10, B12, B13, B14, B15 };

/* ── Module state ─────────────────────────────────────────────────────────── */

static bool    mcp_ready      = false;
static uint8_t mcp_olata      = OLATA_IDLE;
static uint8_t mcp_gpioa      = 0xFF;
static uint8_t mcp_gpiob      = 0xFF;

static matrix_row_t raw_matrix[MATRIX_ROWS];
static matrix_row_t matrix_data[MATRIX_ROWS];

/* ── SPI helpers ──────────────────────────────────────────────────────────── */

static void mcp_write_reg(uint8_t reg, uint8_t val) {
    spi_start(MCP_CS_PIN, false, 0, MCP_SPI_DIVISOR);
    spi_write(MCP_WRITE_OPCODE);
    spi_write(reg);
    spi_write(val);
    spi_stop();
}

static uint8_t mcp_read_reg(uint8_t reg) {
    spi_start(MCP_CS_PIN, false, 0, MCP_SPI_DIVISOR);
    spi_write(MCP_READ_OPCODE);
    spi_write(reg);
    uint8_t val = (uint8_t)spi_read();
    spi_stop();
    return val;
}

/* Reads GPIOA + GPIOB in one transaction (sequential-read, SEQOP=0 default). */
static inline void mcp_read_gpio(void) {
    spi_start(MCP_CS_PIN, false, 0, MCP_SPI_DIVISOR);
    spi_write(MCP_READ_OPCODE);
    spi_write(MCP_GPIOA);
    mcp_gpioa = (uint8_t)spi_read();
    mcp_gpiob = (uint8_t)spi_read();
    spi_stop();
}

/* ── MCP23S17 init ────────────────────────────────────────────────────────── */

static bool init_mcp23s17(void) {
    spi_init();
    gpio_set_pin_output(MCP_CS_PIN);
    gpio_write_pin_high(MCP_CS_PIN);
    wait_ms(10);

    mcp_write_reg(MCP_IODIRA, IODIRA_VALUE);
    mcp_write_reg(MCP_IODIRB, IODIRB_VALUE);
    mcp_write_reg(MCP_GPPUA,  GPPUA_VALUE);
    mcp_write_reg(MCP_GPPUB,  GPPUB_VALUE);
    mcp_write_reg(MCP_OLATA,  OLATA_IDLE);

    /* Verify comms by reading back a known register */
    return (mcp_read_reg(MCP_IODIRA) == IODIRA_VALUE);
}

/* ── KITT startup sequence ────────────────────────────────────────────────── */
/*
 * Physical LED order, left to right:
 *   index 0 = NumLock    (PA15)
 *   index 1 = CapsLock   (PB3)
 *   index 2 = ScrollLock (PB4)
 *
 * Three tunable parameters (set in config.h):
 *   KITT_DURATION_MS   — total wall-clock time for all runs combined
 *   KITT_RUNS          — number of complete left→right→left sweeps
 *   KITT_END_DWELL_MS  — extra pause at the left and right end positions
 *
 * Sequence shape: 1,2,3,[dwell],2,1,[dwell],  1,2,3,[dwell],2,1,[dwell], ...
 *
 * Step timing is derived automatically:
 *   Each run has (2n-1) steps and 2 end-dwells.
 *   step_ms = (KITT_DURATION_MS/KITT_RUNS - 2*KITT_END_DWELL_MS) / (2n-1)
 */
static void knight_rider_sequence(void) {
    static const pin_t leds[] = { A15, B3, B4 };
    const uint8_t n = sizeof(leds) / sizeof(leds[0]);

    /* Derived step duration — clamped to 10ms minimum to stay visible */
    const uint8_t  steps_per_run  = 2 * n - 1;
    const uint16_t time_per_run   = KITT_DURATION_MS / KITT_RUNS;
    const uint16_t dwell_budget   = 2 * KITT_END_DWELL_MS;
    const uint16_t step_ms        = (time_per_run > dwell_budget)
        ? ((time_per_run - dwell_budget) / steps_per_run)
        : 10;

    for (uint8_t run = 0; run < KITT_RUNS; run++) {
        /* Forward sweep: LED0 → LED(n-1) */
        for (uint8_t i = 0; i < n; i++) {
            for (uint8_t j = 0; j < n; j++)
                gpio_write_pin(leds[j], j == i);
            wait_ms(step_ms);
            if (i == n - 1) wait_ms(KITT_END_DWELL_MS);
        }
        /* Backward sweep: LED(n-2) → LED0
         * Start at n-2 so the right-end LED isn't shown twice. */
        for (int8_t i = (int8_t)(n - 2); i >= 0; i--) {
            for (uint8_t j = 0; j < n; j++)
                gpio_write_pin(leds[j], j == (uint8_t)i);
            wait_ms(step_ms);
            if (i == 0) wait_ms(KITT_END_DWELL_MS);
        }
        /* LED0 at end of this run merges naturally with LED0 at start of the
         * next run, producing the correct left-end double-frame. */
    }

    /* All off — QMK indicator system takes over */
    for (uint8_t i = 0; i < n; i++)
        gpio_write_pin_low(leds[i]);
}

/* ── Column read ──────────────────────────────────────────────────────────── */

static matrix_row_t read_cols(void) {
    matrix_row_t val = 0;

    uint32_t pa = palReadPort(GPIOA);
    uint32_t pb = palReadPort(GPIOB);
    uint32_t pc = palReadPort(GPIOC);

    /* MCU columns — active-low */
    if (!(pc & (1U << 14))) val |= (MATRIX_ROW_SHIFTER <<  0);  /* C14 = COL0  */
    if (!(pa & (1U <<  3))) val |= (MATRIX_ROW_SHIFTER <<  1);  /* A3  = COL1  */
    if (!(pb & (1U <<  7))) val |= (MATRIX_ROW_SHIFTER <<  6);  /* B7  = COL6  */
    if (!(pb & (1U <<  1))) val |= (MATRIX_ROW_SHIFTER <<  7);  /* B1  = COL7  */
    if (!(pb & (1U << 10))) val |= (MATRIX_ROW_SHIFTER <<  9);  /* B10 = COL9  */
    if (!(pb & (1U << 12))) val |= (MATRIX_ROW_SHIFTER << 13);  /* B12 = COL13 */
    if (!(pb & (1U << 13))) val |= (MATRIX_ROW_SHIFTER << 14);  /* B13 = COL14 */
    if (!(pb & (1U << 14))) val |= (MATRIX_ROW_SHIFTER << 15);  /* B14 = COL15 */
    if (!(pb & (1U << 15))) val |= (MATRIX_ROW_SHIFTER << 16);  /* B15 = COL16 */

    /* MCP columns — active-low */
    uint8_t b = ~mcp_gpiob;
    if (b & (1 << 0)) val |= (MATRIX_ROW_SHIFTER <<  2);        /* GPB0 = COL2  */
    if (b & (1 << 1)) val |= (MATRIX_ROW_SHIFTER <<  3);        /* GPB1 = COL3  */
    if (b & (1 << 2)) val |= (MATRIX_ROW_SHIFTER <<  4);        /* GPB2 = COL4  */
    if (b & (1 << 3)) val |= (MATRIX_ROW_SHIFTER <<  5);        /* GPB3 = COL5  */
    if (b & (1 << 4)) val |= (MATRIX_ROW_SHIFTER <<  8);        /* GPB4 = COL8  */
    if (b & (1 << 5)) val |= (MATRIX_ROW_SHIFTER << 10);        /* GPB5 = COL10 */
    if (b & (1 << 6)) val |= (MATRIX_ROW_SHIFTER << 11);        /* GPB6 = COL11 */
    if (b & (1 << 7)) val |= (MATRIX_ROW_SHIFTER << 12);        /* GPB7 = COL12 */
    if (~mcp_gpioa & (1 << 0))
        val |= (MATRIX_ROW_SHIFTER << 17);                       /* GPA0 = COL17 */

    return val;
}

/* ── Public API ───────────────────────────────────────────────────────────── */

void matrix_init(void) {
    /*
     * PA4 = SPI1 hardware-NSS AND CS of the onboard W25Q64 flash chip.
     * Must stay HIGH — keep as input-high so the flash never activates.
     */
    gpio_set_pin_input_high(A4);

    for (uint8_t r = 0; r < MCU_ROW_COUNT; r++)
        gpio_set_pin_input_high(mcu_row_pins[r]);

    for (uint8_t c = 0; c < (sizeof(mcu_col_pins) / sizeof(mcu_col_pins[0])); c++)
        gpio_set_pin_input_high(mcu_col_pins[c]);

    gpio_set_pin_output(A15); gpio_write_pin_low(A15);
    gpio_set_pin_output(B3);  gpio_write_pin_low(B3);
    gpio_set_pin_output(B4);  gpio_write_pin_low(B4);

    mcp_ready = init_mcp23s17();

    knight_rider_sequence();

    debounce_init();
    matrix_init_kb();  /* required: enables keyboard/user-level init hooks */
}

uint8_t matrix_scan(void) {
    bool changed = false;

    for (uint8_t row = 0; row < MATRIX_ROWS; row++) {

        /* Select row */
        if (row < MCU_ROW_COUNT) {
            gpio_set_pin_output(mcu_row_pins[row]);
            gpio_write_pin_low(mcu_row_pins[row]);
            wait_us(1);
        } else if (mcp_ready) {
            uint8_t idx = row - MCU_ROW_COUNT;
            mcp_write_reg(MCP_OLATA, mcp_olata & ~mcp_row_bits[idx]);
            wait_us(5);
        }

        /* Read MCP GPIO while row is active, then sample all columns */
        if (mcp_ready) mcp_read_gpio();
        matrix_row_t row_val = read_cols();

        /* Deselect row */
        if (row < MCU_ROW_COUNT) {
            gpio_set_pin_input_high(mcu_row_pins[row]);
        } else if (mcp_ready) {
            mcp_write_reg(MCP_OLATA, mcp_olata);
        }

        if (raw_matrix[row] != row_val) {
            raw_matrix[row] = row_val;
            changed = true;
        }
    }

    changed = debounce(raw_matrix, matrix_data, changed);
    matrix_scan_kb();  /* required: enables keyboard/user-level scan hooks */
    return changed;
}

matrix_row_t matrix_get_row(uint8_t row) { return matrix_data[row]; }
void         matrix_print(void)          {}

/* ── QMK callback stubs ──────────────────────────────────────────────────── */
/* These weak symbols allow keyboard.c and keymap.c to override behaviour     */
/* without breaking builds that don't need them.                              */
__attribute__((weak)) void matrix_init_kb(void)  { matrix_init_user(); }
__attribute__((weak)) void matrix_scan_kb(void)  { matrix_scan_user(); }
__attribute__((weak)) void matrix_init_user(void) {}
__attribute__((weak)) void matrix_scan_user(void) {}
