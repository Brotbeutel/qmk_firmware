// Copyright 2025 Brotbeutel (@Brotbeutel)
// SPDX-License-Identifier: GPL-2.0-or-later

#include "matrix.h"
#include "wait.h"
#include "i2c_master.h"
#include "gpio.h"
#include "debounce.h"
#include "j80_3000.h"

#define I2C_ADDR    MCP_I2C_ADDR
#define MCP_IODIRA  0x00
#define MCP_IODIRB  0x01
#define MCP_GPPUA   0x0C
#define MCP_GPPUB   0x0D
#define MCP_GPIOA   0x12
#define MCP_GPIOB   0x13
#define MCP_TIMEOUT MCP_TIMEOUT_MS

/* GPA0=in(col17), GPA1=in(unused), GPA2-4=out(rows 5-7), GPA5-7=in(unused) */
#define IODIRA_VALUE 0xE3
#define IODIRB_VALUE 0xFF
#define GPPUA_VALUE  0xE3
#define GPPUB_VALUE  0xFF
#define OLATA_IDLE   0xFF

/* ── Column map ───────────────────────────────────────────────────────────── */
typedef enum { SRC_MCU, SRC_MCP_A, SRC_MCP_B } col_source_t;

typedef struct {
    col_source_t src;
    union {
        pin_t   mcu_pin;
        uint8_t mcp_bit;
    };
} col_def_t;

static const col_def_t col_map[MATRIX_COLS] = {
    [0]  = { SRC_MCU,   .mcu_pin = C14 },  // PCB Pin 03
    [1]  = { SRC_MCU,   .mcu_pin = A3  },  // PCB Pin 09
    [2]  = { SRC_MCP_B, .mcp_bit = 0   },  // MCP GPB0, PCB Pin 10
    [3]  = { SRC_MCP_B, .mcp_bit = 1   },  // MCP GPB1, PCB Pin 11
    [4]  = { SRC_MCP_B, .mcp_bit = 2   },  // MCP GPB2, PCB Pin 12
    [5]  = { SRC_MCP_B, .mcp_bit = 3   },  // MCP GPB3, PCB Pin 13
    [6]  = { SRC_MCU,   .mcu_pin = B0  },  // PCB Pin 14
    [7]  = { SRC_MCU,   .mcu_pin = B1  },  // PCB Pin 15
    [8]  = { SRC_MCP_B, .mcp_bit = 4   },  // MCP GPB4, PCB Pin 16
    [9]  = { SRC_MCU,   .mcu_pin = B10 },  // PCB Pin 17
    [10] = { SRC_MCP_B, .mcp_bit = 5   },  // MCP GPB5, PCB Pin 18
    [11] = { SRC_MCP_B, .mcp_bit = 6   },  // MCP GPB6, PCB Pin 19
    [12] = { SRC_MCP_B, .mcp_bit = 7   },  // MCP GPB7, PCB Pin 20
    [13] = { SRC_MCU,   .mcu_pin = B12 },  // PCB Pin 21
    [14] = { SRC_MCU,   .mcu_pin = B13 },  // PCB Pin 22
    [15] = { SRC_MCU,   .mcu_pin = B14 },  // PCB Pin 23
    [16] = { SRC_MCU,   .mcu_pin = B15 },  // PCB Pin 24
    [17] = { SRC_MCP_A, .mcp_bit = 0   },  // MCP GPA0, PCB Pin 29
};

/* ── Row pins ─────────────────────────────────────────────────────────────── */
static const pin_t   mcu_row_pins[] = { B5, A1, A0, B8, B9 };
static const uint8_t mcp_row_bits[] = { (1<<2), (1<<3), (1<<4) };
#define MCU_ROW_COUNT 5
#define MCP_ROW_COUNT 3

/* ── State ────────────────────────────────────────────────────────────────── */
static bool    i2c_initialized  = false;
bool           mcp_ready        = false;
uint8_t        mcp_olata        = OLATA_IDLE;
static uint8_t mcp_gpioa_cache  = 0xFF;
static uint8_t mcp_gpiob_cache  = 0xFF;
static bool    mcp_cache_valid  = false;

static matrix_row_t raw_matrix[MATRIX_ROWS];
static matrix_row_t matrix_data[MATRIX_ROWS];

/* ── Helpers ──────────────────────────────────────────────────────────────── */
static inline bool mcp_write(uint8_t reg, uint8_t val) {
    return i2c_write_register(I2C_ADDR, reg, &val, 1, MCP_TIMEOUT) == I2C_STATUS_SUCCESS;
}

static bool init_mcp23017(void) {
    if (!i2c_initialized) {
        i2c_init();
        i2c_initialized = true;
        wait_ms(100);
    }
    if (!mcp_write(MCP_IODIRA, IODIRA_VALUE)) return false;
    if (!mcp_write(MCP_IODIRB, IODIRB_VALUE)) return false;
    if (!mcp_write(MCP_GPPUA,  GPPUA_VALUE))  return false;
    if (!mcp_write(MCP_GPPUB,  GPPUB_VALUE))  return false;
    mcp_olata = OLATA_IDLE;
    if (!mcp_write(MCP_OLATA,  mcp_olata))    return false;
    return true;
}

/* Read GPIOA + GPIOB in a single 2-byte I2C transaction (sequential registers) */
static inline void refresh_mcp_cache(void) {
    uint8_t buf[2];
    mcp_cache_valid = (i2c_read_register(I2C_ADDR, MCP_GPIOA, buf, 2, MCP_TIMEOUT) == I2C_STATUS_SUCCESS);
    if (mcp_cache_valid) {
        mcp_gpioa_cache = buf[0];
        mcp_gpiob_cache = buf[1];
    }
}

/*
 * read_cols: build row bitmask from MCU GPIO ports and cached MCP registers.
 *
 * MCU pins are read as full port snapshots to minimise IDR accesses:
 *   GPIOB holds cols 6,7,9,13,14,15,16
 *   GPIOA holds col 1
 *   GPIOC holds col 0
 *
 * MCP cols are extracted from the cached GPIOA/GPIOB bytes written by
 * refresh_mcp_cache().  Active-low convention: bit clear = key pressed.
 */
static matrix_row_t read_cols(void) {
    matrix_row_t val = 0;

    /* Single IDR read per port — avoids 9 separate gpio_read_pin calls */
    uint32_t pb = palReadPort(GPIOB);
    uint32_t pa = palReadPort(GPIOA);
    uint32_t pc = palReadPort(GPIOC);

    if (!(pc & (1U<<14))) val |= (MATRIX_ROW_SHIFTER <<  0); // C14
    if (!(pa & (1U<< 3))) val |= (MATRIX_ROW_SHIFTER <<  1); // A3
    if (!(pb & (1U<< 0))) val |= (MATRIX_ROW_SHIFTER <<  6); // B0
    if (!(pb & (1U<< 1))) val |= (MATRIX_ROW_SHIFTER <<  7); // B1
    if (!(pb & (1U<<10))) val |= (MATRIX_ROW_SHIFTER <<  9); // B10
    if (!(pb & (1U<<12))) val |= (MATRIX_ROW_SHIFTER << 13); // B12
    if (!(pb & (1U<<13))) val |= (MATRIX_ROW_SHIFTER << 14); // B13
    if (!(pb & (1U<<14))) val |= (MATRIX_ROW_SHIFTER << 15); // B14
    if (!(pb & (1U<<15))) val |= (MATRIX_ROW_SHIFTER << 16); // B15

    if (mcp_cache_valid) {
        /* GPIOB: GPB0-7 -> cols 2,3,4,5,8,10,11,12 */
        uint8_t b = ~mcp_gpiob_cache;
        if (b & (1<<0)) val |= (MATRIX_ROW_SHIFTER <<  2);
        if (b & (1<<1)) val |= (MATRIX_ROW_SHIFTER <<  3);
        if (b & (1<<2)) val |= (MATRIX_ROW_SHIFTER <<  4);
        if (b & (1<<3)) val |= (MATRIX_ROW_SHIFTER <<  5);
        if (b & (1<<4)) val |= (MATRIX_ROW_SHIFTER <<  8);
        if (b & (1<<5)) val |= (MATRIX_ROW_SHIFTER << 10);
        if (b & (1<<6)) val |= (MATRIX_ROW_SHIFTER << 11);
        if (b & (1<<7)) val |= (MATRIX_ROW_SHIFTER << 12);
        /* GPIOA: GPA0 -> col 17 */
        if (~mcp_gpioa_cache & (1<<0)) val |= (MATRIX_ROW_SHIFTER << 17);
    }

    return val;
}

/* ── Public API ───────────────────────────────────────────────────────────── */
void matrix_init(void) {
    /* All MCU row pins start as inputs with pull-up (high-Z / not driving) */
    for (uint8_t r = 0; r < MCU_ROW_COUNT; r++)
        gpio_set_pin_input_high(mcu_row_pins[r]);

    /* All MCU column pins as inputs with pull-up */
    for (uint8_t c = 0; c < MATRIX_COLS; c++)
        if (col_map[c].src == SRC_MCU)
            gpio_set_pin_input_high(col_map[c].mcu_pin);

    /* LED pins — active-high outputs, start low (off) */
    gpio_set_pin_output(A15); gpio_write_pin_low(A15);
    gpio_set_pin_output(B3);  gpio_write_pin_low(B3);
    gpio_set_pin_output(A2);  gpio_write_pin_low(A2);

    mcp_ready = init_mcp23017();
    debounce_init();
}

/*
 * matrix_scan — optimised scan loop
 *
 * Three performance improvements vs the naive approach:
 *
 * 1. wait_us reduced from 30us to 1us for MCU rows.
 *    GPIO output settles in <1us; 1us is a comfortable margin.
 *    MCP rows need NO explicit wait: the I2C write that selects the row
 *    already takes ~300us, so the pin has been driven for that entire
 *    duration before we read anything.
 *
 * 2. Consecutive MCP rows share a single OLATA write at the transition.
 *    Old: write-select(N) ... write-idle ... write-select(N+1)  [3 writes for 2 rows]
 *    New: write-select(N) ... write-select(N+1)                 [2 writes for 2 rows]
 *    Writing select(N+1) atomically deselects row N because the new OLATA
 *    value drives the previous row bit high.  This saves 2 I2C transactions
 *    per full scan (~800us at 1.3MHz).
 *
 * 3. I2C clock raised to ~1.3MHz (FM+) in config.h, saving ~830us/scan.
 *
 * Expected improvement: ~170Hz -> ~250Hz.
 */
uint8_t matrix_scan(void) {
    bool changed = false;

    for (uint8_t row = 0; row < MATRIX_ROWS; row++) {

        /* ── Select row ───────────────────────────────────────────────── */
        if (row < MCU_ROW_COUNT) {
            gpio_set_pin_output(mcu_row_pins[row]);
            gpio_write_pin_low(mcu_row_pins[row]);
            wait_us(1);
        } else if (mcp_ready) {
            uint8_t idx = row - MCU_ROW_COUNT;
            /* Drive this MCP row low.  For rows 6 and 7 this simultaneously
             * releases the previous row — no separate deselect write needed. */
            mcp_write(MCP_OLATA, mcp_olata & ~mcp_row_bits[idx]);
            /* No wait_us: the I2C write itself provides >100us of settling. */
        }

        /* ── Sample columns ───────────────────────────────────────────── */
        if (mcp_ready) refresh_mcp_cache();
        matrix_row_t row_val = read_cols();

        /* ── Deselect row ─────────────────────────────────────────────── */
        if (row < MCU_ROW_COUNT) {
            gpio_set_pin_input_high(mcu_row_pins[row]);
        } else if (mcp_ready) {
            /* Only restore idle after the final MCP row.
             * Earlier MCP rows are deselected implicitly by the next select. */
            if (row == MATRIX_ROWS - 1)
                mcp_write(MCP_OLATA, mcp_olata);
        }

        /* ── Record change ────────────────────────────────────────────── */
        if (raw_matrix[row] != row_val) {
            raw_matrix[row] = row_val;
            changed = true;
        }
    }

    changed = debounce(raw_matrix, matrix_data, changed);
    return changed;
}

matrix_row_t matrix_get_row(uint8_t row) { return matrix_data[row]; }
void         matrix_print(void)          {}
