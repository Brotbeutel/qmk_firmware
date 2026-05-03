// Copyright 2025 Brotbeutel (@Brotbeutel)
// SPDX-License-Identifier: GPL-2.0-or-later

#include "matrix.h"
#include "wait.h"
#include "gpio.h"
#include "debounce.h"
#include "j80_3000.h"

/* MCP23S17 register addresses */
#define MCP_IODIRA  0x00
#define MCP_IODIRB  0x01
#define MCP_GPPUA   0x0C
#define MCP_GPPUB   0x0D
#define MCP_GPIOA   0x12
#define MCP_GPIOB   0x13

/* GPA0=in(col17), GPA2-4=out(rows 5-7) */
#define IODIRA_VALUE 0xE3
#define IODIRB_VALUE 0xFF
#define GPPUA_VALUE  0xE3
#define GPPUB_VALUE  0xFF
#define OLATA_IDLE   0xFF

/* ── Bit-bang SPI ─────────────────────────────────────────────────────────
 * SCK=PA5, MOSI=PA7, MISO=PA6, CS=PB6
 * CPOL=0 CPHA=0: clock idles LOW, data sampled on rising edge
 */
static inline void mcp_cs_low(void)  { gpio_write_pin_low(B6); }
static inline void mcp_cs_high(void) { gpio_write_pin_high(B6); }

static inline void bb_spi_init(void) {
    gpio_set_pin_output(A5); gpio_write_pin_low(A5);   // SCK
    gpio_set_pin_output(A7); gpio_write_pin_low(A7);   // MOSI
    gpio_set_pin_input_high(A6);                        // MISO — internal pull-up + external 10k
    gpio_set_pin_output(B6); gpio_write_pin_high(B6);  // CS
}

static inline void bb_byte_out(uint8_t b) {
    for (int8_t i = 7; i >= 0; i--) {
        gpio_write_pin(A7, (b >> i) & 1);
        __NOP(); __NOP(); __NOP(); __NOP();
        gpio_write_pin_high(A5);
        __NOP(); __NOP(); __NOP(); __NOP();
        gpio_write_pin_low(A5);
        __NOP(); __NOP(); __NOP(); __NOP();
    }
}

static inline uint8_t bb_byte_in(void) {
    uint8_t b = 0;
    for (int8_t i = 7; i >= 0; i--) {
        gpio_write_pin_low(A5);   /* SCK LOW — MCP puts next bit on MISO */
        wait_us(2);               /* wait for open-drain MISO to charge via pull-up */
        gpio_write_pin_high(A5);  /* SCK HIGH — sample MISO */
        b |= (gpio_read_pin(A6) << i);
    }
    gpio_write_pin_low(A5);
    return b;
}

static inline void mcp_write_reg(uint8_t reg, uint8_t val) {
    mcp_cs_low();
    bb_byte_out(MCP_WRITE_OPCODE);
    bb_byte_out(reg);
    bb_byte_out(val);
    mcp_cs_high();
}

/* ── Column map ───────────────────────────────────────────────────────────── */
typedef enum { SRC_MCU, SRC_MCP_A, SRC_MCP_B } col_source_t;

typedef struct {
    col_source_t src;
    union { pin_t mcu_pin; uint8_t mcp_bit; };
} col_def_t;

static const col_def_t col_map[MATRIX_COLS] = {
    [0]  = { SRC_MCU,   .mcu_pin = C14 },
    [1]  = { SRC_MCU,   .mcu_pin = A3  },
    [2]  = { SRC_MCP_B, .mcp_bit = 0   },
    [3]  = { SRC_MCP_B, .mcp_bit = 1   },
    [4]  = { SRC_MCP_B, .mcp_bit = 2   },
    [5]  = { SRC_MCP_B, .mcp_bit = 3   },
    [6]  = { SRC_MCU,   .mcu_pin = B7  },  // B0 had F401 conflict, using B7
    [7]  = { SRC_MCU,   .mcu_pin = B1  },
    [8]  = { SRC_MCP_B, .mcp_bit = 4   },
    [9]  = { SRC_MCU,   .mcu_pin = B10 },
    [10] = { SRC_MCP_B, .mcp_bit = 5   },
    [11] = { SRC_MCP_B, .mcp_bit = 6   },
    [12] = { SRC_MCP_B, .mcp_bit = 7   },
    [13] = { SRC_MCU,   .mcu_pin = B12 },
    [14] = { SRC_MCU,   .mcu_pin = B13 },
    [15] = { SRC_MCU,   .mcu_pin = B14 },
    [16] = { SRC_MCU,   .mcu_pin = B15 },
    [17] = { SRC_MCP_A, .mcp_bit = 0   },
};

/* ── Row pins ─────────────────────────────────────────────────────────────── */
static const pin_t   mcu_row_pins[] = { B5, A1, A0, B8, B9 };
static const uint8_t mcp_row_bits[] = { (1<<2), (1<<3), (1<<4) };
#define MCU_ROW_COUNT 5
#define MCP_ROW_COUNT 3

/* ── State ────────────────────────────────────────────────────────────────── */
bool    mcp_ready      = false;
uint8_t mcp_olata      = OLATA_IDLE;
static uint8_t mcp_gpioa_cache = 0xFF;
static uint8_t mcp_gpiob_cache = 0xFF;
static bool    mcp_cache_valid = false;

static matrix_row_t raw_matrix[MATRIX_ROWS];
static matrix_row_t matrix_data[MATRIX_ROWS];

static bool init_mcp23s17(void) {
    bb_spi_init();
    wait_ms(10);
    mcp_write_reg(MCP_IODIRA, IODIRA_VALUE);
    mcp_write_reg(MCP_IODIRB, IODIRB_VALUE);
    mcp_write_reg(MCP_GPPUA,  GPPUA_VALUE);
    mcp_write_reg(MCP_GPPUB,  GPPUB_VALUE);
    mcp_olata = OLATA_IDLE;
    mcp_write_reg(MCP_OLATA,  mcp_olata);
    return true;
}

static inline void refresh_mcp_cache(void) {
    mcp_cs_low();
    bb_byte_out(MCP_READ_OPCODE);
    bb_byte_out(MCP_GPIOA);
    mcp_gpioa_cache = bb_byte_in();
    mcp_gpiob_cache = bb_byte_in();
    mcp_cs_high();
    mcp_cache_valid = true;
}

static matrix_row_t read_cols(void) {
    matrix_row_t val = 0;

    uint32_t pb = palReadPort(GPIOB);
    uint32_t pa = palReadPort(GPIOA);
    uint32_t pc = palReadPort(GPIOC);

    if (!(pc & (1U<<14))) val |= (MATRIX_ROW_SHIFTER <<  0);
    if (!(pa & (1U<< 3))) val |= (MATRIX_ROW_SHIFTER <<  1);
    if (!(pb & (1U<< 7))) val |= (MATRIX_ROW_SHIFTER <<  6);  // B7 -> col 6
    if (!(pb & (1U<< 1))) val |= (MATRIX_ROW_SHIFTER <<  7);
    if (!(pb & (1U<<10))) val |= (MATRIX_ROW_SHIFTER <<  9);
    if (!(pb & (1U<<12))) val |= (MATRIX_ROW_SHIFTER << 13);
    if (!(pb & (1U<<13))) val |= (MATRIX_ROW_SHIFTER << 14);
    if (!(pb & (1U<<14))) val |= (MATRIX_ROW_SHIFTER << 15);
    if (!(pb & (1U<<15))) val |= (MATRIX_ROW_SHIFTER << 16);

    if (mcp_cache_valid) {
        uint8_t b = ~mcp_gpiob_cache;
        if (b & (1<<0)) val |= (MATRIX_ROW_SHIFTER <<  2);
        if (b & (1<<1)) val |= (MATRIX_ROW_SHIFTER <<  3);
        if (b & (1<<2)) val |= (MATRIX_ROW_SHIFTER <<  4);
        if (b & (1<<3)) val |= (MATRIX_ROW_SHIFTER <<  5);
        if (b & (1<<4)) val |= (MATRIX_ROW_SHIFTER <<  8);
        if (b & (1<<5)) val |= (MATRIX_ROW_SHIFTER << 10);
        if (b & (1<<6)) val |= (MATRIX_ROW_SHIFTER << 11);
        if (b & (1<<7)) val |= (MATRIX_ROW_SHIFTER << 12);
        if (~mcp_gpioa_cache & (1<<0)) val |= (MATRIX_ROW_SHIFTER << 17);
    }

    return val;
}

void matrix_init(void) {
    for (uint8_t r = 0; r < MCU_ROW_COUNT; r++)
        gpio_set_pin_input_high(mcu_row_pins[r]);
    for (uint8_t c = 0; c < MATRIX_COLS; c++)
        if (col_map[c].src == SRC_MCU)
            gpio_set_pin_input_high(col_map[c].mcu_pin);

    gpio_set_pin_output(A15); gpio_write_pin_low(A15);
    gpio_set_pin_output(B3);  gpio_write_pin_low(B3);
    gpio_set_pin_output(B4);  gpio_write_pin_low(B4);

    mcp_ready = init_mcp23s17();

    debounce_init();
}

uint8_t matrix_scan(void) {
    bool changed = false;

    for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
        if (row < MCU_ROW_COUNT) {
            gpio_set_pin_output(mcu_row_pins[row]);
            gpio_write_pin_low(mcu_row_pins[row]);
            wait_us(1);
        } else if (mcp_ready) {
            uint8_t idx = row - MCU_ROW_COUNT;
            mcp_write_reg(MCP_OLATA, mcp_olata & ~mcp_row_bits[idx]);
            wait_us(5);
        }

        if (mcp_ready) refresh_mcp_cache();
        matrix_row_t row_val = read_cols();

        if (row < MCU_ROW_COUNT) {
            gpio_set_pin_input_high(mcu_row_pins[row]);
        } else if (mcp_ready) {
            if (row == MATRIX_ROWS - 1)
                mcp_write_reg(MCP_OLATA, mcp_olata);
        }

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
