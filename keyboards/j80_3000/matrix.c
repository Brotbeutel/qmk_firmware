/*
 * matrix.c — J80-3000  *** DEBUG BUILD ***
 * CUSTOM_MATRIX = lite
 */

#include "matrix.h"
#include "wait.h"
#include "i2c_master.h"
#include "gpio.h"
#include "print.h"

#define MCP_ADDR    (0x20 << 1)

#define REG_IODIRA  0x00
#define REG_IODIRB  0x01
#define REG_GPPUA   0x0C
#define REG_GPPUB   0x0D
#define REG_GPIOA   0x12
#define REG_GPIOB   0x13
#define REG_OLATA   0x14

#define MCP_TIMEOUT 50   /* generous timeout for debugging */

#define IODIRA_VAL  0xE1
#define IODIRB_VAL  0xFF
#define GPPUA_VAL   0xE1
#define GPPUB_VAL   0xFF
#define OLATA_IDLE  0xFF

static const uint8_t mcp_row_mask[3] = {
    (1u << 2), (1u << 3), (1u << 4),
};

typedef enum { SRC_MCU, SRC_MCP_B, SRC_MCP_A } col_src_t;
typedef struct {
    col_src_t src;
    union { pin_t mcu_pin; uint8_t mcp_bit; };
} col_def_t;

static const col_def_t col_map[MATRIX_COLS] = {
    [0]  = { SRC_MCU,   .mcu_pin = C14 },
    [1]  = { SRC_MCU,   .mcu_pin = A3  },
    [2]  = { SRC_MCP_B, .mcp_bit = 3   },
    [3]  = { SRC_MCP_B, .mcp_bit = 4   },
    [4]  = { SRC_MCP_B, .mcp_bit = 5   },
    [5]  = { SRC_MCP_B, .mcp_bit = 6   },
    [6]  = { SRC_MCU,   .mcu_pin = B0  },
    [7]  = { SRC_MCU,   .mcu_pin = B1  },
    [8]  = { SRC_MCP_B, .mcp_bit = 7   },
    [9]  = { SRC_MCU,   .mcu_pin = B10 },
    [10] = { SRC_MCP_B, .mcp_bit = 0   },
    [11] = { SRC_MCP_B, .mcp_bit = 1   },
    [12] = { SRC_MCP_B, .mcp_bit = 2   },
    [13] = { SRC_MCU,   .mcu_pin = B12 },
    [14] = { SRC_MCU,   .mcu_pin = B13 },
    [15] = { SRC_MCU,   .mcu_pin = B14 },
    [16] = { SRC_MCU,   .mcu_pin = B15 },
    [17] = { SRC_MCP_A, .mcp_bit = 0   },
};

static const pin_t mcu_row_pins[5] = { B5, B6, B7, B8, B9 };
#define MCU_ROWS 5

bool    mcp_ready = false;
uint8_t mcp_olata = OLATA_IDLE;

static i2c_status_t mcp_write(uint8_t reg, uint8_t val) {
    return i2c_write_register(MCP_ADDR, reg, &val, 1, MCP_TIMEOUT);
}

static i2c_status_t mcp_read2(uint8_t reg, uint8_t *a, uint8_t *b) {
    uint8_t buf[2] = {0xFF, 0xFF};
    i2c_status_t r = i2c_read_register(MCP_ADDR, reg, buf, 2, MCP_TIMEOUT);
    *a = buf[0]; *b = buf[1];
    return r;
}

/* ── Init ─────────────────────────────────────────────────────────────── */
static bool init_mcp23017(void) {
    i2c_status_t s;

    print("MCP: i2c_init\n");
    i2c_init();
    wait_ms(100);   /* extra settle for debug */

    /* Probe: try a write and check the return code */
    print("MCP: probe write IODIRA\n");
    s = mcp_write(REG_IODIRA, IODIRA_VAL);
    xprintf("MCP: IODIRA write -> status %d (0=OK)\n", s);
    if (s != I2C_STATUS_SUCCESS) return false;

    s = mcp_write(REG_IODIRB, IODIRB_VAL);
    xprintf("MCP: IODIRB write -> %d\n", s);
    if (s != I2C_STATUS_SUCCESS) return false;

    s = mcp_write(REG_GPPUA, GPPUA_VAL);
    xprintf("MCP: GPPUA  write -> %d\n", s);
    if (s != I2C_STATUS_SUCCESS) return false;

    s = mcp_write(REG_GPPUB, GPPUB_VAL);
    xprintf("MCP: GPPUB  write -> %d\n", s);
    if (s != I2C_STATUS_SUCCESS) return false;

    mcp_olata = OLATA_IDLE;
    s = mcp_write(REG_OLATA, mcp_olata);
    xprintf("MCP: OLATA  write -> %d\n", s);
    if (s != I2C_STATUS_SUCCESS) return false;

    /* Read back IODIRA to verify communication */
    uint8_t ra = 0, rb = 0;
    s = mcp_read2(REG_IODIRA, &ra, &rb);
    xprintf("MCP: readback IODIRA=0x%02X IODIRB=0x%02X (status %d)\n", ra, rb, s);
    xprintf("MCP: expected  IODIRA=0x%02X IODIRB=0x%02X\n", IODIRA_VAL, IODIRB_VAL);

    print("MCP: init SUCCESS\n");
    return true;
}

static matrix_row_t cols_from_mcp(uint8_t ga, uint8_t gb) {
    matrix_row_t cols = 0;
    for (uint8_t c = 0; c < MATRIX_COLS; c++) {
        if (col_map[c].src == SRC_MCP_B) {
            if (!(gb & (1u << col_map[c].mcp_bit)))
                cols |= (MATRIX_ROW_SHIFTER << c);
        } else if (col_map[c].src == SRC_MCP_A) {
            if (!(ga & (1u << col_map[c].mcp_bit)))
                cols |= (MATRIX_ROW_SHIFTER << c);
        }
    }
    return cols;
}

static matrix_row_t cols_from_mcu(void) {
    matrix_row_t cols = 0;
    for (uint8_t c = 0; c < MATRIX_COLS; c++)
        if (col_map[c].src == SRC_MCU)
            if (!gpio_read_pin(col_map[c].mcu_pin))
                cols |= (MATRIX_ROW_SHIFTER << c);
    return cols;
}

/* ── matrix_init_custom ──────────────────────────────────────────────── */
void matrix_init_custom(void) {
    print("matrix_init_custom: START\n");

    for (uint8_t r = 0; r < MCU_ROWS; r++)
        gpio_set_pin_input_high(mcu_row_pins[r]);
    for (uint8_t c = 0; c < MATRIX_COLS; c++)
        if (col_map[c].src == SRC_MCU)
            gpio_set_pin_input_high(col_map[c].mcu_pin);

    gpio_set_pin_output(A15); gpio_write_pin_high(A15);
    gpio_set_pin_output(B3);  gpio_write_pin_high(B3);

    mcp_ready = init_mcp23017();
    xprintf("matrix_init_custom: DONE mcp_ready=%d\n", mcp_ready);
}

/* ── matrix_scan_custom ──────────────────────────────────────────────── */
static uint32_t scan_n = 0;

bool matrix_scan_custom(matrix_row_t current_matrix[]) {
    bool changed = false;
    bool verbose = (++scan_n % 2000 == 0);  /* print every ~2s */

    if (verbose)
        xprintf("scan #%lu mcp_ready=%d\n", (unsigned long)scan_n, mcp_ready);

    for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
        matrix_row_t cols = 0;

        if (row < MCU_ROWS) {
            gpio_set_pin_output(mcu_row_pins[row]);
            gpio_write_pin_low(mcu_row_pins[row]);
            wait_us(5);

            cols = cols_from_mcu();
            if (mcp_ready) {
                uint8_t ga = 0xFF, gb = 0xFF;
                if (mcp_read2(REG_GPIOA, &ga, &gb) == I2C_STATUS_SUCCESS)
                    cols |= cols_from_mcp(ga, gb);
                else if (verbose)
                    xprintf("  row%d: MCP read FAIL\n", row);
            }
            gpio_set_pin_input_high(mcu_row_pins[row]);

        } else {
            if (!mcp_ready) continue;
            uint8_t idx = row - MCU_ROWS;
            i2c_status_t ws = mcp_write(REG_OLATA, mcp_olata & ~mcp_row_mask[idx]);
            if (ws != I2C_STATUS_SUCCESS) {
                if (verbose) xprintf("  row%d: OLATA write FAIL %d\n", row, ws);
                continue;
            }
            uint8_t ga = 0xFF, gb = 0xFF;
            if (mcp_read2(REG_GPIOA, &ga, &gb) == I2C_STATUS_SUCCESS) {
                if (verbose) xprintf("  row%d: GA=0x%02X GB=0x%02X\n", row, ga, gb);
                cols = cols_from_mcp(ga, gb);
                cols |= cols_from_mcu();
            } else if (verbose) {
                xprintf("  row%d: GPIO read FAIL\n", row);
            }
            mcp_write(REG_OLATA, mcp_olata);
        }

        /* Log any active keys */
        if (cols && verbose)
            xprintf("  row%d cols=0x%06lX\n", row, (unsigned long)cols);

        if (current_matrix[row] != cols) {
            current_matrix[row] = cols;
            changed = true;
        }
    }
    return changed;
}
