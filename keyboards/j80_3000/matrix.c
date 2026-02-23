#include "matrix.h"
#include "wait.h"
#include "i2c_master.h"
#include "gpio.h"
#include "debounce.h"
#include "print.h"

#define I2C_ADDR         (0x20 << 1)
#define MCP_IODIRA       0x00
#define MCP_IODIRB       0x01
#define MCP_GPPUA        0x0C
#define MCP_GPPUB        0x0D
#define MCP_OLATA        0x14
#define MCP_GPIOA        0x12
#define MCP_GPIOB        0x13
#define MCP_TIMEOUT      100

#define IODIRA_VALUE     0xE1
#define IODIRB_VALUE     0xFF
#define GPPUA_VALUE      0xE1
#define GPPUB_VALUE      0xFF
#define OLATA_IDLE       0xFF

typedef enum { SRC_MCU, SRC_MCP_A, SRC_MCP_B } col_source_t;

typedef struct {
    col_source_t src;
    union {
        pin_t   mcu_pin;
        uint8_t mcp_bit;
    };
} col_def_t;

static const col_def_t col_map[MATRIX_COLS] = {
    [0]  = { SRC_MCU,   .mcu_pin = C14 },
    [1]  = { SRC_MCU,   .mcu_pin = A3  },
    [2]  = { SRC_MCP_B, .mcp_bit = 0   },
    [3]  = { SRC_MCP_B, .mcp_bit = 1   },
    [4]  = { SRC_MCP_B, .mcp_bit = 2   },
    [5]  = { SRC_MCP_B, .mcp_bit = 3   },
    [6]  = { SRC_MCU,   .mcu_pin = B0  },
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

static const pin_t mcu_row_pins[] = { B5, B6, B7, B8, B9 };
#define MCU_ROW_COUNT 5
static const uint8_t mcp_row_bits[] = { (1<<2), (1<<3), (1<<4) };
#define MCP_ROW_COUNT 3

static bool    i2c_initialized = false;
bool    mcp_ready       = false;
uint8_t mcp_olata       = OLATA_IDLE;
static uint8_t mcp_gpioa_cache = 0xFF;
static uint8_t mcp_gpiob_cache = 0xFF;
static bool    mcp_cache_valid = false;

static matrix_row_t raw_matrix[MATRIX_ROWS];
static matrix_row_t matrix_data[MATRIX_ROWS];

static bool mcp_write(uint8_t reg, uint8_t val) {
    return i2c_write_register(I2C_ADDR, reg, &val, 1, MCP_TIMEOUT) == I2C_STATUS_SUCCESS;
}

static bool mcp_read_reg(uint8_t reg, uint8_t *val) {
    return i2c_read_register(I2C_ADDR, reg, val, 1, MCP_TIMEOUT) == I2C_STATUS_SUCCESS;
}

static void i2c_bus_recover(void) {
    /* Manually clock SCL 9 times to free stuck SDA (I2C bus recovery) */
    /* SCL=PA8, SDA=PB4 — configure as GPIO temporarily */
    gpio_set_pin_output(A8);
    gpio_set_pin_input_high(B4);
    for (uint8_t i = 0; i < 9; i++) {
        gpio_write_pin_low(A8);
        wait_us(5);
        gpio_write_pin_high(A8);
        wait_us(5);
        if (gpio_read_pin(B4)) break; /* SDA released */
    }
    /* Generate STOP condition: SDA low->high while SCL high */
    gpio_set_pin_output(B4);
    gpio_write_pin_low(B4);
    wait_us(5);
    gpio_write_pin_high(A8);
    wait_us(5);
    gpio_write_pin_high(B4);
    wait_us(5);
    /* Release pins for I2C peripheral */
    gpio_set_pin_input_high(A8);
    gpio_set_pin_input_high(B4);
    wait_ms(10);
}

static bool init_mcp23017(void) {
    if (!i2c_initialized) {
        i2c_bus_recover();
        i2c_init();
        i2c_initialized = true;
        wait_ms(100);
    }
    i2c_status_t r;
    r = i2c_write_register(I2C_ADDR, MCP_IODIRA, &(uint8_t){IODIRA_VALUE}, 1, MCP_TIMEOUT);
    uprintf("IODIRA:%d\n", r); if (r) return false;
    r = i2c_write_register(I2C_ADDR, MCP_IODIRB, &(uint8_t){IODIRB_VALUE}, 1, MCP_TIMEOUT);
    uprintf("IODIRB:%d\n", r); if (r) return false;
    r = i2c_write_register(I2C_ADDR, MCP_GPPUA,  &(uint8_t){GPPUA_VALUE},  1, MCP_TIMEOUT);
    uprintf("GPPUA:%d\n",  r); if (r) return false;
    r = i2c_write_register(I2C_ADDR, MCP_GPPUB,  &(uint8_t){GPPUB_VALUE},  1, MCP_TIMEOUT);
    uprintf("GPPUB:%d\n",  r); if (r) return false;
    mcp_olata = OLATA_IDLE;
    r = i2c_write_register(I2C_ADDR, MCP_OLATA, &mcp_olata, 1, MCP_TIMEOUT);
    uprintf("OLATA:%d\n",  r); if (r) return false;
    return true;
}

static void select_row(uint8_t row) {
    if (row < MCU_ROW_COUNT) {
        gpio_set_pin_output(mcu_row_pins[row]);
        gpio_write_pin_low(mcu_row_pins[row]);
    } else {
        uint8_t idx = row - MCU_ROW_COUNT;
        if (idx < MCP_ROW_COUNT && mcp_ready)
            mcp_write(MCP_OLATA, mcp_olata & ~mcp_row_bits[idx]);
    }
}

static void unselect_row(uint8_t row) {
    if (row < MCU_ROW_COUNT)
        gpio_set_pin_input_high(mcu_row_pins[row]);
    else if (mcp_ready)
        mcp_write(MCP_OLATA, mcp_olata);
}

static void refresh_mcp_cache(void) {
    mcp_cache_valid = mcp_read_reg(MCP_GPIOB, &mcp_gpiob_cache)
                   && mcp_read_reg(MCP_GPIOA, &mcp_gpioa_cache);
}

static matrix_row_t read_cols(void) {
    matrix_row_t value = 0;
    for (uint8_t c = 0; c < MATRIX_COLS; c++) {
        bool pressed = false;
        switch (col_map[c].src) {
            case SRC_MCU:
                pressed = !gpio_read_pin(col_map[c].mcu_pin);
                break;
            case SRC_MCP_B:
                if (mcp_cache_valid)
                    pressed = !(mcp_gpiob_cache & (1 << col_map[c].mcp_bit));
                break;
            case SRC_MCP_A:
                if (mcp_cache_valid)
                    pressed = !(mcp_gpioa_cache & (1 << col_map[c].mcp_bit));
                break;
        }
        if (pressed) value |= (MATRIX_ROW_SHIFTER << c);
    }
    return value;
}

void matrix_init(void) {
    for (uint8_t r = 0; r < MCU_ROW_COUNT; r++)
        gpio_set_pin_input_high(mcu_row_pins[r]);
    for (uint8_t c = 0; c < MATRIX_COLS; c++)
        if (col_map[c].src == SRC_MCU)
            gpio_set_pin_input_high(col_map[c].mcu_pin);

    gpio_set_pin_output(A15); gpio_write_pin_high(A15);
    gpio_set_pin_output(B3);  gpio_write_pin_high(B3);

    mcp_ready = init_mcp23017();
    debounce_init();
}

uint8_t matrix_scan(void) {
    bool changed = false;

    static uint32_t scan_count = 0;
    if (++scan_count % 100000 == 0) {
        uprintf("scan#%lu rdy=%d\n", scan_count, mcp_ready);
        if (!mcp_ready) {
            i2c_initialized = false; /* force bus recovery on retry */
            mcp_ready = init_mcp23017();
        }
    }

    for (uint8_t row = 0; row < MATRIX_ROWS; row++) {
        select_row(row);
        wait_us(30);
        if (mcp_ready) refresh_mcp_cache();
        matrix_row_t row_val = read_cols();
        unselect_row(row);

        if (raw_matrix[row] != row_val) {
            raw_matrix[row] = row_val;
            changed = true;
        }
    }


    changed = debounce(raw_matrix, matrix_data, changed);
    return changed;
}

matrix_row_t matrix_get_row(uint8_t row) {
    return matrix_data[row];
}

void matrix_print(void) {}
