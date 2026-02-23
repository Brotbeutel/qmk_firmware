#include "quantum.h"
#include "i2c_master.h"

/* Exported from matrix.c */
extern bool    mcp_ready;
extern uint8_t mcp_olata;

/* Mirror of matrix.c defines — keep in sync */
#define MCP_I2C_ADDR    (0x20 << 1)
#define REG_OLATA       0x14
#define MCP_TIMEOUT     20

static inline void mcp_write_kb(uint8_t reg, uint8_t val) {
    i2c_write_register(MCP_I2C_ADDR, reg, &val, 1, MCP_TIMEOUT);
}

bool led_update_kb(led_t led_state) {
    /*
     * MCU LEDs are active-LOW (common-cathode or open-drain drive).
     * Pins were set as output in matrix_init_custom().
     */
    gpio_write_pin(A15, !led_state.num_lock);
    gpio_write_pin(B3,  !led_state.caps_lock);

    if (mcp_ready) {
        /* ScrollLock LED on GPA1, active LOW.
         * mcp_olata tracks the desired OLATA state including row drive.
         * Only toggle bit1 here — do NOT alter row bits. */
        if (led_state.scroll_lock)
            mcp_olata &= ~(1u << 1);   /* GPA1 LOW → LED on */
        else
            mcp_olata |=  (1u << 1);   /* GPA1 HIGH → LED off */
        mcp_write_kb(REG_OLATA, mcp_olata);
    }

    /* Return true: we handled everything, suppress QMK default LED code. */
    return true;
}
