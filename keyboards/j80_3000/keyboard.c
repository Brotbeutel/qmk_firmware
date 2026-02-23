#include "quantum.h"
#include "i2c_master.h"
#include "ch.h"
#include "hal.h"

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

/* Force PA8 and PB4 into AF4 Open-Drain for I2C3 */
void keyboard_post_init_kb(void) {

    palSetPadMode(GPIOA, 8,
        PAL_MODE_ALTERNATE(4) |
        PAL_STM32_OTYPE_OPENDRAIN
    );

    palSetPadMode(GPIOB, 4,
        PAL_MODE_ALTERNATE(4) |
        PAL_STM32_OTYPE_OPENDRAIN
    );

    keyboard_post_init_user();
}

bool led_update_kb(led_t led_state) {

    gpio_write_pin(A15, !led_state.num_lock);
    gpio_write_pin(B3,  !led_state.caps_lock);

    if (mcp_ready) {
        if (led_state.scroll_lock)
            mcp_olata &= ~(1u << 1);
        else
            mcp_olata |=  (1u << 1);

        mcp_write_kb(REG_OLATA, mcp_olata);
    }

    return true;
}
