#include "quantum.h"
#include "i2c_master.h"
#include "j80_3000.h"

/* Defined in matrix.c */
extern bool    mcp_ready;
extern uint8_t mcp_olata;

static void mcp_write_kb(uint8_t reg, uint8_t val) {
    i2c_write_register(MCP_I2C_ADDR, reg, &val, 1, MCP_TIMEOUT_MS);
}

bool led_update_kb(led_t led_state) {
    /* Pins configured as output in matrix_init() — just write values */
    gpio_write_pin(A15, !led_state.num_lock);
    gpio_write_pin(B3,  !led_state.caps_lock);

    if (mcp_ready) {
        if (led_state.scroll_lock) mcp_olata &= ~(1 << 1);
        else                       mcp_olata |=  (1 << 1);
        mcp_write_kb(MCP_OLATA, mcp_olata);
    }
    return false;
}
