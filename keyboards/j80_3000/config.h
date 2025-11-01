#pragma once


/* KEY MATRIX SIZE */
#define MATRIX_ROWS 8
#define MATRIX_COLS 19

#define MATRIX_ROWS_PER_SIDE (MATRIX_ROWS / 2)
#define MATRIX_COLS_PER_SIDE (MATRIX_COLS / 2)

/*
#define UNUSED_MCU
#define UNUSED_MCP
*/

/* WIRING */

#define MATRIX_ROW_PINS_MCU \
    { B5, B6, B7 }
#define MATRIX_COL_PINS_MCU \
    { A3, A4, A5, A6, A7, B0, B1, B10, B12, B15, A8, A13, A14 }
/*
#define UNUSED_PINS_MCU \
    { }
*/
#define MATRIX_ROW_PINS_MCP \
    { A0, A1, A2, A3, A4 }
#define MATRIX_COL_PINS_MCP\
    { B0, B1, B2, B3, B4, B5 }
/*
#define UNUSED_PINS_MCP \
    { }
*/

#define MATRIX_ROW_PINS \
    { B5, B6, B7, B8, B8, B8, B8, B8 }
#define MATRIX_COL_PINS \
    { A3, A4, A5, A6, A7, B0, B1, B9, B10, B9, B9, B9, B12, B15, A8, B9, B9, A13, A14}
/*
#define UNUSED_PINS \
   { }
*/


/* MCU pins */
//static const pin_t MATRIX_ROW_PINS_MCU[] = { B5, B6, B7 };
//static const pin_t MATRIX_COL_PINS_MCU[] = { A3, A4, A5, A6, A7, B0, B1, B10, B12, B15, A8, A13, A14 };

/* MCP23017 pins */
//static const uint8_t MATRIX_ROW_PINS_MCP[] = { A0, A1, A2, A3, A4 };
//static const uint8_t MATRIX_COL_PINS_MCP[] = { B0, B1, B2, B3, B4, B5 };

/* Matrix Configuration*/
#define DIODE_DIRECTION COL2ROW

/* define if matrix hast ghost (lacks anti-ghosting diodes) */
#define MATRIX_HAS_GHOST

/* Debounce reduces chatter (unintended double-presses) - set 0 if debouncing is not needed */
#define DEBOUNCE 5

/* I2C SETTINGS */
#define I2C_DRIVER I2CD2
#define I2C1_SCL_PIN B8     // I2C Pin 1
#define I2C1_SDA_PIN B9     // I2C Pin 2
#define I2C1_TIMINGR_PRESC 2U
#define I2C1_TIMINGR_SCLDEL 1U
#define I2C1_TIMINGR_SDADEL 0U
#define I2C1_TIMINGR_SCLH 9U
#define I2C1_TIMINGR_SCLL 26U
#define I2C1_SCL_PAL_MODE 1
#define I2C1_SDA_PAL_MODE 1
