// Copyright 2025 Brotbeutel (@Brotbeutel)
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once
#include_next <mcuconf.h>

/* Disable I2C — replaced by bit-bang SPI for MCP23S17 */
#undef  STM32_I2C_USE_I2C1
#define STM32_I2C_USE_I2C1 FALSE
