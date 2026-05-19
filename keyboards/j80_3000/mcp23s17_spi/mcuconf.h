// Copyright 2025 Brotbeutel (@Brotbeutel)
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once
#include_next <mcuconf.h>

/* Disable I2C — not used */
#undef  STM32_I2C_USE_I2C1
#define STM32_I2C_USE_I2C1 FALSE

/* Enable hardware SPI1 for MCP23S17 */
#undef  STM32_SPI_USE_SPI1
#define STM32_SPI_USE_SPI1 TRUE

/*
 * Software-managed CS (we drive PB6 manually).
 * SPI_SELECT_MODE_NONE tells ChibiOS not to touch the NSS pin at all,
 * preventing any conflict with PA4 (onboard W25Q64 flash CS = SPI1 NSS).
 */
#undef  SPI_SELECT_MODE
#define SPI_SELECT_MODE SPI_SELECT_MODE_NONE
