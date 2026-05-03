// Copyright 2025 Brotbeutel (@Brotbeutel)
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once
#include_next <mcuconf.h>

/* Disable I2C — no longer used */
#undef  STM32_I2C_USE_I2C1
#define STM32_I2C_USE_I2C1 FALSE

/* Disable hardware SPI — using bit-bang SPI in matrix.c */
#undef  STM32_SPI_USE_SPI1
#define STM32_SPI_USE_SPI1 FALSE
