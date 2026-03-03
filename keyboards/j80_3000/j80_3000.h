// Copyright 2025 Brotbeutel (@Brotbeutel)
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

/* MCP23017 I2C address: A2=A1=A0=GND → 0x20 */
#define MCP_I2C_ADDR    (0x20 << 1)
#define MCP_OLATA       0x14
#define MCP_TIMEOUT_MS  100
