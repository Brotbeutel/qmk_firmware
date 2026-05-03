// Copyright 2025 Brotbeutel (@Brotbeutel)
// SPDX-License-Identifier: GPL-2.0-or-later

#pragma once

/* MCP23S17 SPI config
 * CS pin: PB6 (former I2C SCL, now free)
 * Opcode byte: 0100 A2 A1 A0 R/W
 * With A2=A1=A0=GND and HAEN=0 (default): write=0x40, read=0x41
 */
#define MCP_CS_PIN       B6
#define MCP_WRITE_OPCODE 0x40
#define MCP_READ_OPCODE  0x41
#define MCP_OLATA        0x14
#define MCP_SPI_DIVISOR  32   /* APB2=84MHz / 32 = 2.6MHz — conservative for F401 */
