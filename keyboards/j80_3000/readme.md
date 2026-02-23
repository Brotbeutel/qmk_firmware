# J80-3000

> QMK & VIA firmware for the Cherry G80-3000, powered by a WeAct STM32F411 BlackPill and an Adafruit MCP23017 I2C GPIO expander.

---

## Overview

This project converts a vintage **Cherry G80-3000** keyboard to run open-source QMK firmware with full VIA support. The original controller is replaced by a **WeAct STM32F411 BlackPill**, and an **Adafruit MCP23017 I2C expander** handles the additional matrix lines that the MCU alone cannot cover.

The target layout is a **102-key ISO** (no Windows/Menu keys, original DIN connector replaced with USB-C).

### Goals
- Full QMK + VIA compatibility
- Hot-swap sockets (Millmax-style from RTLECS)
- 3D-printed switch plate
- USB-C port
- Possible future extensions: Bluetooth / 2.4 GHz

---

## Hardware

| Component | Source |
|---|---|
| WeAct STM32F411 BlackPill | [AliExpress](https://de.aliexpress.com/item/1005001456186625.html) |
| Adafruit MCP23017 I2C Expander | [AliExpress](https://de.aliexpress.com/item/1005005596741592.html) |
| RTLECS Hot-Swap Sockets | [AliExpress](https://de.aliexpress.com/item/1005009260905480.html) |
| 3D-Printable Switch Plate | [Printables](https://www.printables.com/model/607992-plate-for-1990-cherry-g80-3000-hao-wkl-keyboard) |

### Wiring Summary

**MCU → Matrix**

| Pin | Function |
|---|---|
| PB5, PB6, PB7, PB8, PB9 | Rows 0–4 |
| PC14, PA3, PB0, PB1, PB10, PB12–PB15 | Cols 0, 1, 6, 7, 9, 13–16 |
| PA8 | I2C3 SCL → MCP23017 |
| PB4 | I2C3 SDA → MCP23017 |
| PA15 | NumLock LED |
| PB3 | CapsLock LED |

**MCP23017 → Matrix**

| MCP Pin | Function |
|---|---|
| GPA2, GPA3, GPA4 | Rows 5–7 |
| GPA0 | Col 17 |
| GPB0–GPB7 | Cols 2–5, 8, 10–12 |
| GPA1 | ScrollLock LED |

---

## Firmware

### Prerequisites

- [QMK Firmware](https://github.com/qmk/qmk_firmware)
- [QMK MSYS](https://msys.qmk.fm/) (Windows) or native Linux/macOS environment

### Setup

```bash
qmk setup
```

Place this keyboard folder at:
```
qmk_firmware/keyboards/j80_3000/
```

### Build

```bash
qmk compile -kb j80_3000 -km default
```

### Flash

Enter the bootloader first (see below), then:

```bash
qmk flash -kb j80_3000 -km default
```

Or use **STM32CubeProgrammer** with the generated `.bin` file for more reliable flashing.

---

## Bootloader

Enter the bootloader via one of these methods:

- **Bootmagic reset** — Hold the top-left key (matrix position 0,0) while plugging in USB
- **Physical reset** — Press the RESET button on the BlackPill
- **Keycode** — Press the key mapped to `QK_BOOT` if available in your keymap

---

## VIA Support

This keyboard supports [VIA](https://www.caniusevia.com/) for live keymap editing. Load the included `via.json` as a custom definition in VIA if the keyboard is not automatically detected.

---

## Project Status

- [x] USB enumeration working
- [x] MCU-direct keys functional
- [x] QMK + VIA compiling and flashing
- [ ] MCP23017 I2C key scanning (in progress)
- [ ] LED support (NumLock, CapsLock, ScrollLock)
- [ ] Hot-swap socket installation
- [ ] USB-C port integration
- [ ] 3D-printed plate

---

## Maintainer

[Brotbeutel](https://github.com/Brotbeutel)
