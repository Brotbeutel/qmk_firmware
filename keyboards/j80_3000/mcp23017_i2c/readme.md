# J80-3000

> QMK & VIA firmware for the Cherry G80-3000 — powered by a WeAct STM32F411 BlackPill and an Adafruit MCP23017 I2C GPIO expander.

This project converts a vintage **Cherry G80-3000** keyboard to open-source QMK firmware with full VIA support. The original PS/2 controller is replaced by a **WeAct STM32F411 BlackPill**, and an **Adafruit MCP23017 I2C expander** handles the additional matrix lines the MCU alone cannot cover. The result is a fully programmable 105-key ISO keyboard with USB-C and live keymap editing via VIA.

---

## Table of Contents

- [Hardware](#hardware)
- [Disassembly](#disassembly)
- [Wiring](#wiring)
- [Firmware](#firmware)
- [Flashing](#flashing)
- [VIA](#via)
- [Project Goals](#project-goals)
- [Known Limitations](#known-limitations)

---

## Hardware

### Bill of Materials

| Component                    | Link                                                                                    | Notes                                                                                                                                         |
|------------------------------|-----------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------|
| Cherry G80-3000              | —                                                                                       | Used: **G80-3000 LSMDE-0** (601-1966 REV.:01 S01-5214). Other variants may have different matrices — see compatibility warning in Disassembly. |
| WeAct STM32F411 BlackPill    | [AliExpress](https://de.aliexpress.com/item/1005001456186625.html)                      | STM32F411CEU6. Order the **25M HSE** variant. Verify crystal marking reads **25000**. The 8M HSE variant requires manual clock configuration.  |
| Adafruit MCP23017 I2C Expander | [AliExpress](https://de.aliexpress.com/item/1005005596741592.html)                    | Breakout board with onboard 10 kΩ pull-ups on SDA/SCL — no external pull-ups needed.                                                         |
| Hot-Swap Sockets             | [AliExpress](https://de.aliexpress.com/item/1005009260905480.html)                      | Optional. Millmax-style, MX-compatible.                                                                                                       |
| 3D-Printable Switch Plate    | [Printables](https://www.printables.com/model/607992-plate-for-1990-cherry-g80-3000-hao-wkl-keyboard) | Optional. Designed for the 1990-era G80-3000 HAO/WKL variant. WIP.                                                          |
| Wire                         | —                                                                                       | 28–30 AWG stranded recommended for internal routing.                                                                                          |

### Tools

- Soldering iron with fine tip
- Desoldering pump
- Multimeter (continuity mode for verifying connections)
- Flush cutters / wire strippers

---

## Disassembly

1. Remove all keycaps.
2. The G80-3000 case is held together by **plastic clips** — there are no screws. Use your fingers, if they are thin, or a piece of wood, or plastic to open the clips.
3. Lift out the PCB.
4. Using a Multimeter, find out which pin GND and which pin VCC is on the 5 PCB pins of the of the PS/2 connector cable and write it down.
5. Desolder or cut the cable.
6. Mark down the pin numbers of the Motorola chip.
7. Desolder and remove the Motorola chip, using a soldering pump and maybe soldering wick or even a heatgun.

> **⚠️ Compatibility warning:** The G80-3000 was produced in a large number of variants with different part numbers (e.g. LSMDE, HAAUS, LPCUS, ...). These variants can have **completely different matrix layouts**. The wiring and firmware in this repository were developed for and verified on a **G80-3000 LSMDE-0 601-1966 | REV.: 01 | S01-5214** (ISO-DE). If your board has a different part number, do not assume the pinout is the same — map the matrix yourself with a multimeter before soldering anything.

---

## Wiring

The BlackPill does not have enough GPIOs to drive the full 18-column × 8-row matrix on its own. The MCP23017 handles 9 columns and 3 rows via I2C, while the BlackPill drives the remaining 9 columns and 5 rows directly.

### Power & I2C

Connect these first before doing anything else.

| BlackPill | MCP23017 | Notes                               |
|-----------|----------|-------------------------------------|
| 3V3       | VIN      | Powers the MCP23017                 |
| GND       | GND      | Common ground                       |
| PB6       | SCL      | I2C clock                           |
| PB7       | SDA      | I2C data                            |
| 3V3       | RST      | Must be HIGH — wire directly to 3V3 |

> The Adafruit MCP23017 board has onboard 10 kΩ pull-ups on SDA and SCL. No additional pull-up resistors are needed. The address pins (IA/IB) are pulled to GND by default, giving I2C address **0x20**.

Also connect power to the keyboard PCB itself:

| BlackPill | PCB Pin       | Notes           |
|-----------|---------------|-----------------|
| 5V        | Pin 4 (VCC1)  | PCB logic power |
| 5V        | Pin 7 (VCC2)  | PCB logic power |
| GND       | Pin 1 (GND1)  |                 |
| GND       | Pin 8 (GND2)  |                 |

### Columns — BlackPill → PCB

| Matrix Col | BlackPill Pin | PCB Pin |
|------------|---------------|---------|
| COL 0      | PC14          | Pin 3   |
| COL 1      | PA3           | Pin 9   |
| COL 6      | PB0           | Pin 14  |
| COL 7      | PB1           | Pin 15  |
| COL 9      | PB10          | Pin 17  |
| COL 13     | PB12          | Pin 21  |
| COL 14     | PB13          | Pin 22  |
| COL 15     | PB14          | Pin 23  |
| COL 16     | PB15          | Pin 24  |

### Columns — MCP23017 → PCB

| Matrix Col | MCP23017 Pin | PCB Pin |
|------------|--------------|---------|
| COL 2      | GPB0         | Pin 10  |
| COL 3      | GPB1         | Pin 11  |
| COL 4      | GPB2         | Pin 12  |
| COL 5      | GPB3         | Pin 13  |
| COL 8      | GPB4         | Pin 16  |
| COL 10     | GPB5         | Pin 18  |
| COL 11     | GPB6         | Pin 19  |
| COL 12     | GPB7         | Pin 20  |
| COL 17     | GPA0         | Pin 29  |

### Rows — BlackPill → PCB

| Matrix Row | BlackPill Pin | PCB Pin |
|------------|---------------|---------|
| ROW 0      | PB5           | Pin 33  |
| ROW 1      | PA1           | Pin 34  |
| ROW 2      | PA0           | Pin 35  |
| ROW 3      | PB8           | Pin 36  |
| ROW 4      | PB9           | Pin 37  |

### Rows — MCP23017 → PCB

| Matrix Row   | MCP23017 Pin | PCB Pin |
|--------------|--------------|---------|
| ROW 5        | GPA2         | Pin 38  |
| ROW 6        | GPA3         | Pin 39  |
| ROW 7        | GPA4         | Pin 40  |

### LEDs

The indicator LEDs are driven **active-high** (output HIGH = LED on).

| LED        | Source    | Pin  | PCB Pin |
|------------|-----------|------|---------|
| NumLock    | BlackPill | PA15 | Pin 30  |
| CapsLock   | BlackPill | PB3  | Pin 31  |
| ScrollLock | BlackPill | PA2  | Pin 32  |

> LED firmware support is implemented and LED hardware wiring to the PCB is complete.

---

## Firmware

### Prerequisites

- [QMK Firmware](https://github.com/qmk/qmk_firmware) — follow the [official setup guide](https://docs.qmk.fm/newbs_getting_started)
- [QMK MSYS](https://msys.qmk.fm/) on Windows, or a native Linux/macOS terminal

### Setup

```bash
qmk setup
```

Clone or copy this keyboard folder into your QMK installation:

```
qmk_firmware/keyboards/j80_3000/
```

### Build

```bash
qmk compile -kb j80_3000 -km default
```

### Flash

Put the BlackPill into bootloader mode first (see [Bootloader](#bootloader)), then:

```bash
qmk flash -kb j80_3000 -km default
```

Alternatively, use **STM32CubeProgrammer** with the generated `.bin` file for more reliable flashing over USB DFU.

---

## Bootloader

The BlackPill uses the STM32 USB DFU bootloader. Enter it via one of:

| Method        | Instructions                            |
|---------------|-----------------------------------------|
| **Bootmagic** | Hold **RightWin** while plugging in USB |
| **Physical**  | Hold BOOT0, tap RESET, release BOOT0    |
| **Keycode**   | Press `QK_BOOT` → FN + F12 by default   |

---

## VIA

This keyboard supports [VIA](https://www.caniusevia.com/) for live keymap editing without reflashing.

1. Open [usevia.app](https://usevia.app) in a Chromium-based browser.
2. If the keyboard is not detected automatically, go to **Settings → Load Draft Definition** and load the included `via.json`.
3. The keyboard supports **3 fully remappable layers**.

---

## Project Goals

- [x] USB enumeration
- [x] Full 105-key matrix scanning (MCU + MCP23017)
- [x] QMK + VIA support
- [x] LED firmware support (NumLock, CapsLock, ScrollLock)
- [x] LED hardware wiring to PCB
- [x] Hot-swap socket installation
- [ ] Anti-Ghosting Diodes installed
- [ ] N-key rollover activated
- [ ] USB-C port integration
- [ ] 3D-printed switch plate **WIP**

---

## Known Limitations

**VIA Test Matrix mode** does not register keypresses. This is a known VIA-side limitation when `CUSTOM_MATRIX = yes` is used in QMK — VIA's raw matrix polling is incompatible with custom matrix implementations. The standard VIA key tester (without Test Matrix enabled) works correctly.

---

## Maintainer

[Brotbeutel](https://github.com/Brotbeutel)
