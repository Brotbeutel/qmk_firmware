# J80-3000

![j80_3000](imgur.com image replace me!)

This is my first keyboard project involving QMK programming.

This project aims to take old Cherry G80-3000's and make them QMK and VIA ready, by swapping the built-in microchip with a WeAct BlackPill STM32F411, reusing the old PCB and adding an Adafruit to allow for the big switch matrix. Also I will add a female USB-C port to the keyboard case with a 3D-printed cover. But first I have to figure out how to make the I2C Epander work.

My G80-3000 is an old version with a 102 key ISO layout, missing the Windows keys and the Menu key. Also it has an old DIN-connector.
For now I will focus on getting this prototype running and making it hot-swappable by using Millmax style sockets from RTLECS. Also I will add a 3D printed switch plate I found online. After that I might think about adding a 105 key ISO layout and Bluethooth and 2.4 GHz connectivity, or making a ATMega324U version.


* Keyboard Maintainer: [Brotbeutel](https://github.com/Brotbeutel)
* Hardware Supported: G80-3000 PCB, WeAct STM32F411 Blackpill, Adafruit MCP23017 I2C Expander
* Hardware Availability: 

    WeAct BlackPill STM32F411: https://de.aliexpress.com/item/1005001456186625.html<br>
    Adafruit MCP23017 I2C Expander: https://de.aliexpress.com/item/1005005596741592.html<br>
    RTLECS Hot Swap Sockets: https://de.aliexpress.com/item/1005009260905480.html<br>
    3D-Printable Plate: https://www.printables.com/model/607992-plate-for-1990-cherry-g80-3000-hao-wkl-keyboard/collections?lang=de

Make example for this keyboard (after setting up your build environment):

    make j80_3000:default

Flashing example for this keyboard:

    make j80_3000:default:flash

See the [build environment setup](https://docs.qmk.fm/#/getting_started_build_tools) and the [make instructions](https://docs.qmk.fm/#/getting_started_make_guide) for more information. Brand new to QMK? Start with our [Complete Newbs Guide](https://docs.qmk.fm/#/newbs).

## Bootloader

Enter the bootloader in 3 ways:

* **Bootmagic reset**: Hold down the key at (0,0) in the matrix (usually the top left key or Escape) and plug in the keyboard
* **Physical reset button**: Briefly press the button on the back of the PCB - some may have pads you must short instead
* **Keycode in layout**: Press the key mapped to `QK_BOOT` if it is available
