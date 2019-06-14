# Installatie

datum: Thursday, 13. June 2019 09:29PM 


## Firmware for ESP32 boards

The [following files](https://micropython.org/download#esp32) are daily firmware for ESP32-based boards, with separate firmware for boards with and without external SPIRAM. Non-SPIRAM firmware will work on any board, whereas SPIRAM enabled firmware will only work on boards with 4MiB of external pSRAM.

Program your board using the esptool.py program, found [here](https://github.com/espressif/esptool). If you are putting MicroPython on your board for the first time then you should first erase the entire flash using:

```
hestbv@LenovoY700 ~/Projecten/IoT/01_IoT-playground/esp32_wroom $ esptool.py --chip esp32 --port /dev/ttyUSB0 erase_flash
esptool.py v2.6
Serial port /dev/ttyUSB0
Connecting........_____....._____....._____.
Chip is ESP32D0WDQ6 (revision 1)
Features: WiFi, BT, Dual Core, 240MHz, VRef calibration in efuse, Coding Scheme None
MAC: 3c:71:bf:4c:a8:04
Uploading stub...
Running stub...
Stub running...
Erasing flash (this may take a while)...
Chip erase completed successfully in 7.6s
Hard resetting via RTS pin...
```

From then on program the firmware starting at address 0x1000:

```
hestbv@LenovoY700 ~/Projecten/IoT/01_IoT-playground/esp32_wroom $ esptool.py --chip esp32 --port /dev/ttyUSB0 --baud 460800 write_flash -z 0x1000 esp32-20190529-v1.11.bin
esptool.py v2.6
Serial port /dev/ttyUSB0
Connecting........___
Chip is ESP32D0WDQ6 (revision 1)
Features: WiFi, BT, Dual Core, 240MHz, VRef calibration in efuse, Coding Scheme None
MAC: 3c:71:bf:4c:a8:04
Uploading stub...
Running stub...
Stub running...
Changing baud rate to 460800
Changed.
Configuring flash size...
Auto-detected Flash size: 4MB
Compressed 1146864 bytes to 717504...
Wrote 1146864 bytes (717504 compressed) at 0x00001000 in 16.6 seconds (effective 552.2 kbit/s)...
Hash of data verified.

Leaving...
Hard resetting via RTS pin...
```

