# Introduction
This folder contains the software for the ESP32-side of the spider launcher.

On a PC set up with the [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/index.html) software environment, compile this software with the default values and download it to the ESP32 in the spider launcher using a USB cable.  If you have chosen to wire up different GPIO pins on the ESP32 to the ones given in the schematic, change those values in [Kconfig.projbuild](main/Kconfig.projbuild) before compiling.
