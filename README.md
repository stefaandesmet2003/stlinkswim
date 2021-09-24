# ST-LINK SWIM interface

- a basic implementation of the STM8 SWIM protocol running on STM32 (Bluepill or other)  
- On the USB side the code behaves like a ST-LINK v2  
- It is also pin compatible with ST-LINK v2 and the chinese clones  
- With this code you can turn a Bluepill into a SWIM probe for flashing & debugging a STM8. 
- Or you could flash it into a ST-LINK directly, and enjoy the fact that it now runs on open source firmware.
- The setup has been tested with stm8flash & openocd  

## Connections
- RST : PB6
- SWIM_IN : PB7
- SWIM_OUT : PB11
- add resistors as in schematic below (similar to STLINK)

![](swim.jpg)

## Build
use platformio for building & flashing to a STM32 target.

## Note
- the SWIM signal generation is based on code from the impressive but unfortunately defunct versaloon project.
- I don't have the ST-LINK USB protocol specification, so I guessed here and there, based on other code bases (stm8flash, openocd, versaloon).

## TODO
- add target voltage sense
- USB DFU version compatible with STLINK bootloader
- vague plans to integrate STM8 with black magic probe, if I find the time ..
