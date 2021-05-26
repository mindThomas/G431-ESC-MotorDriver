# Tools for STM32G431 ESC

## USB to UART not working after connecting

It appears that the USB to UART built into the ST-Link firmware is flawed right after connecting it to a PC (before starting any debug session). It appears that only TX data (from STM32 to PC) is transmitted while any data sent from the PC is never transmitted on the UART lines to the STM32 DUT (device under test).

For now the solution is to reset the debugger by either manually initiating a debug session and running or alternatively by using the `make reset_firmware` target within the Firmware build directory.