# ESC MotorDriver firmware for STM32G431

## ARM Toolchain
This project requires [CMake](https://cmake.org/download/) 3.15 or later and the [arm-none-eabi-gcc toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads) to be installed seperately.
This project was recently tested and used with the [GNU Arm Embedded Toolchain Version 7-2018-q2-update](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads/7-2018-q2-update).
OpenOCD is used for flashing and debugging. The project has been tested with [xpack-openocd-0.11.0](https://xpack.github.io/blog/2021/03/15/openocd-v0-11-0-1-released/) on Ubuntu 18.04.

Add/set the path to your ARM toolchain folder (which contains the executable `arm-none-eabi-gcc` and more) in your `.bashrc`:
```
export STM32_TOOLCHAIN_PATH=/path/to/gcc-arm-none-eabi-xxx
export ARM_TOOLCHAIN_DIR=$STM32_TOOLCHAIN_PATH/bin
export PATH=$PATH:/path/to/xpack-openocd-0.11.0-1/bin
```

## IDE
Please note that [CLion](https://www.jetbrains.com/clion/) can be used to load the project and comes bundled with OpenOCD: https://www.jetbrains.com/help/clion/openocd-support.html
Just select the automatically generated OpenOCD configuration file when creating the __OpenOCD Download & Run__ configuration. Remember also to set:
 - `Download` --> `Updated Only`
 - `Reset` --> `Halt`   _(this appears to be highly important, otherwise registers etc. is not reset/cleared after starting a new debug and flashing)_

Alternatively VS Code can be used: https://medium.com/@lixis630/getting-started-to-code-embedded-c-on-stm32-e90e7910b2c

## CLion tricks
To show the content of an array of data given by its pointer, `char * data`, and its length, add the following cast to the watch or evaluation expression:
```
(char[8])*data
```

## Debugging in CLion
Debugging with an ST-Link can be done with CLion in 3 different ways:
1. OpenOCD using bundled OpenOCD (simplest): https://www.jetbrains.com/help/clion/openocd-support.html
2. Using open source `st-util` GDB server: https://github.com/stlink-org/stlink
3. Using ST-LINK GDB server: https://nicolai86.eu/blog/2020/08/on-chip-debugging-stm32/

### Bundled OpenOCD
This is by far the easiest option as it just requires you to create an `OpenOCD Download & Run` debug configuration in CLion and configure it as follows:
1. Set the target and executable to your firmware
2. Set the GDB to `Bundled GDB`
3. Set the Download executable to `Always`.
4. Select the auto-generate OpenOCD Board config file
5. Set Download to `Updated Only`
6. Set Reset to `Halt`

### Open Source `st-util` GDB server
Alternatively the open source `st-util` can be used as GDB server. To do so you should create an `Embedded GDB Server` debug configuration in CLion instead.

Clone the `stlink` repo from and build it by doing:
```
git clone https://github.com/stlink-org/stlink
cd stlink
git checkout develop
make clean
make release
```
Now you will have the `st-util` in `stlink/build/Release/bin`.

In the CLion debug config do the following:
1. Set the GDB to `Bundled GDB`
2. Set the Download executable to `Always`.
3. Set the GDB Server to the `st-util` binary
4. Set the `target remote` args to: `localhost:4242`

Hint: If you are struggling with breakpoints not getting hit make sure that you don't have too many (max 6 breakpoints) and make sure that your firmware was flashed. If your debug configuration is set to `Download executable: Updated Only` and the upload fails, CLion might think that the firmware has been uploaded and whenever you start a new debug session it won't reupload. So a general advice is to set the Download executable to `Always`.

### ST-LINK GDB server
This requires an existing installation of STM32CubeIDE.
Find the installation folder and find the `stlink-gdb-server` subfolder under plugins. Note this as `GDB_SERVER_PATH`.
Next find the `cubeprogrammer` subfolder also under plugins. Note this as `CUBE_PROGRAMMER_PATH`.
Create a GDB server launch script as `stlink_gdb_launch.sh`:
```
#!/bin/sh
GDB_SERVER_PATH=~/st/stm32cubeide_1.6.0/plugins/com.st.stm32cube.ide.mcu.externaltools.stlink-gdb-server.linux64_1.6.0.202101291314/tools/bin
CUBE_PROGRAMMER_PATH=~/st/stm32cubeide_1.6.0/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.linux64_1.6.0.202101291314/tools/bin

PATH=$PATH:$GDB_SERVER_PATH
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$GDB_SERVER_PATH/native/linux_x64/

ST-LINK_gdbserver -e -f debug.log -p 61234 -r 15 -d -cp $CUBE_PROGRAMMER_PATH
```
Finally create an `Embedded GDB Server` configuration in CLion where you:
1. Use `Bundled GDB`
2. Set the Download executable to `Always`.
3. Set `target remote` args to: `localhost:61234`
4. Set the GDB Server to the `stlink_gdb_launch.sh` script

### Upgrade ST-Link firmware
This requires an existing installation of STM32CubeIDE.
Find the installation folder and find the `stlink-gdb-server` subfolder which was noted as `GDB_SERVER_PATH` above. Now launch the upgrade tool with:

```
java -jar $GDB_SERVER_PATH/STLinkUpgrade.jar
```



## FreeRTOS useful variables
List all tasks, needs a breakpoint after calling `uxTaskGetSystemState` (e.g. in `CPULoad.cpp`)
```
(TaskStatus_t[uxCurrentNumberOfTasks])*pxTaskStatusArray
```

List all registered Queues (including semaphores):
```
xQueueRegistry
```