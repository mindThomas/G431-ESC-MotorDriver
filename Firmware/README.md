# ESC MotorDriver firmware for STM32G431

### Cloning the repository
If you have not already cloned this repository you do so by using the following command to include the tested acados version:
```
git clone --recursive https://github.com/mindThomas/acados-STM32.git
```

If you have already cloned it you need to run:
```
git submodule update --init --recursive
```

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
Just select the automatically generated OpenOCD configuration file when creating the __OpenOCD Download & Run__ configuration.

Alternatively VS Code can be used: https://medium.com/@lixis630/getting-started-to-code-embedded-c-on-stm32-e90e7910b2c

## CLion tricks
To show the content of an array of data given by its pointer, `char * data`, and its length, add the following cast to the watch or evaluation expression:
```
(char[8])*data
```

## FreeRTOS useful variables
List all tasks, needs a breakpoint after calling `uxTaskGetSystemState` (e.g. in `CPULoad.cpp`)
```
(TaskStatus_t[uxCurrentNumberOfTasks])*pxTaskStatusArray
```