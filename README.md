# stm32f4-bare-metal

Bare metal STM32F4 examples to serve as starting points for projects. Educational purposes.

Some of the STM32F4xx family based processor headers are added in the `include` folder to get register locations. CMSIS library is added for a general support. No extra HAL libraries is used except the selected projects described below.

Common startup functions are located in the `include/startup_stm32f407vgtx.s` and `include/system_stm32f4xx.c` files to be included in all projects.

## Installation

First, clone the project using `git clone --recurse-submodules https://github.com/fcayci/stm32f4-bare-metal`. The repository includes other repositories such as `CMSIS` as submodules under `lib` folder. You can alternatively clone the repository, then initialize and update the submodules with `git submodule init` and `git submodule update` commands.

## Development

There are two options for development. First one is to use [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html) from ST. Second one is the setup your own development environment. Both options are supported with relevant project settings or makefiles.

### Option 1 - STM32CubeIDE

- Download and install [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html). Select workspace on the root folder, then import existing projects to workspace. (File -> Import -> General -> Existing Projects into Workspace)
- You do not need any additional tools. It comes with the compiler and debugger pre-installed.
- Rest of the sections are for Option 2.

### Option 2 - Custom development environment

- Get **toolchain** (for compiler and binutils) from [GNU Arm Embedded Toolchain](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads)
- *For windows only*, install **make** tool. You can just get the make tool from [gnuwin32](http://gnuwin32.sourceforge.net/packages/make.htm). Alternatively you can install the minimalist GNU tools for windows from [mingw](https://mingw-w64.org/) and [MSYS](https://www.msys2.org/)
- For the **programmer/debugger**, you can use - [stlink](https://github.com/texane/stlink) or [OpenOCD](http://openocd.org/). Though only stlink utility support is added.
- You can use your favorite code editor to view/edit the contents. Here is an open source one: [Visual Studio Code](https://code.visualstudio.com/).

#### Compile

[makefile](projects/armf4.mk) contains necessary build scripts and compiler flags for all projects. Individual settings can be changed from local makefiles from projects such as [blinky makefile](projects/blinky/makefile)

Browse into any directory and run `make` to compile.

```
cd projects/blinky
make
```

If everything is setup correctly, you should see the completed message.
```
Cleaning blinky
Building blinky.c
   text    data     bss     dec     hex filename
    852	      8	   1568	   2428	    97c	blinky.elf
Successfully finished...
```

If you see any errors about command not found, make sure the toolchain binaries are in your `PATH`. On Windows check the *Environment Variables* for your account. On Linux/macOS run `echo $PATH` to verify your installation.

#### Program

Run `make burn` to program the chip.
```
...
.. Flash written and verified! jolly good!
```

Install the [ST LINK](https://www.st.com/en/development-tools/st-link-v2.html) drivers if you cannot see your board when `make burn` is run.

#### Disassemble

Run `make disass` / `make disass-all` to disassamble.

#### Debug

In order to debug your code, connect your board to the PC, run `st-util` (comes with stlink utility) from one terminal, and from another terminal within the project directory run `make debug`. You can then use general **gdb** commands to browse through the code.

## Projects

* [blinky](projects/blinky/) - Good old blink LEDs example
* [clock](projects/clock/) - Shows how to change clock frequencies on the fly
* [math](projects/math/) - A simple sine function to test math library operation
* [systick](projects/systick/) - Blinks LEDs using systick timer.
* [timer](projects/timer/) - Blinks LEDs one at a time using the Timer module and Timer interrupt
* [pwm](projects/pwm/) - Fades an LED using pwm functionality using Timer module
* [external](projects/external/) - External interrupt example using the on-board push-button
* [dac](projects/dac/) - On-chip digital to analog converter operation
* [dac-timer](projects/dac-timer/) - On-chip digital to analog converter operation with timer trigger
* [uart](projects/uart/) - UART example to show how to send data over
* [uart-tx-int](projects/uart-tx-int/) - UART example with tx interrupt
* [uart-dma](projects/uart-dma/) - UART tx example with DMA transfer
* [spi](projects/spi/) - SPI example that is customized for on-board motion sensor (lis302dl version)
* [i2c](projects/i2c/) - I2C example that communicates with the on-board Audio DAC
* [wwdg](projects/wwdg/) - Window Watchdog example
* [itm](projects/itm/) - Message sending through CoreSight ITM port 0
* [dma](projects/dma/) - Example DMA transfer using memory-to-memory mode
* [sleepy](projects/sleepy/) - Low Power mode operations (sleep and stop)
* [flash](projects/flash/) - Example to show how to write/erase internal flash memory
* [i2s-beep](projects/i2s-beep/) - I2S example that plays a couple notes using the internal beep generator from on-board Audio DAC through audio jack. It does not send any I2S data.
* [pendsv](projects/pendsv) - Example project on a non-OS use of PendSV exception
