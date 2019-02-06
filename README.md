# stm32f4-bare-metal

Bare metal STM32F4 projects.

Some of the STM32F4xx family based processor headers are added in the `include` folder to get 
register locations. CMSIS library is added for a general support. No extra HAL libraries is used except the selected projects described below.

Common startup functions are moved to `include/system_stm32f4xx.c` to include in all projects.

## Installation

* Clone the project using `git clone --recurse-submodules https://github.com/fcayci/stm32f4-bare-metal`
with the included external tools/libraries such as CMSIS repo and any additional libraries some of the projects use.
* Install a compiler. You can get one free from [GNU ARM Embedded Toolchain](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads)
* Install programmer from [stlink](https://github.com/texane/stlink)
* Install the make utility (for Windows only) from [MinGW and MSYS](http://www.mingw.org/)
* Add these to your `PATH`

## Usage

Browse into any directory from terminal and run `make`.
```
cd blinky
make
```

If everything is setup correctly, you should see the completed message.
```
Cleaning blinky
Building blinky.c
   text    data     bss     dec     hex filename
    368	      0	      0	    368	    170	blinky.elf
Successfully finished...
```

## Program

Run `make burn` from the project directory after `make`
```
...
.. Flash written and verified! jolly good!
```

## Projects

* [blinky](projects/blinky/) - Good old blink LEDs example
* [clock](projects/clock/) - Shows how to change clock frequencies on the fly
* [math](projects/math/) - A simple sine function to test math library operation
* [systick](projects/systick/) - Blinks LEDs using systick timer. Processor clock is set to max (168 Mhz)
* [timer](projects/timer/) - Blinks LEDs one at a time using the Timer module and Timer interrupt
* [pwm](projects/pwm/) - Fades an LED using pwm functionality using Timer module
* [extint](projects/extint/) - External interrupt example using the on-board push-button
* [usb-vcp](projects/usb-vcp/) - USB Virtual COM Port implementation example. It depends on the [libopencm3](https://github.com/libopencm3/libopencm3) library for the USB stack