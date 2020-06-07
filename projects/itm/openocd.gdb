# openocd -f stm32f4-ocd.cfg
# arm-none-eabi-gdb --command=openocd.gdb Debug/itm.elf
target extended-remote :3333

# print demangled symbols
set print asm-demangle on

# set backtrace limit to not have infinite backtrace loops
set backtrace limit 32

# detect unhandled exceptions, hard faults
break DefaultHandler
break HardFault
# stop at main (if possible)
break main

# enable semihost
# monitor arm semihosting enable

# # send captured ITM to the itme file
# # (the microcontroller SWO pin must be connected to the programmer SWO pin)
# # 8000000 must match the core clock frequency
# monitor tpiu config internal itm.txt uart off 168000000

# # OR: make the microcontroller SWO pin output compatible with UART (8N1)
# # 8000000 must match the core clock frequency
# # 2000000 is the frequency of the SWO pin
# monitor tpiu config external uart off 168000000 2000000

# # enable ITM port 0
# monitor itm port 0 on

load

# start the process but immediately halt the processor
stepi
