TARGET = uart
SRCS = uart.c

LINKER_SCRIPT = ../../flash/STM32F407VGTX_FLASH.ld

# Generate debug info
DEBUG = 1

# Choose processor
CDEFS  = -DSTM32F407xx
# Enable FPU
#CDEFS += -D__VFP_FP__

# link math library
#LIBS = -lm

include ../armf4.mk