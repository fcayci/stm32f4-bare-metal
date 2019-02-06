TARGET = systick
SRCS = systick.c

LINKER_SCRIPT = ../../flash/stm32f407.ld

# Generate debug info
DEBUG = 0

# Choose processor
CDEFS  = -DSTM32F407xx
# Enable FPU
#CDEFS += -D__VFP_FP__

# link math library
#LIBS = -lm

include ../armf4.mk