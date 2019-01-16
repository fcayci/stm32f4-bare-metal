TARGET = pwm
SRCS = pwm.c

LINKER_SCRIPT = ../../flash/stm32f407.ld

# Choose processor
CDEFS  = -DSTM32F407xx
# Enable FPU
CDEFS += -D__VFP_FP__

# link math library
LIBS = -lm

include ../armf4.mk