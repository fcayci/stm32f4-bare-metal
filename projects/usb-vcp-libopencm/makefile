TARGET = usb-vcp
SRCS = usb-vcp.c

LINKER_SCRIPT = ../../flash/STM32F407VGTX_FLASH.ld

# Generate debug info
DEBUG = 1

CDEFS  = -DSTM32F407xx

# Enable FPU
#CDEFS += -D__VFP_FP__

# link math library
#LIBS = -lm

# add libopencm3 for the USB stack
LIBOPENCM3 = ../../libs/libopencm3
INCLUDES += -I$(LIBOPENCM3)/include

# Add STM32F4 for memoryamp
CDEFS += -DSTM32F4

# USB sources to be compiled
SRCS += $(LIBOPENCM3)/lib/usb/usb.c
SRCS += $(LIBOPENCM3)/lib/usb/usb_standard.c
SRCS += $(LIBOPENCM3)/lib/usb/usb_control.c
SRCS += $(LIBOPENCM3)/lib/usb/usb_dwc_common.c
SRCS += $(LIBOPENCM3)/lib/usb/usb_f107.c

include ../armf4.mk