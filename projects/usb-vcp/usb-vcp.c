/*
 * usb-vcp.c
 *
 * author: Furkan Cayci
 * description:
 *    USB Virtual COM Port (vcp) implementation.
 *    Uses USB CDC class (Communications Device Class) from libopencm3
 *    After programming, micro-USB will act as a serial port.
 *    Any other libopencm3 library calls are not used, thus not included.
 *    Only the USB stack is included.
 *
 * setup:
 *    uses the micro-USB port for PC communication.
 *    Connect it as you would to a serial port.
 *    (putty, screen, minicom, realterm, etc...)
 *    It will receive a character and print the next character.
 */


#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include <stddef.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/cdc.h>
#include "usb-vcp.h"

/*
 * NOTE: This is added since we are not using
 * all of the libopencm3 function calls.
 * On usb_f107.c file, there is a single call to
 * enable OTGFS clock which is already done in
 * our code.
 */
void rcc_periph_clock_enable(x) {};

/*************************************************
* function declarations
*************************************************/
void Default_Handler(void);
void Systick_Handler(void);
void init_systick(uint32_t s, uint8_t cen);
int main(void);
void delay_ms(volatile uint32_t);

/*************************************************
* variables
*************************************************/
static volatile uint32_t tDelay;

/*************************************************
* Vector Table
*************************************************/
// get the stack pointer location from linker
typedef void (* const intfunc)(void);
extern unsigned long __stack;

// attribute puts table in beginning of .vectors section
//   which is the beginning of .text section in the linker script
// Add other vectors -in order- here
// Vector table can be found on page 372 in RM0090
__attribute__ ((section(".vectors")))
void (* const vector_table[])(void) = {
	(intfunc)((unsigned long)&__stack), /* 0x000 Stack Pointer */
	Reset_Handler,                      /* 0x004 Reset         */
	Default_Handler,                    /* 0x008 NMI           */
	Default_Handler,                    /* 0x00C HardFault     */
	Default_Handler,                    /* 0x010 MemManage     */
	Default_Handler,                    /* 0x014 BusFault      */
	Default_Handler,                    /* 0x018 UsageFault    */
	0,                                  /* 0x01C Reserved      */
	0,                                  /* 0x020 Reserved      */
	0,                                  /* 0x024 Reserved      */
	0,                                  /* 0x028 Reserved      */
	Default_Handler,                    /* 0x02C SVCall        */
	Default_Handler,                    /* 0x030 Debug Monitor */
	0,                                  /* 0x034 Reserved      */
	Default_Handler,                    /* 0x038 PendSV        */
	Systick_Handler                     /* 0x03C SysTick       */
};

/*************************************************
* default interrupt handler
*************************************************/
void Default_Handler(void)
{
	for (;;);  // Wait forever
}

/*************************************************
* default systick interrupt handler
*************************************************/
void Systick_Handler(void)
{
	if (tDelay != 0x00)
	{
		tDelay--;
	}
}

/*************************************************
* initialize SysTick
*************************************************/
void init_systick(uint32_t s, uint8_t cen)
{
	// Clear CTRL register
	SysTick->CTRL = 0x00000;
	// Main clock source is running with HSI by default which is at 8 Mhz.
	// SysTick clock source can be set with CTRL register (Bit 2)
	// 0: Processor clock/8 (AHB/8)
	// 1: Processor clock (AHB)
	SysTick->CTRL |= (0 << 2);
	// Enable callback (bit 1)
	SysTick->CTRL |= ((uint32_t)cen << 1);
	// Load the value
	SysTick->LOAD = s;
	// Set the current value to 0
	SysTick->VAL = 0;
	// Enable SysTick (bit 0)
	SysTick->CTRL |= (1 << 0);
}

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static enum usbd_request_return_codes cdcacm_control_request(usbd_device *usbd_dev,
	struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
	void (**complete)(usbd_device *usbd_dev, struct usb_setup_data *req))
{
	(void)complete;
	(void)buf;
	(void)usbd_dev;

	switch (req->bRequest) {
	case USB_CDC_REQ_SET_CONTROL_LINE_STATE: {
		/*
		 * This Linux cdc_acm driver requires this to be implemented
		 * even though it's optional in the CDC spec, and we don't
		 * advertise it in the ACM functional descriptor.
		 */
		return USBD_REQ_HANDLED;
		}
	case USB_CDC_REQ_SET_LINE_CODING:
		if (*len < sizeof(struct usb_cdc_line_coding)) {
			return USBD_REQ_NOTSUPP;
		}

		return USBD_REQ_HANDLED;
	}
	return USBD_REQ_NOTSUPP;
}

static void cdcacm_set_config(usbd_device *usbd_dev, uint16_t wValue)
{
	(void)wValue;

	/* setup receive callback */
	usbd_ep_setup(usbd_dev, 0x01, USB_ENDPOINT_ATTR_BULK, 64, cdcacm_data_rx_cb);

	/* setup transmit callback */
	usbd_ep_setup(usbd_dev, 0x82, USB_ENDPOINT_ATTR_BULK, 64, NULL);

	usbd_ep_setup(usbd_dev, 0x83, USB_ENDPOINT_ATTR_INTERRUPT, 16, NULL);

	usbd_register_control_callback(
				usbd_dev,
				USB_REQ_TYPE_CLASS | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				cdcacm_control_request);
}

/*
 * Receiver callback function
 */
static void cdcacm_data_rx_cb(usbd_device *usbd_dev, uint8_t ep)
{
	(void)ep;

	char buf[64];
	int len = usbd_ep_read_packet(usbd_dev, 0x01, buf, 64);

	if (len) {
		/* create a function to perform stuff... */
		for (int i=0; i<len; i++){
			if (buf[i] == '\r'){
			}
			else if (!((buf[i] == 'z') || (buf[i] == 'Z'))) {
				buf[i] += 1;
			} else {
				buf[i] -= 25;
			}
		}
		while (usbd_ep_write_packet(usbd_dev, 0x82, buf, len) == 0);
	}

	/* toggle an LED to see something is happening */
	GPIOD->ODR ^= 0x1000;
}

/*************************************************
* main code starts from here
*************************************************/
int main(void)
{
	set_sysclk_to_168();
	init_systick(21000, 1);

	/* Enable GPIOD clock : bit3 */
	RCC->AHB1ENR |= (1 << 3);

	/* Set up pins 12,13,14,15 as output */
	GPIOD->MODER &= 0x00FFFFFF;
	GPIOD->MODER |= 0x55000000;

	/* Turn on last LED */
	GPIOD->ODR |= 0x8000;

	/* Enable GPIOA clock : bit0 */
	RCC->AHB1ENR |= (1 << 0);

	/* Enable OTG FS clock : bit7 in AHB2 */
	RCC->AHB2ENR |= (1 << 7);

	/* Setup GPIOA pins 9, 11, 12 as AF for USB */
	GPIOA->MODER &= (uint32_t)~(0x03CC0000);
	GPIOA->MODER |= (0x2 << 18); // pin 9
	GPIOA->MODER |= (0x2 << 22); // pin 11
	GPIOA->MODER |= (0x2 << 24); // pin 12

	// Choose OTG FS as Alternative Function 10 for pins 9, 11, 12
	GPIOA->AFR[1] |= (10 << 4);  // pin 9
	GPIOA->AFR[1] |= (10 << 12);  // pin 11
	GPIOA->AFR[1] |= (10 << 16);  // pin 12

	usbd_device *usbd_dev;

	usbd_dev = usbd_init(&otgfs_usb_driver, &dev, &config,
			usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer)
	);

	usbd_register_set_config_callback(usbd_dev, cdcacm_set_config);

	while(1){
		usbd_poll(usbd_dev);
	};

	return 0;
}

void delay_ms(volatile uint32_t s)
{
	tDelay = s;
	while(tDelay != 0);
}
