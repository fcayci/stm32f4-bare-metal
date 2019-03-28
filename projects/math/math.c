/*
 * math.c
 *
 * author: Furkan Cayci
 * description:
 *    implements a simple sine function to test math library operation
 *    processor clock is set to max (168 Mhz)
 *    math library needs to be linked. added to makefile
 *
 * setup:
 *    uses 4 on-board LEDs
 */

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include <math.h>

/*************************************************
* function declarations
*************************************************/
void Default_Handler(void);
void Systick_Handler(void);
void init_systick(uint32_t s, uint8_t cen);
int main(void);
void delay_ms(volatile uint32_t s);

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
* default interrupt handler
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

/*************************************************
* main code starts from here
*************************************************/
int main(void)
{
	/* set system clock to 168 Mhz */
	set_sysclk_to_168();

	uint32_t i;
	double x;
	// configure SysTick to interrupt every 21k ticks
	// enable callback
	init_systick(21000, 1);

	// setup LEDs
	RCC->AHB1ENR |= (1 << 3); // enable GPIOD clock
	GPIOD->MODER &= 0x00FFFFFF; // clear bits 31-24
	GPIOD->MODER |= 0x55000000; // set bit groups to 01 (0101 is 5 in hex)
	GPIOD->ODR |= 0xF000; // turn on all LEDs

	while(1)
	{
		for (i=0; i<1000; i++)
		{
			x = 4 * ( sin(2 * M_PI * (float)i / 1000) + 1);

			if (x > 4)      GPIOD->ODR = (0xF << 12);
			else if (x > 3) GPIOD->ODR = (0x7 << 12);
			else if (x > 2) GPIOD->ODR = (0x3 << 12);
			else if (x > 1) GPIOD->ODR = (0x1 << 12);
			else            GPIOD->ODR = (0x0 << 12);

			delay_ms(1);
		}
	}

	return 0;
}

/*
 * Millisecond delay function.
 *   volatile is used so that compiler does not optimize it away
 */
void delay_ms(volatile uint32_t s)
{
	tDelay = s;
	while(tDelay != 0);
}
