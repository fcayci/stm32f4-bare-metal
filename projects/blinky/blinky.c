/*
 * blinky.c
 *
 * author: Furkan Cayci
 * description:
 *    blinks LEDs
 *    system clock is running from HSI (16 Mhz)
 *
 * setup:
 *    uses 4 on-board LEDs
 */

#include "stm32f407xx.h"
#include "system_stm32f4xx.h"

#define LEDDELAY	1000000

/*************************************************
* function declarations
*************************************************/
void Reset_Handler(void);
void Default_Handler(void);
void _init_data(void);
int main(void);
void delay(volatile uint32_t);

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
	(intfunc)((unsigned long)&__stack),	/* 0x000 Stack Pointer */
	Reset_Handler,               		/* 0x004 Reset         */
	Default_Handler,					/* 0x008 NMI           */
	Default_Handler,                    /* 0x00C HardFaullt    */
	Default_Handler,                    /* 0x010 MemManage     */
	Default_Handler,                    /* 0x014 BusFault      */
	Default_Handler,                    /* 0x018 UsageFault    */
	0,                   			    /* 0x01C Reserved      */
	0,     				                /* 0x020 Reserved      */
	0,               			        /* 0x024 Reserved      */
	0, 				                    /* 0x028 Reserved      */
	Default_Handler,                    /* 0x02C SVCall        */
	Default_Handler,                    /* 0x030 Debug Monitor */
	0,                                  /* 0x034 Reserved      */
	Default_Handler,                    /* 0x038 PendSV        */
	Default_Handler                     /* 0x03C SysTick       */
};

/*************************************************
* entry point for the program
* initializes data and bss sections and calls main program
*************************************************/
void Reset_Handler(void)
{
	/* initialize data and bss sections */
	_init_data();

	/* FPU settings */
	#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
	#endif

  	/* Reset the RCC clock configuration to the default reset state */
  	/* Set HSION bit */
  	RCC->CR |= (1 << 0);

	/* Reset CFGR register */
	RCC->CFGR = 0x00000000;

	/* Reset HSEON (16), CSSON (19) and PLLON (24) bits */
	RCC->CR &= ~(uint32_t)((1 << 16) | (1 << 19) | (1 << 24));

	/* Reset PLLCFGR register to reset value*/
	RCC->PLLCFGR = 0x24003010;

	/* Reset HSEBYP bit */
	RCC->CR &= ~(uint32_t)(1 << 18);

	/* Disable all interrupts */
	RCC->CIR = 0x00000000;

  	/* Configure the Vector Table location add offset address */
	#ifdef VECT_TAB_SRAM
	SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal SRAM */
	#else
	SCB->VTOR = FLASH_BASE | VECT_TAB_OFFSET; /* Vector Table Relocation in Internal FLASH */
	#endif

	/* call main function */
	main();

	/* wait forever */
	for (;;);
}

/*************************************************
* Copy the data contents from LMA to VMA
* Initializes data and bss sections
*************************************************/
void _init_data(void)
{
	extern unsigned long __etext, __data_start__, __data_end__, __bss_start__, __bss_end__;
	unsigned long *src = &__etext;
	unsigned long *dst = &__data_start__;

	/* ROM has data at end of text; copy it.  */
	while (dst < &__data_end__)
		*dst++ = *src++;

	/* zero bss.  */
	for (dst = &__bss_start__; dst< &__bss_end__; dst++)
		*dst = 0;
}

/*************************************************
* default interrupt handler
*************************************************/
void Default_Handler(void)
{
	for (;;);  // Wait forever
}

/*************************************************
* main code starts from here
*************************************************/
int main(void)
{
	// Each module is powered separately. In order to turn on a module
	// we need to enable the relevant clock.
	// Set Bit 3 to enable GPIOD clock in AHB1ENR
	// AHB1ENR: XXXX XXXX XXXX XXXX XXXX XXXX XXXX 1XXX
	RCC->AHB1ENR |= 0x00000008;

	// Another way to write a 1 to a bit location is to shift it that much
	// Meaning shift number 1, 3 times to the left. Which would result in
	// 0b1000 or 0x8
	// RCC->AHB1ENR |= (1 << 3);

	// In order to make a pin output, we need to write 01 to the relevant
	// section in MODER register
	// We first need to AND it to reset them, then OR it to set them.
	//                     bit31                                         bit0
	// MODER register bits : 01 01 01 01 XX XX XX XX XX XX XX XX XX XX XX XX
	//                      p15                                           p0

	GPIOD->MODER &= 0x00FFFFFF;   // Reset bits 31-24 to clear old values
	GPIOD->MODER |= 0x55000000;   // Set MODER bits to 01 (0101 is 5 in hex)

	// You can do the same setup with shifting
	// GPIOD->MODER &= ~(0xFF << 24); //or GPIOD->MODER &= ~(0b11111111 << 24);
	// GPIOD->MODER |=  (0x55 << 24); //or GPIOD->MODER |=  (0b01010101 << 24);

	// Set Pins 12-15 to 1 to turn on all LEDs
	// ODR: 1111 XXXX XXXX XXXX
	GPIOD->ODR |= 0xF000;

	// You can do the same setup with shifting
	// GPIOD->ODR |= (0xF << 12);

	while(1)
	{
		delay(LEDDELAY);
		GPIOD->ODR ^= 0xF000;  // Toggle LEDs
	}

	__asm__("NOP"); // Assembly inline can be used if needed
	return 0;
}

// A simple and not accurate delay function
void delay(volatile uint32_t s)
{
	for(s; s>0; s--){
		// Do nothing
	}
}
