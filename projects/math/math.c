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
void Reset_Handler(void);
void Default_Handler(void);
void Systick_Handler(void);
void set_sysclk_to_max(void);
void init_systick(uint32_t s, uint8_t cen);
void _init_data(void);
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
	Systick_Handler                     /* 0x03C SysTick       */
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

	/* max system clock to 168 Mhz */
	set_sysclk_to_max();

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
* configure system clock to max
*************************************************/
void set_sysclk_to_max(void)
{
	/* Enable HSE */
	RCC->CR |= ((uint32_t)RCC_CR_HSEON);
	/* Wait till HSE is ready */
	while(!(RCC->CR & RCC_CR_HSERDY));
	/* Select regulator voltage output Scale 1 mode */
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	PWR->CR |= PWR_CR_VOS;
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;

	RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) |
		(RCC_PLLCFGR_PLLSRC_HSE) | (PLL_Q << 24);
	/* Enable the main PLL */
	RCC->CR |= RCC_CR_PLLON;
	/* Wait till the main PLL is ready */
	while((RCC->CR & RCC_CR_PLLRDY) == 0);
	/* Configure Flash prefetch, Instruction cache, Data cache and wait state */
	FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN |FLASH_ACR_DCEN |FLASH_ACR_LATENCY_5WS;

	/* Select the main PLL as system clock source */
	RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	/* Wait till the main PLL is used as system clock source */
	while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS ) != RCC_CFGR_SWS_PLL);
}

/*************************************************
* main code starts from here
*************************************************/
int main(void)
{
	uint32_t i;
	double x;
	// configure SysTick to interrupt every 21k ticks
	// enable callback
	init_systick(21000, 1);

	// enable GPIOD clock
	RCC->AHB1ENR |= (1 << 3);

	GPIOD->MODER &= 0x00FFFFFF;   // Reset bits 31-24 to clear old values
	GPIOD->MODER |= 0x55000000;   // Set MODER bits to 01 (0101 is 5 in hex)

	// Set Pins 12-15 to 1 to turn on all LEDs
	GPIOD->ODR |= 0xF000;

	while(1)
	{
		for (i=0; i<1000; i++)
		{
			x = 4 * ( sin(2 * M_PI * (float)i / 1000) + 1);

			if (x > 4)  	GPIOD->ODR = (0xF << 12);
			else if (x > 3) GPIOD->ODR = (0x7 << 12);
			else if (x > 2) GPIOD->ODR = (0x3 << 12);
			else if (x > 1) GPIOD->ODR = (0x1 << 12);
			else 			GPIOD->ODR = (0x0 << 12);

			delay_ms(1);
		}
	}

	return 0;
}

/*
 * Millisecond delay function.
 *   volatile keyword is used so that compiler does not optimize it away
 * Polling method (If interrupt is not enabled)
 */
void delay_ms(volatile uint32_t s)
{
	tDelay = s;
	while(tDelay != 0);
}