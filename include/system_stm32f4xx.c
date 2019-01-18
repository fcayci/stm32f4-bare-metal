/*
 * system_stm32f07.c
 *
 * author: Furkan Cayci
 * description:
 *    this is a custom code for common usage in projects.
 */

#include "system_stm32f4xx.h"

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
* entry point for the program
* initializes data and bss sections and calls main program
*************************************************/
void Reset_Handler(void)
{
	/* initialize data and bss sections */
	_init_data();

	/* FPU settings, can be enabled from project makefile */
	#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  /* set CP10 and CP11 Full Access */
	#endif

	/* reset clock */
	reset_clock();

	/* call main function */
	main();

	/* wait forever */
	for (;;);
}


/*************************************************
* reset clock to HSI
*************************************************/
void reset_clock(void)
{
  	/* Reset the RCC clock configuration to the default reset state */
  	/* Set HSION bit */
  	RCC->CR = (1U << 0);

	/* Reset CFGR register */
	RCC->CFGR = 0x00000000;

	/* Reset HSEON (16), CSSON (19) and PLLON (24) bits */
	RCC->CR &= ~(uint32_t)((1U << 16) | (1U << 19) | (1U << 24));

	/* Reset PLLCFGR register to reset value*/
	RCC->PLLCFGR = 0x24003010UL;

	/* Reset HSEBYP bit */
	RCC->CR &= ~(uint32_t)(1U << 18);

	/* Disable all clock interrupts */
	RCC->CIR = 0x00000000UL;
}


/*************************************************
* configure system clock to 168 Mhz
* this is only tested on stm32f4 discovery board
*************************************************/
void set_sysclk_to_168(void)
{
	reset_clock();

	/* Enable HSE (CR: bit 16) */
	RCC->CR |= ((uint32_t) (1 << 16));
	/* Wait till HSE is ready (CR: bit 17) */
	while(!(RCC->CR & (1 << 17)));

	/* Enable power interface clock (APB1ENR:bit 28) */
	RCC->APB1ENR |= (1 << 28);

	/* set voltage scale to 1 for max frequency (PWR_CR:bit 14)
	 * (0b0) scale 2 for fCLK <= 144 Mhz
	 * (0b1) scale 1 for 144 Mhz < fCLK <= 168 Mhz
	 */
	PWR->CR |= (1 << 14);

	/* set AHB prescaler to /1 (CFGR:bits 7:4) */
	RCC->CFGR |= (0 << 4);
	/* set ABP low speed prescaler to /4 (APB1) (CFGR:bits 12:10) */
	RCC->CFGR |= (0x5 << 10);
	/* set ABP high speed prescaper to /2 (ABP2) (CFGR:bits 15:13) */
	RCC->CFGR |= (0x4 << 13);

	/* Set M, N, P and Q PLL dividers
	 * PLLCFGR: bits 5:0 (M), 14:6 (N), 17:16 (P), 27:24 (Q)
	 * Set PLL source to HSE, PLLCFGR: bit 22, 1:HSE, 0:HSI
	 */
	RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) |
				   (PLL_Q << 24) | (1 << 22);
	/* Enable the main PLL (CR: bit 24) */
	RCC->CR |= (1 << 24);
	/* Wait till the main PLL is ready (CR: bit 25) */
	while((RCC->CR & (1 << 25)) == 0);
	/* Configure Flash
	 * prefetch enable (ACR:bit 8)
	 * instruction cache enable (ACR:bit 9)
	 * data cache enable (ACR:bit 10)
	 * set latency to 5 wait states (ARC:bits 2:0)
	 *   see Table 10 on page 80 in RM0090
	 */
	FLASH->ACR = (1 << 8) | (1 << 9) | (1 << 10 ) | (0x5 << 0);

	/* Select the main PLL as system clock source, (CFGR:bits 1:0)
	 * 0b00 - HSI
	 * 0b01 - HSE
	 * 0b10 - PLL
	 */
	RCC->CFGR &= (uint32_t)~(0x3 << 0);
	RCC->CFGR |= (0x2 << 0);
	/* Wait till the main PLL is used as system clock source (CFGR:bits 3:2) */
	while ((RCC->CFGR & (uint32_t)(0x2 << 2) ) != (uint32_t)(0x2 << 2));
}
