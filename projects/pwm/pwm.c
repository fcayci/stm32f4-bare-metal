/*
 * pwm.c
 *
 * author: Furkan Cayci
 * description:
 *    fades one LED using pwm functionality on timer4
 *    the LEDs are connected to GPIOD 12-15
 *    which has timer4 capability.
 *    GPIOD 12 is connected to timer4 channel1
 *
 * setup:
 *    uses 1 on-board LED
 */

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include <math.h>

/*************************************************
* function declarations
*************************************************/
void Reset_Handler(void);
void Default_Handler(void);
void tim4_handler(void);
void set_sysclk_to_max(void);
void _init_data(void);
int main(void);

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
	0,                                  /* 0x02C SVCall        */
	0,                                  /* 0x030 Debug Monitor */
	0,                                  /* 0x034 Reserved      */
	0,                                  /* 0x038 PendSV        */
	0,                                  /* 0x03C SysTick       */
	0,							        /* 0x040 Window WatchDog Interrupt                                         */
	0,        				            /* 0x044 PVD through EXTI Line detection Interrupt                         */
	0,                                  /* 0x048 Tamper and TimeStamp interrupts through the EXTI line             */
	0,                                  /* 0x04C RTC Wakeup interrupt through the EXTI line                        */
	0,                                  /* 0x050 FLASH global Interrupt                                            */
	0,                                  /* 0x054 RCC global Interrupt                                              */
	0,                                  /* 0x058 EXTI Line0 Interrupt                                              */
	0,                                  /* 0x05C EXTI Line1 Interrupt                                              */
	0,                                  /* 0x060 EXTI Line2 Interrupt                                              */
	0,           					    /* 0x064 EXTI Line3 Interrupt                                              */
	0,                                  /* 0x068 EXTI Line4 Interrupt                                              */
	0,                                  /* 0x06C DMA1 Stream 0 global Interrupt                                    */
	0,                                  /* 0x070 DMA1 Stream 1 global Interrupt                                    */
	0,                                  /* 0x074 DMA1 Stream 2 global Interrupt                                    */
	0,                                  /* 0x078 DMA1 Stream 3 global Interrupt                                    */
	0,                                  /* 0x07C DMA1 Stream 4 global Interrupt                                    */
	0,                                  /* 0x080 DMA1 Stream 5 global Interrupt                                    */
	0,                                  /* 0x084 DMA1 Stream 6 global Interrupt                                    */
	0,                                  /* 0x088 ADC1, ADC2 and ADC3 global Interrupts                             */
	0,                                  /* 0x08C CAN1 TX Interrupt                                                 */
	0,                                  /* 0x090 CAN1 RX0 Interrupt                                                */
	0,                                  /* 0x094 CAN1 RX1 Interrupt                                                */
	0,                                  /* 0x098 CAN1 SCE Interrupt                                                */
	0,                                  /* 0x09C External Line[9:5] Interrupts                                     */
	0,                                  /* 0x0A0 TIM1 Break interrupt and TIM9 global interrupt                    */
	0,                                  /* 0x0A4 TIM1 Update Interrupt and TIM10 global interrupt                  */
	0,                                  /* 0x0A8 TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
	0,                                  /* 0x0AC TIM1 Capture Compare Interrupt                                    */
	0,                                  /* 0x0B0 TIM2 global Interrupt                                             */
	0,                                  /* 0x0B4 TIM3 global Interrupt                                             */
	tim4_handler,                       /* 0x0B8 TIM4 global Interrupt                                             */
	0,                                  /* 0x0BC I2C1 Event Interrupt                                              */
	0,                                  /* 0x0C0 I2C1 Error Interrupt                                              */
	0,                                  /* 0x0C4 I2C2 Event Interrupt                                              */
	0,                                  /* 0x0C8 I2C2 Error Interrupt                                              */
	0,                                  /* 0x0CC SPI1 global Interrupt                                             */
	0,                                  /* 0x0D0 SPI2 global Interrupt                                             */
	0,                                  /* 0x0D4 USART1 global Interrupt                                           */
	0,                                  /* 0x0D8 USART2 global Interrupt                                           */
	0,                                  /* 0x0DC USART3 global Interrupt                                           */
	0,                                  /* 0x0E0 External Line[15:10] Interrupts                                   */
	0,                                  /* 0x0E4 RTC Alarm (A and B) through EXTI Line Interrupt                   */
	0,                                  /* 0x0E8 USB OTG FS Wakeup through EXTI line interrupt                     */
	0,                                  /* 0x0EC TIM8 Break Interrupt and TIM12 global interrupt                   */
	0,                                  /* 0x0F0 TIM8 Update Interrupt and TIM13 global interrupt                  */
	0,                                  /* 0x0F4 TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
	0,                                  /* 0x0F8 TIM8 Capture Compare global interrupt                             */
	0,                                  /* 0x0FC DMA1 Stream7 Interrupt                                            */
	0,                                  /* 0x100 FSMC global Interrupt                                             */
	0,                                  /* 0x104 SDIO global Interrupt                                             */
	0,                                  /* 0x108 TIM5 global Interrupt                                             */
	0,                                  /* 0x10C SPI3 global Interrupt                                             */
	0,                                  /* 0x110 UART4 global Interrupt                                            */
	0,                                  /* 0x114 UART5 global Interrupt                                            */
	0,                                  /* 0x118 TIM6 global and DAC1&2 underrun error  interrupts                 */
	0,                                  /* 0x11C TIM7 global interrupt                                             */
	0,                                  /* 0x120 DMA2 Stream 0 global Interrupt                                    */
	0,                                  /* 0x124 DMA2 Stream 1 global Interrupt                                    */
	0,                                  /* 0x128 DMA2 Stream 2 global Interrupt                                    */
	0,                                  /* 0x12C DMA2 Stream 3 global Interrupt                                    */
	0,                                  /* 0x130 DMA2 Stream 4 global Interrupt                                    */
	0,                                  /* 0x134 Ethernet global Interrupt                                         */
	0,                                  /* 0x138 Ethernet Wakeup through EXTI line Interrupt                       */
	0,                                  /* 0x13C CAN2 TX Interrupt                                                 */
	0,                                  /* 0x140 CAN2 RX0 Interrupt                                                */
	0,                                  /* 0x144 CAN2 RX1 Interrupt                                                */
	0,                                  /* 0x148 CAN2 SCE Interrupt                                                */
	0,                                  /* 0x14C USB OTG FS global Interrupt                                       */
	0,                                  /* 0x150 DMA2 Stream 5 global interrupt                                    */
	0,                                  /* 0x154 DMA2 Stream 6 global interrupt                                    */
	0,                                  /* 0x158 DMA2 Stream 7 global interrupt                                    */
	0,                                  /* 0x15C USART6 global interrupt                                           */
	0,                                  /* 0x160 I2C3 event interrupt                                              */
	0,                                  /* 0x164 I2C3 error interrupt                                              */
	0,                                  /* 0x168 USB OTG HS End Point 1 Out global interrupt                       */
	0,                                  /* 0x16C USB OTG HS End Point 1 In global interrupt                        */
	0,                                  /* 0x170 USB OTG HS Wakeup through EXTI interrupt                          */
	0,                                  /* 0x174 USB OTG HS global interrupt                                       */
	0,                                  /* 0x178 DCMI global interrupt                                             */
	0,                                  /* 0x17C RNG global Interrupt                                              */
	0                                   /* 0x180 FPU global interrupt                                              */
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

const uint32_t period = 5000;

/*************************************************
* timer 4 interrupt handler
*************************************************/
void tim4_handler(void)
{
	double x;
	static uint32_t t = 0;
	const uint32_t f = 10*period;
	// generate new value each time
	x = f/2 * ( sin(2 * M_PI * (float)t / (float)f) + 1);
	if (t == f)
	{
		t = 0;
	} else
	{
		++t;
	}

	// set new duty cycle
	TIM4->CCR1 = (uint16_t)(x/10);
}

/*************************************************
* main code starts from here
*************************************************/
int main(void)
{

	uint32_t duty = period/2;

	// enable GPIOD clock
	RCC->AHB1ENR |= (1 << 3);
	GPIOD->MODER &= 0x00FFFFFF;   // Reset bits 31-24 to clear old values
	GPIOD->MODER |= 0x02000000;   // Set pin 12 to alternate func. mode (0b10)

	// Choose Timer4 as Alternative Function for pin 12 led
	GPIOD->AFR[1] |= (0x2 << 16);

	// enable TIM4 clock (bit2)
	RCC->APB1ENR |= (1 << 2);

	// set prescaler to 167
	//   it will increment counter every prescalar cycles
	// fCK_PSC / (PSC[15:0] + 1)
	// 168 Mhz / 167 + 1 = 1 Mhz timer clock speed
	TIM4->PSC = 167;

	// set period
	TIM4->ARR = period;

	// set duty cycle on channel 1
	TIM4->CCR1 = duty;

	// enable channel 1 in capture/compare register
	// set oc1 mode as pwm (0b110 or 0x6 in bits 6-4)
	TIM4->CCMR1 |= (0x6 << 4);
	// enable oc1 preload bit 3
	TIM4->CCMR1 |= (1 << 3);

	// enable capture/compare ch1 output
	TIM4->CCER |= (1 << 0);

	// enable interrupt CC1IE (bit 1)
	TIM4->DIER |= (1 << 1);

	// enable TIM4 IRQ from NVIC
	NVIC_EnableIRQ(TIM4_IRQn);

	// Enable Timer 4 module (CEN, bit0)
	TIM4->CR1 |= (1 << 0);

	while(1)
	{
		// Do nothing. let timer handler do its magic
	}

	return 0;
}
