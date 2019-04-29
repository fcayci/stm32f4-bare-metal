/*
 * dma.c
 *
 * author: Furkan Cayci
 * description:
 *   DMA memory to memory transfer example
 *   to transfer 256 bytes of data.
 *   Only DMA2 controller is able to perform memory-to-memory transfers
 *
 * setup:
 *   1. Enable DMA2 clock from AHBENR register (bit22)
 *   2. Disable any ongoing dma transfers and make sure it is ended
 *   3. set data direction to memory-to-memory
 *   4. Choose channel using CHSEL bit. memory-to-memory does not matter
 *      peripheral-to-memory check the table
 *   5. Choose increment mode for both memory and peripheral registers (MINC/PINC)
 *   6. Choose increment size MSIZE/PSIZE
 *   7. Set source/destination memory addresses PAR/M0AR
 *   8. Set number of items to be transferred NDTR
 *   9. Set priority PL
 *   10. (optional) Enable transfer complete interrupt
 *   11. Enable dma transfer
 */

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

/*************************************************
* function declarations
*************************************************/
void Default_Handler(void);
void DMA2_Stream0_IRQHandler(void);
int main(void);

// Create a pointer at source and destination addresses
uint8_t *src_addr = (uint8_t *) 0x2000A000;
uint8_t *dst_addr = (uint8_t *) 0x2000A400;

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
	Default_Handler,                    /* 0x03C SysTick       */
	0,                                  /* 0x040 Window WatchDog Interrupt                                         */
	0,                                  /* 0x044 PVD through EXTI Line detection Interrupt                         */
	0,                                  /* 0x048 Tamper and TimeStamp interrupts through the EXTI line             */
	0,                                  /* 0x04C RTC Wakeup interrupt through the EXTI line                        */
	0,                                  /* 0x050 FLASH global Interrupt                                            */
	0,                                  /* 0x054 RCC global Interrupt                                              */
	0,                                  /* 0x058 EXTI Line0 Interrupt                                              */
	0,                                  /* 0x05C EXTI Line1 Interrupt                                              */
	0,                                  /* 0x060 EXTI Line2 Interrupt                                              */
	0,                                  /* 0x064 EXTI Line3 Interrupt                                              */
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
	0,                                  /* 0x0B8 TIM4 global Interrupt                                             */
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
	DMA2_Stream0_IRQHandler,            /* 0x120 DMA2 Stream 0 global Interrupt                                    */
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
* default interrupt handler
*************************************************/
void Default_Handler(void)
{
	for (;;);  // Wait forever
}

void DMA2_Stream0_IRQHandler(void)
{
	// clear stream 0 transfer complete interrupt
	DMA2->LIFCR |= (1 << 5);
	// Check out the destination contents after DMA transfer
	for (uint32_t i=0; i<64; i++){
		GPIOD->ODR = (uint16_t)(dst_addr[i] << 12);
		for(uint32_t j=1000000; j>0; j--);
	}
}

/*************************************************
* main code starts from here
*************************************************/
int main(void)
{
	volatile uint32_t i;

	/* set system clock to 168 Mhz */
	set_sysclk_to_168();

	/* Enable GPIOD clock (AHB1ENR: bit 3) */
	RCC->AHB1ENR |= (1 << 3);

	// make leds output
	GPIOD->MODER &= 0xCCFFFFFF;
	GPIOD->MODER |= 0x55000000;
	// light them up before the storm
	GPIOD->ODR = (0xF << 12);
	// wait a bit
	for(i=10000000; i>0; i--);

	// Fill src_addr with numbers
	// Zero out dst_addr
	for (i=0; i<256; i++){
		src_addr[i] = (uint8_t)i;
		dst_addr[i] = 0;
	}

	// SETUP DMA2

	// enable DMA2 clock, bit 22 on AHB1ENR
	RCC->AHB1ENR |= (1 << 22);

	// disable any ongoing dma bit0
	//DMA2_Stream0->CR &= (uint32_t)~(1 << 0);
	DMA2_Stream0->CR = 0;
	// wait until dma is disabled
	while(DMA2_Stream0->CR & (1 << 0));

	// set channel CHSEL: bits27:25 to channel0
	DMA2_Stream0->CR |= (0 << 25);

	// set data transfer direction DIR: bits7:6 memory-to-memory
	DMA2_Stream0->CR |= (0x2 << 6);

	// increment memory MINC : bit10
	DMA2_Stream0->CR |= (1 << 10);
	// memory data size MSIZE : bits14:13 to byte
	DMA2_Stream0->CR |= (0x0 << 13);

	// increment peripheral PINC : bit9
	DMA2_Stream0->CR |= (1 << 9);
	// peripheral data size PSIZE : bits12:11 to byte
	DMA2_Stream0->CR |= (0x0 << 11);

	// source memory address
	DMA2_Stream0->PAR = (uint32_t)src_addr;
	// destination memory address
	DMA2_Stream0->M0AR = (uint32_t)dst_addr;
	// number of items to be transferred
	DMA2_Stream0->NDTR = 256;

	// set channel priority PL bits17:16 to medium
	DMA2_Stream0->CR |= (0x1 << 16);

	// enable transfer complete interrupt bit4
	DMA2_Stream0->CR |= (1 << 4);
	NVIC_EnableIRQ(DMA2_Stream0_IRQn);

	// enable dma bit0
	DMA2_Stream0->CR |= (1 << 0);

	while(1)
	{
	}

	return 0;
}
