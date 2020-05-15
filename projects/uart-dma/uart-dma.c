/*
 * uart-dma.c
 *
 * author: Furkan Cayci
 * description:
 *   UART example with DMA transfer on TX
 *   uses USART2 PA2/PA3 pins to transmit data
 *   connect a Serial to USB adapter to see the
 *   data on PC
 *
 * setup:
 *   1. enable usart and associated dma clocks from RCC
 *   2. setup usart and enable dma transfer / receive bits
 *   3. setup memory-to-peripheral or peripheral-to-memory mode for DMA
 *   4. setup source and destination addresses, as well as the number of bytes to be transferred
 *   5. enable completion interrupt and enable both peripherals
 */

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

/*************************************************
* function declarations
*************************************************/
int main(void);

volatile uint8_t msg[] = "https://furkan.space/\n\r";

void DMA1_Stream6_IRQHandler(void)
{
    // clear stream transfer complete interrupt - bit21 for stream 6
    if (DMA1->HISR & DMA_HISR_TCIF6) {
        // clear interrupt
        DMA1->HIFCR |= DMA_HISR_TCIF6;
        // clean all LEDs
        GPIOD->ODR = 0;
    }
}

/*************************************************
* main code starts from here
*************************************************/
int main(void)
{
    /* set system clock to 168 Mhz */
    set_sysclk_to_168();

    /* Enable GPIOD clock (AHB1ENR: bit 3) */
    RCC->AHB1ENR |= (1 << 3);

    // make leds output
    GPIOD->MODER &= ~(0xFFU << 24);
    GPIOD->MODER |=  (0x55U << 24);
    // light them up before the storm
    GPIOD->ODR = (0xF << 12);
    // wait a bit
    for(volatile int i=10000000; i>0; i--);

    // enable USART2 clock, bit 17 on APB1ENR
    RCC->APB1ENR |= (1 << 17);

    // enable GPIOA clock, bit 0 on AHB1ENR
    RCC->AHB1ENR |= (1 << 0);

    // set pin modes as alternate mode 7 (pins 2 and 3)
    // USART2 TX and RX pins are PA2 and PA3 respectively
    GPIOA->MODER &= ~(0xFU << 4); // Reset bits 4:5 for PA2 and 6:7 for PA3
    GPIOA->MODER |=  (0xAU << 4); // Set   bits 4:5 for PA2 and 6:7 for PA3 to alternate mode (10)

    // choose AF7 for USART2 in Alternate Function registers
    GPIOA->AFR[0] |= (0x7 << 8); // for pin A2
    GPIOA->AFR[0] |= (0x7 << 12); // for pin A3

    // enable transmit DMA
    USART2->CR3 |= USART_CR3_DMAT;

    // USART2 word length M, bit 12
    //USART2->CR1 |= (0 << 12); // 0 - 1,8,n

    // USART2 parity control, bit 9
    USART2->CR1 |= (1 << 9); // 1 - odd parity

    // baud rate = fCK / (8 * (2 - OVER8) * USARTDIV)
    //   for fCK = 42 Mhz, baud = 115200, OVER8 = 0
    //   USARTDIV = 42Mhz / 115200 / 16
    //   = 22.7864 22.8125
    // we can also look at the table in RM0090
    //   for 42 Mhz PCLK, OVER8 = 0 and 115.2 KBps baud
    //   we need to program 22.8125
    // Fraction : 16*0.8125 = 13 (multiply fraction with 16)
    // Mantissa : 22
    // 12-bit mantissa and 4-bit fraction
    USART2->BRR |= (22 << 4);
    USART2->BRR |= 13;

    // USART2 TX enable, TE bit 3
    USART2->CR1 |= (1 << 3);

    // enable usart2 - UE, bit 13
    USART2->CR1 |= (1 << 13);

    // clear TC bit
    USART2->SR &= ~USART_SR_TC;

    // DMA Setup
    // We will transfer a given buffer to fixed UART peripheral address

    // USART2 TX DMA is mapped to DMA1 Stream6 Channel4
    //   from RM0090 pg 307

    // enable DMA1 clock, bit 21 on AHB1ENR
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
    // clear control register
    DMA1_Stream6->CR = 0;
    // wait until DMA is disabled
    while(DMA1_Stream6->CR & (1 << 0));

    // channel select
    DMA1_Stream6->CR |= (0x4 << 25); // channel4

    // enable transfer complete interrupt
    DMA1_Stream6->CR |= DMA_SxCR_TCIE; // (1 << 4);

    // peripheral data size already 00 for byte
    // memory increment mode
    DMA1_Stream6->CR |= DMA_SxCR_MINC; // (1 << 10);

    // Priority level
    DMA1_Stream6->CR |= (0x2 << 16); // high - 10

    // DIR bits should be 01 for memory-to-peripheral
    //   source is SxM0AR, dest is SxPAR
    DMA1_Stream6->CR |= (0x1 << 6);

    // source memory address
    DMA1_Stream6->M0AR = (uint32_t)msg;
    // destination memory address
    DMA1_Stream6->PAR = (uint32_t)&(USART2->DR);
    // number of items to be transferred
    DMA1_Stream6->NDTR = sizeof(msg);

    NVIC_SetPriority(DMA1_Stream6_IRQn, 3); // Priority level 3
    NVIC_EnableIRQ(DMA1_Stream6_IRQn);

    // enable DMA
    DMA1_Stream6->CR |= DMA_SxCR_EN; // (1 << 0);

    while(1)
    {

    }

    return 0;
}
