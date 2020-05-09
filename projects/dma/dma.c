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
int main(void);

#define BUF_SIZE 256

// Create a pointer at source and destination addresses
volatile uint8_t *src_addr = (uint8_t *) 0x2000A000;
volatile uint8_t *dst_addr = (uint8_t *) 0x2000A400;

void DMA2_Stream0_IRQHandler(void)
{
    if (DMA2->LISR & (1 << 5)) {
        // clear stream 0 transfer complete interrupt
        DMA2->LIFCR |= (1 << 5);
        // Check out the destination contents after DMA transfer
        for (int i=0; i<BUF_SIZE/4; i++){
            GPIOD->ODR = (uint16_t)(dst_addr[i] << 12);
            for(volatile int j=1000000; j>0; j--);
        }
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

    // Fill src_addr with numbers
    // Zero out dst_addr
    for (int i=0; i<BUF_SIZE; i++){
        src_addr[i] = (uint8_t)i;
        dst_addr[i] = 0;
    }

    // SETUP DMA2

    // enable DMA2 clock, bit 22 on AHB1ENR
    RCC->AHB1ENR |= (1 << 22);

    // clear DMA Stream
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
    DMA2_Stream0->NDTR = BUF_SIZE;

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
