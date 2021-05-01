/*
 * dma-gpio.c
 *
 * author: Furkan Cayci
 * modifier: Cerem Cem ASLAN
 * description:
 *   DMA memory to GPIO transfer example that transfers 256 bytes of data.
 *   GPIO registers are just another are of memory. Only DMA2 controller 
 *   is able to perform memory-to-memory transfers.
 *
 * Setup:
 *   1. Enable DMA2 clock from AHBENR register (bit22)
 *   2. Disable any ongoing dma transfers and make sure it is ended
 *   3. Set data direction to memory-to-memory
 *   4. Choose channel using CHSEL bit. memory-to-memory does not matter
 *      peripheral-to-memory check the table
 *   5. Choose increment mode for both memory and peripheral registers (MINC/PINC)
 *   6. Choose increment size MSIZE/PSIZE
 *   7. Set source/destination memory addresses PAR/M0AR
 *   8. Set number of items to be transferred NDTR
 *   9. Set priority PL
 *   10. (optional) Enable transfer complete interrupt
 *   11. Fire dma transfer
 * 
 * Application:
 *   1. Setup DMA
 *   2. Refire the same operation when it's complete.
 */

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

/*************************************************
* function declarations
*************************************************/
int main(void);

#define BUF_SIZE 256

// Create a pointer at source and destination addresses
volatile uint16_t *src_addr = (uint16_t *) 0x2000A000;
volatile uint16_t *dst_addr = (uint16_t *) &(GPIOE->ODR);

void fire_dma_transfer(){
    /********************************
     * setup DMA
     * (RM: 10.5.5 DMA stream x configuration register (DMA_SxCR))
     *******************************/

    // enable DMA2 clock, bit 22 on AHB1ENR
    RCC->AHB1ENR |= (1 << 22);

    // clear DMA Stream configuration register
    // single transfer, M0 is target, single buffer mode, circular buffer disabled
    DMA2_Stream0->CR = 0;
    // wait until dma is disabled
    while(DMA2_Stream0->CR & (1 << 0));

    // set channel CHSEL: bits27:25 to channel0
    DMA2_Stream0->CR |= (0 << 25);

    // set data transfer direction DIR: bits7:6 memory-to-memory
    DMA2_Stream0->CR |= (2 << 6);

    // set channel priority PL bits17:16 to medium
    DMA2_Stream0->CR |= (0x1 << 16);

    // enable circular mode 
    //DMA2_Stream0->CR |= (0x1 << 8); // optional

    // source
    // increment peripheral PINC : bit9
    DMA2_Stream0->CR |= (1 << 9);
    // peripheral data size PSIZE : bits12:11 to half-word
    DMA2_Stream0->CR |= (1 << 11);

    // destination
    // memory pointer is fixed: MINC : bit10
    DMA2_Stream0->CR |= (0 << 10);
    // memory data size MSIZE : bits14:13 to half-word
    DMA2_Stream0->CR |= (1 << 13);

    // source memory address
    DMA2_Stream0->PAR = (uint32_t)src_addr;
    // destination memory address
    DMA2_Stream0->M0AR = (uint32_t)dst_addr;
    // number of items to be transferred
    DMA2_Stream0->NDTR = BUF_SIZE;

    // enable transfer complete interrupt bit4
    DMA2_Stream0->CR |= (1 << 4);
    NVIC_EnableIRQ(DMA2_Stream0_IRQn);

    // enable dma bit0
    DMA2_Stream0->CR |= (1 << 0);

}

void DMA2_Stream0_IRQHandler(void)
{
    if (DMA2->LISR & (1 << 5)) {
        // clear stream 0 transfer complete interrupt
        DMA2->LIFCR |= (1 << 5);

        // wait a bit 
        for(volatile int j=100; j>0; j--);    

        // restart dma transfer with the same settings 
        DMA2_Stream0->CR |= (1 << 0); //// debugger: printf "dma2 stream: %x\n", DMA2_Stream0->CR
    }
}


/*************************************************
* main code starts from here
*************************************************/
int main(void)
{
    set_sysclk_to_168();
    /********************************
     * setup output - GPIOE 12,13,14,15
     *******************************/
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    GPIOE->MODER &= ~(0xFFU << 24);
    GPIOE->MODER |= (0x55 << 24);
    GPIOE->ODR    = (0xaaaa);

    // wait a bit
    for(volatile int i=10000000; i>0; i--);

    // Fill src_addr with numbers
    uint16_t x = 0xff; 
    for (int i=0; i<BUF_SIZE; i++){
        src_addr[i] = (uint16_t)x << 8;
        if ((i % 32) == 0) // result is 666 kHz if we toggle the output on every 32 transfer
            x = ~x; 
    }

    fire_dma_transfer();

    while(1)
    {
    }

    return 0;
}
