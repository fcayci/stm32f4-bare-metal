/*
 * uart.c
 *
 * author: Furkan Cayci
 * description:
 *   UART example with tx interrupt
 *   uses USART2 PA2/PA3 pins to transmit data
 *   connect a Serial to USB adapter to see the
 *   data on PC
 *
 * setup:
 *   1. enable usart clock from RCC
 *   2. enable gpioa clock
 *   3. set PA2 and PA3 as af7
 *   4. set uart word length and parity
 *   5. enable transmit and receive (TE/RE bits)
 *   6. calculate baud rate and set BRR
 *   7. enable uart
 *   8. setup uart handler to send out a given buffer
 *   9. enable tx interrupt from NVIC
 *   10.enable tx interrupt and disable when the buffer
 *      transmission is complete
 */

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

/*************************************************
* function declarations
*************************************************/
int main(void);

const uint8_t brand[] = "furkan.space\n\r";
volatile int tx_complete = 0;
volatile int bufpos = 0;

void USART2_IRQHandler(void)
{
    // check if the source is transmit interrupt
    if (USART2->SR & (1 << 7)) {
        // clear interrupt
        USART2->SR &= (uint32_t)~(1 << 7);

        if (bufpos == sizeof(brand)) {
            // buffer is flushed out, disable tx interrupt
            tx_complete = 1;
            USART2->CR1 &= (uint32_t)~(1 << 7);
        }
        else {
            // flush ot the next char in the buffer
            tx_complete = 0;
            USART2->DR = brand[bufpos++];
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

    // enable USART2 clock, bit 17 on APB1ENR
    RCC->APB1ENR |= (1 << 17);

    // enable GPIOA clock, bit 0 on AHB1ENR
    RCC->AHB1ENR |= (1 << 0);

    // set pin modes as alternate mode 7 (pins 2 and 3)
    GPIOA->MODER &= 0xFFFFFF0F; // Reset bits 10-15 to clear old values
    GPIOA->MODER |= 0x000000A0; // Set pin 2/3 to alternate func. mode (0b10)

    // set pin modes as high speed
    GPIOA->OSPEEDR |= 0x000000A0; // Set pin 2/3 to high speed mode (0b10)

    // choose AF7 for USART2 in Alternate Function registers
    GPIOA->AFR[0] |= (0x7 << 8); // for pin 2
    GPIOA->AFR[0] |= (0x7 << 12); // for pin 3

    // usart2 word length M, bit 12
    //USART2->CR1 |= (0 << 12); // 0 - 1,8,n

    // usart2 parity control, bit 9
    //USART2->CR1 |= (0 << 9); // 0 - no parity

    // usart2 tx enable, TE bit 3
    USART2->CR1 |= (1 << 3);

    // usart2 rx enable, RE bit 2
    USART2->CR1 |= (1 << 2);

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

    // enable usart2 - UE, bit 13
    USART2->CR1 |= (1 << 13);

    NVIC_EnableIRQ(USART2_IRQn);

    // now that everything is ready,
    // enable tx interrupt and let it push out the buffer
    tx_complete = 0;
    bufpos = 0;
    // enable usart2 tx interrupt
    USART2->CR1 |= (1 << 7);

    while(1)
    {
        // restart transmission
        if (tx_complete) {
            bufpos = 0;
            // enable usart2 tx interrupt
            USART2->CR1 |= (1 << 7);
        }
    }

    return 0;
}
