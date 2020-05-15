/*
 * uart-tx-int.c
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
 *   9. enable uart interrupt from NVIC
 *   10.enable tx interrupt and disable when the buffer
 *      transmission is complete
 */

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

/*************************************************
* function declarations
*************************************************/
int main(void);

const uint8_t msg[] = "https://furkan.space/\n\r";
volatile int bufpos = 0;

void USART2_IRQHandler(void)
{
    // check if the source is transmit interrupt
    if (USART2->SR & (1 << 7)) {

        if (bufpos == sizeof(msg)) {
            // buffer is flushed out, disable tx interrupt
            bufpos = 0;
            USART2->CR1 &= ~(1U << 7);
        }
        else {
            // flush ot the next char in the buffer
            USART2->DR = msg[bufpos++];
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
    // USART2 TX and RX pins are PA2 and PA3 respectively
    GPIOA->MODER &= ~(0xFU << 4); // Reset bits 4:5 for PA2 and 6:7 for PA3
    GPIOA->MODER |=  (0xAU << 4); // Set   bits 4:5 for PA2 and 6:7 for PA3 to alternate mode (10)

    // choose AF7 for USART2 in Alternate Function registers
    GPIOA->AFR[0] |= (0x7 << 8); // for pin A2
    GPIOA->AFR[0] |= (0x7 << 12); // for pin A3

    // USART2 word length M, bit 12
    //USART2->CR1 |= (0 << 12); // 0 - 1,8,n

    // USART2 parity control, bit 9
    //USART2->CR1 |= (0 << 9); // 0 - no parity

    // USART2 TX enable, TE bit 3
    USART2->CR1 |= (1 << 3);

    // USART2 RX enable, RE bit 2
    //USART2->CR1 |= (1 << 2);

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

    NVIC_SetPriority(USART2_IRQn, 1); // Priority level 1
    NVIC_EnableIRQ(USART2_IRQn);

    // now that everything is ready,
    // enable tx interrupt and let it push out
    USART2->CR1 |= (1 << 7);

    while(1)
    {
        for(volatile int i=0; i<10000000; i++); // a long wait. should be more than enough the push out the buffer
        // restart transmission by enabling usart2 tx interrupt
        USART2->CR1 |= (1 << 7);
    }

    return 0;
}
