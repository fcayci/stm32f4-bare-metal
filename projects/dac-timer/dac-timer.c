/*
 * dac.c
 *
 * author: Furkan Cayci
 * description:
 *   uses DAC channel 1 connected to PA4 to generate
 *   a ramp waveform using timer2 as trigger source
 *
 * setup:
 *   1. set PA4 an analog mode
 *   2. enable DAC channel 1
 *   3. enable trigger
 *   4. choose timerx trigger as source
 *   5. update the value
 *   6. setup timerx
 *   7. setup timerx master mode with update TRGO
 *   8. enable interrupts
 */

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include <math.h>

/*************************************************
* function declarations
*************************************************/
void TIM2_IRQHandler(void);
int main(void);

volatile uint32_t dac_value = 0xD00;

/*************************************************
* timer2 interrupt handler
*************************************************/
void TIM2_IRQHandler(void)
{
    // Clear pending bit first
    TIM2->SR = (uint16_t)(~(1 << 0));

    // update the new value, let timer trgo handle dac
    dac_value += 0x10;
    DAC->DHR12R1 = dac_value;
    // for manual software trigger (not needed)
    //DAC->SWTRIGR |= (1 << 0 ); // trigger ch1
}

/*************************************************
* main code starts from here
*************************************************/
int main(void)
{
    /* set system clock to 168 Mhz */
    set_sysclk_to_168();

    /*****************************
     ********* DAC SETUP *********
     *****************************/

    // enable GPIOA clock, bit 0 on AHB1ENR
    RCC->AHB1ENR |= (1 << 0);

    GPIOA->MODER &= 0xFFFFFCFF; // Reset bits 8-9 to clear old values
    GPIOA->MODER |= 0x00000300; // Set pin 4 to analog mode (0b11)

    // enable DAC clock, bit 29 on APB1ENR
    RCC->APB1ENR |= (1 << 29);

    DAC->CR |= (1 << 0); // enable DAC channel 1
    DAC->CR |= (0 << 1); // enable DAC ch1 output buffer
    DAC->CR |= (1 << 2); // enable trigger
    // let tim2 handler the dac updates
    // change 4 to 7 for manual updates
    DAC->CR |= (4 << 3); // choose tim2 TRGO as source (0b100)

    // set output to Vref * (dac_value/0xFFF)
    DAC->DHR12R1 = dac_value;
    // for manual software trigger
    //DAC->SWTRIGR |= (1 << 0 ); // trigger ch1

    /*****************************
     ******** TIMER SETUP ********
     *****************************/

    // enable TIM2 clock (bit0)
    RCC->APB1ENR |= (1 << 0);

    // set timer2 prescaler
    TIM2->PSC = 83;
    // Set the auto-reload
    TIM2->ARR = 10;

    // Update Interrupt Enable
    TIM2->DIER |= (1 << 0);

    // Choose update as master mode (0b010)
    TIM2->CR2 |= (0x2 << 4);

    // enable TIM2 IRQ from NVIC
    NVIC_EnableIRQ(TIM2_IRQn);

    // Enable Timer 2 module (CEN, bit0)
    TIM2->CR1 |= (1 << 0);

    while(1)
    {
        // for manual software triggering (not needed)
        // dac_value += 0x10;
        // DAC->DHR12R1 = dac_value;
        // DAC->SWTRIGR |= (1 << 0 ); // trigger ch1
        // for (i=0; i<1000; i++);
    }

    return 0;
}
