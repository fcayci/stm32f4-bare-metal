/*
 * dac.c
 *
 * author: Furkan Cayci
 * description:
 *   uses DAC channel 1 connected to PA4 to generate
 *   a ramp waveform
 *   DAC is setup to use software trigger
 *   as the source, and it needs to be triggered
 *   manually
 *
 * setup:
 *   1. set PA4 an analog mode
 *   2. enable DAC channel 1
 *   3. enable trigger
 *   4. choose software trigger as source
 *   5. update the value
 *   6. trigger DAC
 */

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include <math.h>

/*************************************************
* function declarations
*************************************************/
int main(void);

/*************************************************
* main code starts from here
*************************************************/
int main(void)
{
    /* set system clock to 168 Mhz */
    set_sysclk_to_168();

    uint32_t dac_value = 0xD00;
    int32_t i = 0;

    // enable GPIOA clock, bit 0 on AHB1ENR
    RCC->AHB1ENR |= (1 << 0);

    GPIOA->MODER &= 0xFFFFFCFF; // Reset bits 8-9 to clear old values
    GPIOA->MODER |= 0x00000300; // Set pin 4 to analog mode (0b11)

    // enable DAC clock, bit 29 on APB1ENR
    RCC->APB1ENR |= (1 << 29);

    DAC->CR |= (1 << 0); // enable DAC channel 1
    DAC->CR |= (0 << 1); // enable DAC ch1 output buffer
    DAC->CR |= (1 << 2); // enable trigger
    DAC->CR |= (7 << 3); // choose sw trigger as source (0b111)

    // set output to Vref * (dac_value/0xFFF)
    DAC->DHR12R1 = dac_value;
    DAC->SWTRIGR |= (1 << 0); // trigger ch1

    while(1)
    {
        dac_value += 0x10;
        DAC->DHR12R1 = dac_value;
        DAC->SWTRIGR |= (1 << 0 ); // trigger ch1
        for (i=0; i<1000; i++);
    }

    return 0;
}
