/*
 * external.c
 *
 * author: Furkan Cayci
 * description:
 *   connects the push button located at PA0 to external interrupt 0
 *   lights up the next LED in each press.
 *   Does not worry about debouncing, so will move more then 1 LED most
 *   of the time.
 *
 * external interrupt setup steps:
 *   1. Enable relevant GPIOx port clock from RCC
 *   2. Enable SYSCLK clock from RCC
 *   3. Tie the pins(buttons) to EXTIx using EXTICRx
 *   4. Choose the trigger edge using RTSR or FTSR
 *   5. Mask the selected EXTIs from IMR
 *   6. Set priority for the interrupts from IP
 *   7. Enable the EXTIx interrupt from NVIC
 */

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

/*************************************************
* function declarations
*************************************************/
int main(void);

/*************************************************
* external interrupt handler
*************************************************/
void EXTI0_IRQHandler(void)
{
    static uint16_t i = 1;

    // Check if the interrupt came from exti0
    if (EXTI->PR & (1 << 0)){
        GPIOD->ODR = (uint16_t)(i << 12);

        /* wait little bit */
        for(int j=0; j<1000000; j++);

        if (8 == i) {
            i = 1;
        } else {
            i = (uint16_t)(i << 1);
        }

        // Clear pending bit
        EXTI->PR = (1 << 0);
    }
}

/*************************************************
* main code starts from here
*************************************************/
int main(void)
{
    /* set system clock to 168 Mhz */
    set_sysclk_to_168();

    /* setup LEDs */
    // enable GPIOD clock (AHB1ENR: bit 3)
    RCC->AHB1ENR |= (1 << 3);
    GPIOD->MODER &= 0x00FFFFFF;   // Reset bits 31-24 to clear old values
    GPIOD->MODER |= 0x55000000;   // Write 01 for all 4 leds to make them output

    /* set up button */
    // enable GPIOA clock (AHB1ENR: bit 0)
    RCC->AHB1ENR |= (1 << 0);
    GPIOA->MODER &= 0xFFFFFFFC;   // Reset bits 0-1 to clear old values
    GPIOA->MODER |= 0x00000000;   // Make button an input

    // enable SYSCFG clock (APB2ENR: bit 14)
    RCC->APB2ENR |= (1 << 14);

    /* tie push button at PA0 to EXTI0 */
    // EXTI0 can be configured for each GPIO module.
    //   EXTICR1: 0b XXXX XXXX XXXX 0000
    //               pin3 pin2 pin1 pin0
    //
    //   Writing a 0b0000 to pin0 location ties PA0 to EXT0
    SYSCFG->EXTICR[0] |= 0x00000000; // Write 0000 to map PA0 to EXTI0

    // Choose either rising edge trigger (RTSR) or falling edge trigger (FTSR)
    EXTI->RTSR |= 0x00001;   // Enable rising edge trigger on EXTI0

    // Mask the used external interrupt numbers.
    EXTI->IMR |= 0x00001;    // Mask EXTI0

    // Set Priority for each interrupt request
    NVIC_SetPriority(EXTI0_IRQn, 1); // Priority level 1

    // enable EXT0 IRQ from NVIC
    NVIC_EnableIRQ(EXTI0_IRQn);

    while(1)
    {
    }

    return 0;
}
