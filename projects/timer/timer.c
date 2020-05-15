/*
 * timer.c
 *
 * author: Furkan Cayci
 * description:
 *   blinks LEDs one at a time using timer interrupt
 *   timer2 is used as the source, and it is setup
 *   to run at 10 kHz. LED blinking rate is set to
 *   1 second.
 *
 * timer and timer interrupt setup steps:
 *   1. Enable TIMx clock from RCC
 *   2. Set prescaler for the timer from PSC
 *   3. Set auto-reload value from ARR
 *   4. (optional) Enable update interrupt from DIER bit 0
 *   5. (optional) Enable TIMx interrupt from NVIC
 *   6. Enable TIMx module from CR1 bit 0
 */

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

/*************************************************
* function declarations
*************************************************/
int main(void);

/*************************************************
* timer 2 interrupt handler
*************************************************/
void TIM2_IRQHandler(void)
{
    static uint32_t i = 1;

    // clear interrupt status
    if (TIM2->DIER & 0x01) {
        if (TIM2->SR & 0x01) {
            TIM2->SR &= ~(1U << 0);
        }
    }

    GPIOD->ODR = (i << 12);

    if (i == 0x08) {
        i = 1;
    }
    else {
        i = (i << 1);
    }
}

/*************************************************
* main code starts from here
*************************************************/
int main(void)
{
    /* set system clock to 168 Mhz */
    set_sysclk_to_168();

    // setup LEDs
    RCC->AHB1ENR |= (1 << 3);
    GPIOD->MODER &= 0x00FFFFFF;
    GPIOD->MODER |= 0x55000000;
    GPIOD->ODR = 0;

    // enable TIM2 clock (bit0)
    RCC->APB1ENR |= (1 << 0);

    // Timer clock runs at ABP1 * 2
    //   since ABP1 is set to /4 of fCLK
    //   thus 168M/4 * 2 = 84Mhz
    // set prescaler to 83999
    //   it will increment counter every prescalar cycles
    // fCK_PSC / (PSC[15:0] + 1)
    // 84 Mhz / 8399 + 1 = 10 khz timer clock speed
    TIM2->PSC = 8399;

    // Set the auto-reload value to 10000
    //   which should give 1 second timer interrupts
    TIM2->ARR = 10000;

    // Update Interrupt Enable
    TIM2->DIER |= (1 << 0);

    NVIC_SetPriority(TIM2_IRQn, 2); // Priority level 2
    // enable TIM2 IRQ from NVIC
    NVIC_EnableIRQ(TIM2_IRQn);

    // Enable Timer 2 module (CEN, bit0)
    TIM2->CR1 |= (1 << 0);

    while(1)
    {
        // Do nothing.
    }

    return 0;
}
