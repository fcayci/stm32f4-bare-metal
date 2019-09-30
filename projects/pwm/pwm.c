/*
 * pwm.c
 *
 * author: Furkan Cayci
 * description:
 *    fades one LED using pwm functionality on timer4
 *    the LEDs are connected to GPIOD 12-15
 *    which has timer4 capability.
 *    GPIOD 12 is connected to timer4 channel1
 *
 * setup:
 *    uses 1 on-board LED
 */

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include <math.h>

/*************************************************
* function declarations
*************************************************/
int main(void);

const uint32_t period = 5000;

/*************************************************
* timer 4 interrupt handler
*************************************************/
void TIM4_IRQHandler(void)
{
    double x;
    static uint32_t t = 0;
    const uint32_t f = 10*period;
    // generate new value each time
    x = f/2 * ( sin(2 * M_PI * (float)t / (float)f) + 1);
    if (t == f)
    {
        t = 0;
    } else
    {
        ++t;
    }

    // set new duty cycle
    TIM4->CCR1 = (uint16_t)(x/10);
}

/*************************************************
* main code starts from here
*************************************************/
int main(void)
{
    /* set system clock to 168 Mhz */
    set_sysclk_to_168();

    uint32_t duty = period/2;

    // enable GPIOD clock
    RCC->AHB1ENR |= (1 << 3);
    GPIOD->MODER &= 0x00FFFFFF;   // Reset bits 31-24 to clear old values
    GPIOD->MODER |= 0x02000000;   // Set pin 12 to alternate func. mode (0b10)

    // Choose Timer4 as Alternative Function for pin 12 led
    GPIOD->AFR[1] |= (0x2 << 16);

    // enable TIM4 clock (bit2)
    RCC->APB1ENR |= (1 << 2);

    // set prescaler to 167
    //   it will increment counter every prescalar cycles
    // fCK_PSC / (PSC[15:0] + 1)
    // 168 Mhz / 167 + 1 = 1 Mhz timer clock speed
    TIM4->PSC = 167;

    // set period
    TIM4->ARR = period;

    // set duty cycle on channel 1
    TIM4->CCR1 = duty;

    // enable channel 1 in capture/compare register
    // set oc1 mode as pwm (0b110 or 0x6 in bits 6-4)
    TIM4->CCMR1 |= (0x6 << 4);
    // enable oc1 preload bit 3
    TIM4->CCMR1 |= (1 << 3);

    // enable capture/compare ch1 output
    TIM4->CCER |= (1 << 0);

    // enable interrupt CC1IE (bit 1)
    TIM4->DIER |= (1 << 1);

    // enable TIM4 IRQ from NVIC
    NVIC_EnableIRQ(TIM4_IRQn);

    // Enable Timer 4 module (CEN, bit0)
    TIM4->CR1 |= (1 << 0);

    while(1)
    {
        // Do nothing. let timer handler do its magic
    }

    return 0;
}
