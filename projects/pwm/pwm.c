/*
 * pwm.c
 *
 * author: Furkan Cayci
 * description:
 *    fades green LED using pwm functionality on timer4
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

#define PWMPERIOD 100
#define SAMPLE 100

/*************************************************
* function declarations
*************************************************/
int main(void);

/*************************************************
* timer 4 interrupt handler
*************************************************/
void TIM4_IRQHandler(void)
{
	static uint32_t t = 0;
	static uint16_t duty = 0;

	// clear interrupt status
	if (TIM4->DIER & 0x01) {
		if (TIM4->SR & 0x01) {
			TIM4->SR &= ~(1U << 0);
		}
	}

	duty =(uint16_t)(PWMPERIOD/2.0 * (sin(2*M_PI*(double)t/(SAMPLE)) + 1.0));
	++t;
	if (t == SAMPLE) t = 0;
    // set new duty cycle
    TIM4->CCR1 = duty;
}

/*************************************************
* main code starts from here
*************************************************/
int main(void)
{
    /* set system clock to 168 Mhz */
    set_sysclk_to_168();

    // enable GPIOD clock
    RCC->AHB1ENR |= (1 << 3);
    GPIOD->MODER |= (0x2 << 24);   // pin 12 af mode 10
    // Choose Timer4 as Alternative Function for pin 12 led
    GPIOD->AFR[1] |= (0x2 << 16);
    // enable TIM4 clock (bit2)
    RCC->APB1ENR |= (1 << 2);

    // fCK_PSC / (PSC[15:0] + 1)
    // 84 Mhz / 8399 + 1 = 10 khz timer clock speed
    TIM4->PSC = 8399;
    // set period
    TIM4->ARR = PWMPERIOD;
    // set duty cycle on channel 1
    TIM4->CCR1 = 1;

    // enable channel 1 in capture/compare register
    // set oc1 mode as pwm (0b110 or 0x6 in bits 6-4)
    TIM4->CCMR1 |= (0x6 << 4);
    // enable oc1 preload bit 3
    TIM4->CCMR1 |= (1 << 3);
    // enable capture/compare ch1 output
    TIM4->CCER |= (1 << 0);
    // enable update interrupt
    TIM4->DIER |= (1 << 0);

    NVIC->IP[TIM4_IRQn] = 0x20; // Priority level 2
    // enable TIM4 IRQ from NVIC
    NVIC_EnableIRQ(TIM4_IRQn);

    // Enable Timer 4 module (CEN, bit0)
    TIM4->CR1 |= (1 << 0);

    while(1)
    {
        // Do nothing.
    }

    return 0;
}
