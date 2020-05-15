/*
 * math.c
 *
 * author: Furkan Cayci
 * description:
 *    implements a simple sine function to test math library operation
 *    processor clock is set to max (168 Mhz)
 *    math library needs to be linked.
 */

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include <math.h>

/*************************************************
* function declarations
*************************************************/
void init_systick(uint32_t s);
int main(void);
void delay_ms(uint32_t s);

/*************************************************
* variables
*************************************************/
static volatile uint32_t tDelay;
extern uint32_t SystemCoreClock;

/*************************************************
* SysTick interrupt handler
*************************************************/
void SysTick_Handler(void)
{
    if (tDelay != 0)
    {
        tDelay--;
    }
}

/*************************************************
* main code starts from here
*************************************************/
int main(void)
{
    /* set system clock to 168 Mhz */
    set_sysclk_to_168();
    // configure SysTick to interrupt at 1khz freq.
    // make sure to include SysTick Handler function
    SysTick_Config(SystemCoreClock/1000);

    uint32_t i = 0;
    double x;

    // setup LEDs
    RCC->AHB1ENR |= (1 << 3);
    GPIOD->MODER &= ~(0xFFU << 24);
    GPIOD->MODER |=  (0x55  << 24);

    while(1)
    {
        x = 4 * ( sin(2 * M_PI * (float)i / 1000) + 1);

        if (x > 4)      GPIOD->ODR = (0xF << 12);
        else if (x > 3) GPIOD->ODR = (0x7 << 12);
        else if (x > 2) GPIOD->ODR = (0x3 << 12);
        else if (x > 1) GPIOD->ODR = (0x1 << 12);
        else            GPIOD->ODR = (0x0 << 12);

        if (1000 == i++) {
            i = 0;
        }
        delay_ms(1);
    }

    return 0;
}

/*
 * Millisecond delay function
 */
void delay_ms(volatile uint32_t s)
{
    tDelay = s;
    while(tDelay != 0);
}
