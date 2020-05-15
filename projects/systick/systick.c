/*
 * systick.c
 *
 * author: Furkan Cayci
 * description:
 *    blinks LEDs using systick timer
 *      at exactly 1 second intervals
 *
 */

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

/*************************************************
* function declarations
*************************************************/
void init_systick(uint32_t s);
int main(void);
void delay_ms(uint32_t);

/*************************************************
* variables
*************************************************/
static volatile uint32_t tDelay;
extern uint32_t SystemCoreClock;

/*************************************************
* default interrupt handler
*************************************************/
void SysTick_Handler(void)
{
    if (tDelay != 0)
    {
        tDelay--;
    }
}

/*************************************************
* initialize SysTick
*************************************************/
void init_systick(uint32_t s)
{
    // Clear CTRL register
    SysTick->CTRL = 0x00000;
    // Main clock source is running with HSI by default which is at 8 Mhz.
    // SysTick clock source can be set with CTRL register (Bit 2)
    // 0: Processor clock/8 (AHB/8)
    // 1: Processor clock (AHB)
    SysTick->CTRL |= (1 << 2);
    // Enable callback (bit 1)
    SysTick->CTRL |= (1 << 1);
    // Load the value
    SysTick->LOAD = (uint32_t)(s-1);
    // Set the current value to 0
    SysTick->VAL = 0;
    // Enable SysTick (bit 0)
    SysTick->CTRL |= (1 << 0);
}

/*************************************************
* main code starts from here
*************************************************/
int main(void)
{
    /* optinally set system clock to 168 Mhz */
    //set_sysclk_to_168();

    // SystemCoreClock should be configured correctly
    // depending on the operating frequency
    // SysTick runs at the same speed, so if we generate
    // a tick every clock_speed/1000 cycles, we can generate
    // a 1 ms tick speed.
    init_systick(SystemCoreClock/1000);

    // Similar function is given with CMSIS
    //SysTick_Config(SystemCoreClock/1000);

    // setup LEDs and turn them on
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER &= ~(0xFFU << 24);
    GPIOD->MODER |=  (0x55  << 24);
    GPIOD->ODR |= (0xF << 12);

    while(1)
    {
        delay_ms(1000); // 1 sec delay
        GPIOD->ODR ^= (0xF << 12);  // Toggle LEDs
    }

    return 0;
}

/*
 * Millisecond delay function.
 */
void delay_ms(uint32_t s)
{
    tDelay = s;
    while(tDelay != 0);
}
