/*
 * sleepy.c
 *
 * author: Furkan Cayci
 * description:
 *   three LEDs are configured.
 *   green LED is set to toggle on main while loop
 *   red LED is set to toggle on timer interrupt
 *   orange LED is set toggle on button press (external interrupt)
 *   processor is put to sleep mode initially (so no green LED toggling)
 *   red LED toggles normally (peripherals are on)
 *   each time the interrupt happens, the processor goes back to sleep (sleeponexit bit)
 *   after pressing the button 10 times, sleeponexit bit is reset causing the processor
 *   to not go to sleep, so green LED starts blinking
 *   when in deepsleep, timer should also not run (no red blinking)
 *
 * sleep setup steps:
 *   setup external interrupt / event
 *   set sleepdeep bit if desired
 *   set sleeponexit bit if desired (return to sleep after interrupt)
 *   execute 'wfi' or 'wfe' instruction
 *   __WFI() and __WFE() in CMSIS
 */

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

#define LEDDELAY    1000000

/*************************************************
* function declarations
*************************************************/
int main(void);
void delay(volatile uint32_t);

void EXTI0_IRQHandler(void)
{
    static int i = 0;
    // Check if the interrupt came from exti0
    if (EXTI->PR & (1 << 0)){
        GPIOD->ODR ^= (1 << 13); // Toggle Orange LED
        i++;

        /* an awful debouncer */
        for(int j=0; j<1000000; j++);

        // after 5 button press, put the processor in deepsleep
        if (i == 5) {
            SCB->SCR |= (1U << 2); // Enable SleepDeep bit
        }
        // after 10 button press return back to normal
        else if (i == 10) {
            i = 0;
            SCB->SCR ^= (1 << 1); // Toggle SleepOnExit bit
            SCB->SCR &= ~(1U << 2); // Disable SleepDeep bit
            // after exiting deepsleep, processor goes back to HSI clock (16Mhz)
        }
        // Clear pending bit
        EXTI->PR = (1 << 0);
    }
}

/*************************************************
* timer 2 interrupt handler
*************************************************/
void TIM2_IRQHandler(void)
{
    // This handler should execute when in normal sleep
    // Should not execute when in deep sleep
    TIM2->SR = ~(1U << 0); // Clear pending bit
    GPIOD->ODR ^= (1 << 14); // Toggle Red LED
}

int main(void)
{

    __disable_irq();

    set_sysclk_to_168();

    /* Enable GPIOD clock (AHB1ENR: bit 3) */
    RCC->AHB1ENR |= (1 << 3);

    /* Make Pins12,13,14,15 output */
    GPIOD->MODER &= ~(0xFFU << 24);
    GPIOD->MODER |=  (0x55U << 24);

    GPIOD->ODR |= (1 << 12);

    /* Setup external interrupt for button at PA0 */
    // enable GPIOA clock (AHB1ENR: bit 0)
    RCC->AHB1ENR |= (1 << 0);
    /* Make Pin 0 input (MODER: bits 1:0) */
    GPIOA->MODER &= ~(3U << 0);   // Reset bits 0-1 to clear old values

    // enable SYSCFG clock (APB2ENR: bit 14)
    RCC->APB2ENR |= (1 << 14);

    SYSCFG->EXTICR[0] |= 0x00000000; // Write 0000 to map PA0 to EXTI0
    // Choose either rising edge trigger (RTSR) or falling edge trigger (FTSR)
    EXTI->RTSR |= 0x00001;   // Enable rising edge trigger on EXTI0
    // Mask the used external interrupt numbers.
    EXTI->IMR |= 0x00001;    // Mask EXTI0
    // Set Priority for each interrupt request
    NVIC_SetPriority(EXTI0_IRQn, 1); // Priority level 1
    // enable EXT0 IRQ from NVIC
    NVIC_EnableIRQ(EXTI0_IRQn);

    /* Setup timer interrupt for periodic updates */

    // enable TIM2 clock (bit0)
    RCC->APB1ENR |= (1 << 0);
    // 84 Mhz / 8399 + 1 = 10 khz timer clock speed
    TIM2->PSC = 8399;
    // Set the auto-reload value to 10000
    //   which should give 1 second timer interrupts
    TIM2->ARR = 10000;
    // Update Interrupt Enable
    TIM2->DIER |= (1 << 0);
    // enable TIM2 IRQ from NVIC
    NVIC_SetPriority(TIM2_IRQn, 2); // Priority level 2
    NVIC_EnableIRQ(TIM2_IRQn);
    // Enable Timer 2 module (CEN, bit0)
    TIM2->CR1 |= (1 << 0);

    // enable interrupts
    __enable_irq();

    // Note about DeepSleep mode, the oscillator will be selected as the internal one
    // (HSI) upon waking up
    SCB->SCR |= (1 << 1); // SleepOnExit

    // sleep
    __WFI(); // wait for interrupt

    while(1)
    {
        // when sleeping, this LED should not blink
        delay(LEDDELAY);
        GPIOD->ODR ^= (1 << 12);  // Toggle Green LED
    }

    return 0;
}

void delay(volatile uint32_t s)
{
    for(; s>0; s--){}
}
