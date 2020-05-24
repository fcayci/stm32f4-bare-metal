/*
 * pendsv.c
 *
 * author: Furkan Cayci
 *
 * description:
 *  PendSV (Pended Service Call) is an exception to help with OS operation.
 *  It is set by software, and it enters handler whenever there is no higher
 *  priority interrupt pending.
 *  We can however utilize it to execute lower priority section of the
 *  interrupt routines if it requires a lot of time to complete. First part
 *  of the routine can handle the interrupt with high priority, then pend
 *  PendSV to handle the rest of the process whenever there is no other
 *  interrupts running. In this example we have external interrupt with
 *  highest priority, and timer interrupt with normal priority. External
 *  calls PendSV to handle the rest of the process so that timer can operate
 *  normally without any delay.
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
    // Check if the interrupt came from exti0
    if (EXTI->PR & (1 << 0)){
        // Execute the critical part that needs to
        // be executed with high priority
        GPIOD->ODR ^= (1 << 13); // orange led

        // Set pending service call to execute the rest
        //  with lowest priority so that it does not
        //  block rest of the interrupts
        SCB->ICSR |= SCB_ICSR_PENDSVSET_Msk;

        // Clear pending bit
        EXTI->PR = (1 << 0);
    }
}

/*************************************************
* timer 2 interrupt handler
*************************************************/
void TIM2_IRQHandler(void)
{
    // clear interrupt status
    if (TIM2->DIER & 0x01) {
        if (TIM2->SR & 0x01) {
            TIM2->SR &= ~(1U << 0);
        }
    }

    // toggle leds
    GPIOD->ODR ^= (1 << 12);
    GPIOD->ODR ^= (1 << 14);
}


/*************************************************
* pendsv interrupt handler
*************************************************/
void PendSV_Handler(void)
{
    // remaining routine to be handled with low priority
    GPIOD->ODR |= (1 << 15); // blue
    // example routine that takes time
    for(volatile int i=0; i<1000000; i++);
    GPIOD->ODR &= ~(1U << 15); // blue
}

/*************************************************
* main code starts from here
*************************************************/
int main(void)
{
    set_sysclk_to_168();
    /********************************
     * setup LEDs - GPIOD 12,13,14,15
     *******************************/
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    GPIOD->MODER &= ~(0xFFU << 24);
    GPIOD->MODER |= (0x55 << 24);
    GPIOD->ODR    = (1 << 12);

    /********************************
     * setup Timer 2 - 1s, mid priority
     *******************************/
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2->PSC = 8399;
    TIM2->ARR = 10000;
    TIM2->DIER |= TIM_DIER_UIE;
    NVIC_SetPriority(TIM2_IRQn, 5);
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->CR1 |= TIM_CR1_CEN;

    /********************************
     * setup External int - high priority
     *  button on GPIOA Pin0
     *******************************/
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    GPIOA->MODER &= ~(0x3U << 24);
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[0] |= (0 << 0);
    EXTI->RTSR |= (1 << 0);
    EXTI->IMR |= (1 << 0);
    NVIC_SetPriority(EXTI0_IRQn, 1);
    NVIC_EnableIRQ(EXTI0_IRQn);

    /********************************
     * setup PendSV - lowest priority
     *******************************/
    NVIC_SetPriority(PendSV_IRQn, 15);

    while(1)
    {
    }

    return 0;
}
