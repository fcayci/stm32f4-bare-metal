/*
 * wwdg.c
 *
 * author: Furkan Cayci
 * description:
 *   demonstrates the operation of window watchdog timer
 *    sets up the wwdg to around 24 ms max
 *    and fails to update it at some point to show the
 *    reset operation
 *
 * setup:
 *   four on-board LEDs
 */

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

/*************************************************
* function declarations
*************************************************/
int main(void);
void dummy_function(volatile uint32_t s);

/*************************************************
* window watchdog interrupt handler
*************************************************/
void WWDG_IRQHandler(void)
{
    for (;;);  // FIXME: replace me
}

/*************************************************
* main code starts from here
*************************************************/
int main(void)
{
    // set system clock to 168 Mhz
    // AHB is /1
    // APB1 is /4
    // APB2 is /2
    set_sysclk_to_168();
    // Set Bit 3 to enable GPIOD clock in AHB1ENR
    RCC->AHB1ENR |= (1 << 3);

    GPIOD->MODER &= 0x00FFFFFF;   // Reset bits 31-24 to clear old values
    GPIOD->MODER |= 0x55000000;   // Set MODER bits to 01 (0101 is 5 in hex)

    // flash all LEDs to demonstrate a reset
    GPIOD->ODR |= 0xF000;
    dummy_function(10000000);
    GPIOD->ODR = 0x0000;

    /* set up Window Watchdog (WWDG) */

    // enable window watchdog clock from RCC (bit 11 on APB1ENR)
    RCC->APB1ENR |= (1 << 11);

    // tWWDG = tPCLK1(ms) * 4096 * 2**WDGTB[1:0] * (T[5:0] + 1) ms
    // tWWDG : WWDG timeout
    // tPCLK1 : APB1 clock period in ms

    // in other words T[6;0] is the watchdog counter, and it gets
    // decremented every 4096 * 2**WDGTB[1:0] * PCLK1 cycles
    // it will generate a reset when it rolls over from 0x40 to 0x3F

    // for APB1 frequency of 42Mhz
    // let's go with prescaler of 2 (WDGTB alue)
    // tWWDGmax = 1/(42000) * 4096 * 2**2 * 1 = 390 us
    // tWWDGmin = 1/(42000) * 4096 * 2**2 * 64 = 24.9 ms

    // Configure watchdog
    // EWI - Early wakup interrupt (bit 9)

    // WDGTB[1:] - timer base (bits 8:7)
    WWDG->CFR |= (0x2 << 7);

    // W[6:0] - window value (bits 6:0)
    // can reset watchdog after counter reaches down to 0x70
    WWDG->CFR |= (0x70);

    // Enable WDGA (bit 7 - activation bit) and
    //   set counter to 7F to start watchdog
    WWDG->CR |= (0xFF);


    // blink an LED ten times to show the operation
    // reset watchdog every time
    for (int i=0; i<20; i++){
        GPIOD->ODR ^= 0x8000;  // Toggle blue LED
        // since watchdog timer is set to a max of 24.9 ms
        // 100k comes up to around 6 ms wait time, so should be fine
        // and add repeat this 10 times to see the led blinking
        for (int j=0; j<10; j++){
            // wait for some time
            dummy_function(100000);
            // feed the dog
            WWDG->CR |= (0x7F); // refresh the T bits contents
        }
    }

    // here watchdog will kick in and reset the board
    // because it is not fed anymore
    while(1)
    {
        dummy_function(1000);
        GPIOD->ODR ^= 0x1000;  // Toggle green LED
    }

    return 0;
}

// just a dummy function to spend some time
void dummy_function(volatile uint32_t s)
{
    for (s;s>0;s--);
}

