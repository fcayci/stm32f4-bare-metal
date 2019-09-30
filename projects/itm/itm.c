/*
 * itm.c
 *
 * author: Furkan Cayci
 * description:
 *    sends out a string from the swo.
 *    string can be captured using openocd
 *    config file is given within the directory
 *
 *
 * setup:
 *    install openocd and run
 *    openocd -f stm32f4-ocd.cfg
 *    for a few seconds. then quit with Ctrl+C
 *
 *    a new file will be generated called "swo.log"
 *    you can see the message in that file
 */

#include "stm32f407xx.h"
#include "system_stm32f4xx.h"

#define LEDDELAY    100000

/*************************************************
* function declarations
*************************************************/
int main(void);
void delay(volatile uint32_t);

/*************************************************
* main code starts from here
*************************************************/
int main(void)
{
    set_sysclk_to_168();

    // enable GPIOD clock
    RCC->AHB1ENR |= (1 << 3);

    // make leds output
    GPIOD->MODER &= 0x00FFFFFF;
    GPIOD->MODER |= 0x55000000;

    // clear leds
    GPIOD->ODR = 0;

    uint8_t msg[] = "http://furkan.space/\n";
    uint8_t *pmsg = msg;

    uint32_t led = 1;
    while(1)
    {
        delay(LEDDELAY);
        GPIOD->ODR ^= (led << 12);  // Toggle LED

        // Using CMSIS function directly
        ITM_SendChar(*pmsg++);

        // or manually (same thing)
        // while (ITM->PORT[0].u8 == 0);
        // ITM->PORT[0].u8 = *pmsg++;

        // reset the message
        if (*pmsg == '\0') {
            pmsg = &msg[0];
            led = (led * 2) % 15;
        }
    }

    return 0;
}

// A simple and not accurate delay function
// that will change the speed based on the optimization settings
void delay(volatile uint32_t s)
{
    for(s; s>0; s--){
        // Do nothing
    }
}
