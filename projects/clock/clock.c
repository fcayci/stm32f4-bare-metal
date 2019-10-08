/*
 * clock.c
 *
 * author: Furkan Cayci
 * description:
 *    example code to show how to run clock at different
 *    frequencies and different sources and change it on the fly.
 *    some of the optimizations might be missing. Use with caution.
 *
 *    Also note that with different optimization options, blinking speeds will
 *      vary since a dummy counter is in use.
 *
 *    Three different clock sources can derive the system clock (SYSCLK):
 *      HSI -> 16 Mhz internal oscillator clock
 *      HSE ->  8 Mhz external oscillator clock
 *      PLL ->  PLLVCO, input frequency must be between 1 - 2 Mhz.
 *        2 Mhz for limiting jitter. Division factors M, N, P, and Q.
 *
 *    Secondary clocks:
 *      32 kHz internal RC (LSI RC) which drives watchdog and optionally RTC
 *      32.768 kHz external crystal (LSE RC) which optionally drives RTC
 *
 *    All peripherals are derived from SYSCLK except:
 *      USB OTG FS clock -> 48 Mhz (derived from PLL48CLK)
 *      RNG clock        -> 48 Mhz (derived from PLL48CLK)
 *      SDIO clock       -> 48 Mhz (derived from PLL48CLK)
 *      I2S clock -> Can be derived from a separate PLLI2C clock or I2C_CKIN pin
 *
 *    Main PLL clock can be configured by the following formula:
 *      * choosing HSI or HSE as the source:
 *      * fVCO = source_clock * (N / M)
 *      * Main PLL = fVCO / P
 *      * PLL48CLK = fVCO / Q
 *
 *    source_clock: can be HSI or HSE
 *      bit 22 on RCC_PLLCFGR register. 1:HSE, 0:HSI
 *
 *    Q: can be between 2 <= Q <= 15
 *      bits 27:24 on RCC_PLLCFGR register
 *
 *    P: can be 2, 4, 6, or 8
 *      bits 17:16 on RCC_PLLCFGR register
 *
 *    N: can be 50 <= N <= 432
 *      bits  14:6 on RCC_PLLCFGR register
 *
 *    M: can be 2 <= M <= 63
 *      bits   5:0 on RCC_PLLCFGR register
 *
 * setup:
 *    uses 4 on-board LEDs
 */

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"

/*************************************************
* function declarations
*************************************************/
void set_sysclk_to_hse(void);
void set_sysclk_to_hsi(void);
void set_sysclk_to_84(void);

int main(void);
void delay(volatile uint32_t s);

void set_sysclk_to_hse(void)
{
    SystemInit();

    /* Enable HSE (CR: bit 16) */
    RCC->CR |= ((uint32_t) (1 << 16));
    /* Wait till HSE is ready (CR: bit 17) */
    while(!(RCC->CR & (1 << 17)));

    /* Configure Flash
     * prefetch enable (ACR:bit 8)
     * instruction cache enable (ACR:bit 9)
     * data cache enable (ACR:bit 10)
     * set latency to 0 wait states (ARC:bits 2:0)
     *   see Table 10 on page 80 in RM0090
     */
    FLASH->ACR = (1 << 8) | (1 << 9) | (1 << 10 ) | (0x0 << 0);

    /* Select the HSE as system clock source, (CFGR:bits 1:0)
     * 0b00 - HSI
     * 0b01 - HSE
     * 0b10 - PLL
     */
    RCC->CFGR &= (uint32_t)~(0x3 << 0);
    RCC->CFGR |= (0x1 << 0);
    /* Wait till the main PLL is used as system clock source (CFGR:bits 3:2) */
    while (!(RCC->CFGR & (uint32_t)(0x1 << 2)));
}

void set_sysclk_to_hsi(void)
{
    /* Reset goes to HSI by default */
    SystemInit();

    /* Configure Flash
     * prefetch enable (ACR:bit 8)
     * instruction cache enable (ACR:bit 9)
     * data cache enable (ACR:bit 10)
     * set latency to 0 wait states (ARC:bits 2:0)
     *   see Table 10 on page 80 in RM0090
     */
    FLASH->ACR = (1 << 8) | (1 << 9) | (1 << 10 ) | (0x0 << 0);
}

/* set sysclock to 84Mhz
 * no need to touch M, N or Q
 * PLL_P needs to be doubled
 */
void set_sysclk_to_84(void)
{
    SystemInit();

    #undef PLL_P
    uint32_t PLL_P = 4;

    /* Enable HSE (CR: bit 16) */
    RCC->CR |= ((uint32_t) (1 << 16));
    /* Wait till HSE is ready (CR: bit 17) */
    while(!(RCC->CR & (1 << 17)));

    /* set voltage scale to 1 for max frequency */
    /* first enable power interface clock (APB1ENR:bit 28) */
    RCC->APB1ENR |= (1 << 28);

    /* then set voltage scale to 1 for max frequency (PWR_CR:bit 14)
     * (0b0) scale 2 for fCLK <= 144 Mhz
     * (0b1) scale 1 for 144 Mhz < fCLK <= 168 Mhz
     */
    PWR->CR |= (1 << 14);

    /* set AHB prescaler to /1 (CFGR:bits 7:4) */
    RCC->CFGR |= (0 << 4);
    /* set ABP low speed prescaler to /4 (APB1) (CFGR:bits 12:10) */
    RCC->CFGR |= (0x5 << 10);
    /* set ABP high speed prescaper to /2 (ABP2) (CFGR:bits 15:13) */
    RCC->CFGR |= (0x4 << 13);

    /* Set M, N, P and Q PLL dividers
     * PLLCFGR: bits 5:0 (M), 14:6 (N), 17:16 (P), 27:24 (Q)
     * Set PLL source to HSE, PLLCFGR: bit 22, 1:HSE, 0:HSI
     */
    RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) |
                   (PLL_Q << 24) | (1 << 22);
    /* Enable the main PLL (CR: bit 24) */
    RCC->CR |= (1 << 24);
    /* Wait till the main PLL is ready (CR: bit 25) */
    while((RCC->CR & (1 << 25)) == 0);
    /* Configure Flash
     * prefetch enable (ACR:bit 8)
     * instruction cache enable (ACR:bit 9)
     * data cache enable (ACR:bit 10)
     * set latency to 2 wait states (ARC:bits 2:0)
     *   see Table 10 on page 80 in RM0090
     */
    FLASH->ACR = (1 << 8) | (1 << 9) | (1 << 10 ) | (0x2 << 0);

    /* Select the main PLL as system clock source, (CFGR:bits 1:0)
     * 0b00 - HSI
     * 0b01 - HSE
     * 0b10 - PLL
     */
    RCC->CFGR &= (uint32_t)~(0x3 << 0);
    RCC->CFGR |= (0x2 << 0);
    /* Wait till the main PLL is used as system clock source (CFGR:bits 3:2) */
    while (!(RCC->CFGR & (uint32_t)(0x2 << 2)));
}

/*************************************************
* main code starts from here
*************************************************/
int main(void)
{
    /* set system clock to 168 Mhz */
    set_sysclk_to_168();

    volatile uint32_t i = 0;

    // enable GPIOD clock
    RCC->AHB1ENR |= (1 << 3);

    GPIOD->MODER &= 0x00FFFFFF;   // Reset bits 31-24 to clear old values
    GPIOD->MODER |= 0x55000000;   // Set MODER bits to 01 (0101 is 5 in hex)

    // Set Pins 12-15 to 1 to turn on all LEDs
    GPIOD->ODR |= 0xF000;

    while(1)
    {
        // increment i every time
        // since we set up to 168Mhz in the
        // beginning we can just start from 1
        // reset when it reaches to 180
        if (i > 179){
            i = 0;
        } else {
            i++;
        }

        switch (i)
        {
            case 0: // 168 Mhz
                set_sysclk_to_168();
                break;

            case 100: // 84 Mhz
                set_sysclk_to_84();
                break;

            case 150: // 16 Mhz
                set_sysclk_to_hsi();
                break;

            case 170: // 8 Mhz
                set_sysclk_to_hse();
                break;

            default:
                break;
        }

        delay(500000);
        GPIOD->ODR ^= 0xF000;
    }

    return 0;
}

// A simple and not accurate delay function
void delay(volatile uint32_t s)
{
    for(s; s>0; s--){
        // Do nothing
    }
}
