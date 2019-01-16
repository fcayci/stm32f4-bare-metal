#ifndef __SYSTEM_STM32F4XX_H
#define __SYSTEM_STM32F4XX_H

#ifdef __cplusplus
 extern "C" {
#endif

#define VECT_TAB_OFFSET  0x00

/* Clock PLLs for 407 chip */
#ifdef STM32F407xx
#define PLL_M	8
#define PLL_Q 	7
#define PLL_N 	336
#define PLL_P 	2
#endif

#ifdef __cplusplus
}
#endif

#endif /*__SYSTEM_STM32F4XX_H */
