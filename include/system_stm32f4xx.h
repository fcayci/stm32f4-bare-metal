#ifndef __SYSTEM_STM32F4XX_H
#define __SYSTEM_STM32F4XX_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx.h"

void _init_data(void);
void Reset_Handler(void);
void reset_clock(void);
void set_sysclk_to_168(void);
/* bring main */
extern int main(void);

#ifdef __cplusplus
}
#endif

#endif /*__SYSTEM_STM32F4XX_H */
