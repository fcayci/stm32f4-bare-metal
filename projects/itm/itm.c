/*
 * itm.c
 *
 * author: Furkan Cayci
 * description:
 *    SWV can be used from CubeIDE
 *
 * setup:
 *
 */

#include <stdio.h>
#include "stm32f407xx.h"
#include "system_stm32f4xx.h"

#define LEDDELAY    1000000

/*************************************************
* function declarations
*************************************************/
int main(void);
void delay(volatile uint32_t);

/* _write is a system call that uses to flush out buffers */
// the output is directed to ITM channel 0
// ITM_SendChar function is defined in CMSIS
int _write(int file, char *ptr, int len){
	(void)file;
    int i;
    for (i =0; i<len; i++){
        ITM_SendChar(*ptr++);
    }
    return len;
}

// simply call write
// we need other functions for interger or hex printing
void print(char *ptr, int len){
    _write(0, ptr, len);
}

/*************************************************
* main code starts from here
*************************************************/
int main(void)
{
    set_sysclk_to_168();

    char msg[] = "https://furkan.space/\n";

    while(1)
    {
        delay(LEDDELAY);

        // printf is linked from libc which will have a big footprint
        printf("%s", msg);
        // our custom implementation of a string print function
        print(msg, sizeof(msg));
    }

    return 0;
}

void delay(volatile uint32_t s)
{
    for(s; s>0; s--){
    }
}
