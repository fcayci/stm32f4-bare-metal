#ifndef __DEBUG_H
#define __DEBUG_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "core_cm4.h"

#define PRINTBUF 20

// DWT control register
// CYCCNTENA (bit 0) Enable cycle counter. It should be initialized before enabling.
__STATIC_INLINE void init_cycles()
{
    // Debug Exception and Monitor Control Register
    // TRCENA (bit 24) - Trace system enable; to use DWT, ETM, ITM and TPIU
    //CoreDebug->DEMCR |= (1 << 24);
    DWT->CYCCNT = 0;      // initialize to 0
    DWT->CTRL = (1 << 0); // enable
}

// Elapsed cycle count can be read from CYCCNT register at any time
__STATIC_INLINE uint32_t read_cycles()
{
    return DWT->CYCCNT;
}

// direct _write system call to ITM
__STATIC_INLINE int _write(int file, char *ptr, int len)
{
    (void)file;
    for (int DataIdx = 0; DataIdx < len; DataIdx++) {
        ITM_SendChar(*ptr++);
    }
    return len;
}

__STATIC_INLINE void print(char *s)
{
    int len = 0;
    while (s[len++] != '\0');
    _write(0, s, len);
}

__STATIC_INLINE void printuint(unsigned int num)
{
    char buf[PRINTBUF] = {0};
    int i = PRINTBUF-1;
    if (num) {
        while(num) {
            buf[i--] = (char) ((num % 10) + 48);
            num = (unsigned int)(num / 10);
        }
    } else {
        buf[i--] = '0';
    }

    (void)_write(0, &buf[i], PRINTBUF-i);
}

__STATIC_INLINE void printint(int num)
{
    char buf[PRINTBUF] = {0};
    int i = PRINTBUF-1;
    if (num < 0){
        num = -num;
        while(num) {
            buf[i--] = (char) ((num % 10) + 48);
            num = (int)(num / 10);
        }
        buf[i--] = '-';
    }
    else if (num > 0) {
        while(num) {
            buf[i--] = (char) ((num % 10) + 48);
            num = (int)(num / 10);
        }
    }
    else {
        buf[i--] = '0';
    }

    (void)_write(0, &buf[i], PRINTBUF-i);
}

__STATIC_INLINE void printhex(unsigned int num)
{
    char buf[] = "00000000";
    unsigned int i = 7;
    while(num) {
        unsigned int rem = num % 16;
        if (rem < 10)
            buf[i--] = (char) (rem + 48);
        else
            buf[i--] = (char) (rem + 55);
        num = (unsigned int)(num / 16);
    }

    (void)_write(0, buf, 8);
}

#ifdef __cplusplus
}
#endif

#endif /*__DEBUG_H */
