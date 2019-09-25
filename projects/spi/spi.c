/*
 * spi.c
 *
 * author: Furkan Cayci
 * description:
 *   a motion sensor (three-axis linear accelerometer)
 *   is connected through the SPI port.
 *   lights up LEDs based on the tilt values
 *
 *   LIS302DL is present on the STM32F4 Discovery board
 *     rev. MB997B and
 *   LIS3DSH is present on the rev. MB997C
 *   this code is tested on rev. B
 *     PA5 - SPI1 SCK
 *     PA6 - SPI1 MISO
 *     PA7 - SPI1 MOSI
 *     PE3 - SPI CS
 *     PE0 - INT1 / DRDY
 *     PE1 - INT2
 *
 * setup:
 *    These stesp are for SPI master mode
 *    0. configure pins alternate mode and choose the
 *       correct AF number (5 for SPI1)
 *    1. set BR[2:0] for serial clock baud rate (SPI_CR1)
 *    2. set DFF bit for 8/16 bit data format
 *    3. select CPOL and CPHA bits for data/clock relationship
 *    4. LSBFIRST bit for frame format
 *    5. set SSM bit for software slave management
 *    6. set SSI bit (important step)
 *    7. do not choose TI mode
 *    8. choose master mode MSTR
 *    9. enable SPI
 */

#include "stm32f4xx.h"
#include "system_stm32f4xx.h"
#include "lis302dl.h"

/*************************************************
* function declarations
*************************************************/
void Default_Handler(void);
int main(void);
void spi_write(uint8_t reg, uint8_t data);
uint8_t spi_read(uint8_t reg);

/*************************************************
* Vector Table
*************************************************/
// get the stack pointer location from linker
typedef void (* const intfunc)(void);
extern unsigned long __stack;

// attribute puts table in beginning of .vectors section
//   which is the beginning of .text section in the linker script
// Add other vectors -in order- here
// Vector table can be found on page 372 in RM0090
__attribute__ ((section(".vectors")))
void (* const vector_table[])(void) = {
    (intfunc)((unsigned long)&__stack), /* 0x000 Stack Pointer */
    Reset_Handler,                      /* 0x004 Reset         */
    Default_Handler,                    /* 0x008 NMI           */
    Default_Handler,                    /* 0x00C HardFault     */
    Default_Handler,                    /* 0x010 MemManage     */
    Default_Handler,                    /* 0x014 BusFault      */
    Default_Handler,                    /* 0x018 UsageFault    */
    0,                                  /* 0x01C Reserved      */
    0,                                  /* 0x020 Reserved      */
    0,                                  /* 0x024 Reserved      */
    0,                                  /* 0x028 Reserved      */
    Default_Handler,                    /* 0x02C SVCall        */
    Default_Handler,                    /* 0x030 Debug Monitor */
    0,                                  /* 0x034 Reserved      */
    Default_Handler,                    /* 0x038 PendSV        */
    Default_Handler,                    /* 0x03C SysTick       */
    0,                                  /* 0x040 Window WatchDog Interrupt                                         */
    0,                                  /* 0x044 PVD through EXTI Line detection Interrupt                         */
    0,                                  /* 0x048 Tamper and TimeStamp interrupts through the EXTI line             */
    0,                                  /* 0x04C RTC Wakeup interrupt through the EXTI line                        */
    0,                                  /* 0x050 FLASH global Interrupt                                            */
    0,                                  /* 0x054 RCC global Interrupt                                              */
    0,                                  /* 0x058 EXTI Line0 Interrupt                                              */
    0,                                  /* 0x05C EXTI Line1 Interrupt                                              */
    0,                                  /* 0x060 EXTI Line2 Interrupt                                              */
    0,                                  /* 0x064 EXTI Line3 Interrupt                                              */
    0,                                  /* 0x068 EXTI Line4 Interrupt                                              */
    0,                                  /* 0x06C DMA1 Stream 0 global Interrupt                                    */
    0,                                  /* 0x070 DMA1 Stream 1 global Interrupt                                    */
    0,                                  /* 0x074 DMA1 Stream 2 global Interrupt                                    */
    0,                                  /* 0x078 DMA1 Stream 3 global Interrupt                                    */
    0,                                  /* 0x07C DMA1 Stream 4 global Interrupt                                    */
    0,                                  /* 0x080 DMA1 Stream 5 global Interrupt                                    */
    0,                                  /* 0x084 DMA1 Stream 6 global Interrupt                                    */
    0,                                  /* 0x088 ADC1, ADC2 and ADC3 global Interrupts                             */
    0,                                  /* 0x08C CAN1 TX Interrupt                                                 */
    0,                                  /* 0x090 CAN1 RX0 Interrupt                                                */
    0,                                  /* 0x094 CAN1 RX1 Interrupt                                                */
    0,                                  /* 0x098 CAN1 SCE Interrupt                                                */
    0,                                  /* 0x09C External Line[9:5] Interrupts                                     */
    0,                                  /* 0x0A0 TIM1 Break interrupt and TIM9 global interrupt                    */
    0,                                  /* 0x0A4 TIM1 Update Interrupt and TIM10 global interrupt                  */
    0,                                  /* 0x0A8 TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
    0,                                  /* 0x0AC TIM1 Capture Compare Interrupt                                    */
    0,                                  /* 0x0B0 TIM2 global Interrupt                                             */
    0,                                  /* 0x0B4 TIM3 global Interrupt                                             */
    0,                                  /* 0x0B8 TIM4 global Interrupt                                             */
    0,                                  /* 0x0BC I2C1 Event Interrupt                                              */
    0,                                  /* 0x0C0 I2C1 Error Interrupt                                              */
    0,                                  /* 0x0C4 I2C2 Event Interrupt                                              */
    0,                                  /* 0x0C8 I2C2 Error Interrupt                                              */
    0,                                  /* 0x0CC SPI1 global Interrupt                                             */
    0,                                  /* 0x0D0 SPI2 global Interrupt                                             */
    0,                                  /* 0x0D4 USART1 global Interrupt                                           */
    0,                                  /* 0x0D8 USART2 global Interrupt                                           */
    0,                                  /* 0x0DC USART3 global Interrupt                                           */
    0,                                  /* 0x0E0 External Line[15:10] Interrupts                                   */
    0,                                  /* 0x0E4 RTC Alarm (A and B) through EXTI Line Interrupt                   */
    0,                                  /* 0x0E8 USB OTG FS Wakeup through EXTI line interrupt                     */
    0,                                  /* 0x0EC TIM8 Break Interrupt and TIM12 global interrupt                   */
    0,                                  /* 0x0F0 TIM8 Update Interrupt and TIM13 global interrupt                  */
    0,                                  /* 0x0F4 TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
    0,                                  /* 0x0F8 TIM8 Capture Compare global interrupt                             */
    0,                                  /* 0x0FC DMA1 Stream7 Interrupt                                            */
    0,                                  /* 0x100 FSMC global Interrupt                                             */
    0,                                  /* 0x104 SDIO global Interrupt                                             */
    0,                                  /* 0x108 TIM5 global Interrupt                                             */
    0,                                  /* 0x10C SPI3 global Interrupt                                             */
    0,                                  /* 0x110 UART4 global Interrupt                                            */
    0,                                  /* 0x114 UART5 global Interrupt                                            */
    0,                                  /* 0x118 TIM6 global and DAC1&2 underrun error  interrupts                 */
    0,                                  /* 0x11C TIM7 global interrupt                                             */
    0,                                  /* 0x120 DMA2 Stream 0 global Interrupt                                    */
    0,                                  /* 0x124 DMA2 Stream 1 global Interrupt                                    */
    0,                                  /* 0x128 DMA2 Stream 2 global Interrupt                                    */
    0,                                  /* 0x12C DMA2 Stream 3 global Interrupt                                    */
    0,                                  /* 0x130 DMA2 Stream 4 global Interrupt                                    */
    0,                                  /* 0x134 Ethernet global Interrupt                                         */
    0,                                  /* 0x138 Ethernet Wakeup through EXTI line Interrupt                       */
    0,                                  /* 0x13C CAN2 TX Interrupt                                                 */
    0,                                  /* 0x140 CAN2 RX0 Interrupt                                                */
    0,                                  /* 0x144 CAN2 RX1 Interrupt                                                */
    0,                                  /* 0x148 CAN2 SCE Interrupt                                                */
    0,                                  /* 0x14C USB OTG FS global Interrupt                                       */
    0,                                  /* 0x150 DMA2 Stream 5 global interrupt                                    */
    0,                                  /* 0x154 DMA2 Stream 6 global interrupt                                    */
    0,                                  /* 0x158 DMA2 Stream 7 global interrupt                                    */
    0,                                  /* 0x15C USART6 global interrupt                                           */
    0,                                  /* 0x160 I2C3 event interrupt                                              */
    0,                                  /* 0x164 I2C3 error interrupt                                              */
    0,                                  /* 0x168 USB OTG HS End Point 1 Out global interrupt                       */
    0,                                  /* 0x16C USB OTG HS End Point 1 In global interrupt                        */
    0,                                  /* 0x170 USB OTG HS Wakeup through EXTI interrupt                          */
    0,                                  /* 0x174 USB OTG HS global interrupt                                       */
    0,                                  /* 0x178 DCMI global interrupt                                             */
    0,                                  /* 0x17C RNG global Interrupt                                              */
    0                                   /* 0x180 FPU global interrupt                                              */
};

/*************************************************
* default interrupt handler
*************************************************/
void Default_Handler(void)
{
    for (;;);  // Wait forever
}


/*
 * write spi function customized for lis302dl
 */
void spi_write(uint8_t reg, uint8_t data)
{
    GPIOE->ODR &= (uint16_t)(~(1 << 3)); // enable
    // bit 15 is 0 for write for lis302dl
    uint16_t frame = 0;
    frame |= (uint16_t)(reg << 8);
    frame |= data;
    // Send data
    SPI1->DR = frame;
    // wait until transmit is done (TXE flag)
    while (!(SPI1->SR & (1 << 1)));
    // wait until rx buf is not empty (RXNE flag)
    while (!(SPI1->SR & (1 << 0)));

    GPIOE->ODR |= (1 << 3); // disable
    (void)SPI1->DR; // dummy read
}

/*
 * read spi function customized for lis302dl
 */
uint8_t spi_read(uint8_t reg)
{
    GPIOE->ODR &= (uint16_t)(~(1 << 3)); // enable
    // bit 15 is 1 for read for lis302dl
    uint16_t frame = 0;
    frame |= (1 << 15);
    frame |= (uint16_t)(reg << 8);
    // Send data
    SPI1->DR = frame;
    // wait until tx buf is empty (TXE flag)
    while (!(SPI1->SR & (1 << 1)));
    // wait until rx buf is not empty (RXNE flag)
    while (!(SPI1->SR & (1 << 0)));

    GPIOE->ODR |= (1 << 3); // disable
    return (uint8_t)SPI1->DR;
}

/*************************************************
* main code starts from here
*************************************************/
int main(void)
{
    /* set system clock to 168 Mhz */
    set_sysclk_to_168();

    // enable GPIOD clock, bit 3 on AHB1ENR
    // setup LEDs
    RCC->AHB1ENR |= (1 << 3);
    GPIOD->MODER &= 0x00FFFFFF;
    GPIOD->MODER |= 0x55000000;
    GPIOD->ODR    = 0;

    // SPI1 Chip-Select setup (PE3)

    // enable GPIOE clock, bit 4 on AHB1ENR
    RCC->AHB1ENR |= (1 << 4);
    GPIOE->MODER &= 0xFFFFFF3F; // reset bits 6-7
    GPIOE->MODER |= 0x00000040; // set bits 6-7 to 0b01 (output)
    GPIOE->ODR |= (1 << 3);

    // SPI1 data pins setup

    // enable GPIOA clock, bit 0 on AHB1ENR
    RCC->AHB1ENR |= (1 << 0);
    // enable SPI1 clock, bit 12 on APB2ENR
    RCC->APB2ENR |= (1 << 12);

    // set pin modes as alternate mode
    GPIOA->MODER &= 0xFFFF03FF; // Reset bits 10-15 to clear old values
    GPIOA->MODER |= 0x0000A800; // Set pin 5/6/7 to alternate func. mode (0b10)

    // set pin modes as very high speed
    GPIOA->OSPEEDR |= 0x0000FC00; // Set pin 5/6/7 to very high speed mode (0b11)

    // choose AF5 for SPI1 in Alternate Function registers
    GPIOA->AFR[0] |= (0x5 << 20); // for pin 5
    GPIOA->AFR[0] |= (0x5 << 24); // for pin 6
    GPIOA->AFR[0] |= (0x5 << 28); // for pin 7

    // Disable SPI1 and set the rest (no OR'ing)

    // baud rate - BR[2:0] is bit 5:3
    // fPCLK/2 is selected by default
    SPI1->CR1 = (0x4 << 3); // set baud rate to fPCLK/32

    // 8/16-bit mode - DFF is bit 11
    SPI1->CR1 |= (1 << 11); // 1 - 16-bit mode

    // motion sensor expects clk to be high and
    //  transmission happens on the falling edge
    // clock polarity - CPOL bit 1
    SPI1->CR1 |= (0 << 1); // clk goes 1 when idle
    // clock phase - CPHA bit 0
    SPI1->CR1 |= (0 << 0); // first clock transaction

    // frameformat - LSBFIRST bit 7, msb/lsb transmit first
    // 0 - MSB transmitted first
    //SPI1->CR1 |= (0 << 7); // 1 - LSB transmitted first

    // frame format - FRF bit 4 on CR2
    // 0 - Motorola mode, 1 - TI mode
    //   TI Mode autosets a lot of things
    //   so do not enable it, unless that is what you want
    //SPI1->CR2 |= (1 << 4); // 1 - SPI TI mode

    // software slave management - SSM bit 9
    SPI1->CR1 |= (1 << 9); // 1- ssm enabled
    // internal slave select - SSI bit 8
    SPI1->CR1 |= (1 << 8); // set ssi to 1

    // master config - MSTR bit 2
    SPI1->CR1 |= (1 << 2); // 1 - master mode

    // enable SPI - SPE bit 6
    SPI1->CR1 |= (1 << 6);

    int16_t rbuf[3];

    // reboot memory
    spi_write(LIS302_REG_CTRL_REG2, 0x40);
    // active mode, +/-2g
    spi_write(LIS302_REG_CTRL_REG1, 0x47);
    // wait
    for(int i=0; i<10000000; i++);
    // read who am i
    rbuf[0] = (int8_t)spi_read(LIS302_REG_WHO_AM_I);

    while(1)
    {
        rbuf[0] = (int8_t)spi_read(LIS302_REG_OUT_X);
        rbuf[1] = (int8_t)spi_read(LIS302_REG_OUT_Y);
        rbuf[2] = (int8_t)spi_read(LIS302_REG_OUT_Z);

        if (rbuf[0] > 16) {
            GPIOD->ODR &= (uint16_t)~0x2000;
            GPIOD->ODR |= 0x8000;
        } else if (rbuf[0] < -16 ) {
            GPIOD->ODR &= (uint16_t)~0x8000;
            GPIOD->ODR |= 0x2000;
        } else {
            GPIOD->ODR &= (uint16_t)~0xA000;
        }

        if (rbuf[1] > 16) {
            GPIOD->ODR &= (uint16_t)~0x1000;
            GPIOD->ODR |= 0x4000;
        } else if (rbuf[1] < -16 ) {
            GPIOD->ODR &= (uint16_t)~0x4000;
            GPIOD->ODR |= 0x1000;
        } else {
            GPIOD->ODR &= (uint16_t)~0x5000;
        }

        for(int i=0; i<10000; i++);
    }

    return 0;
}
