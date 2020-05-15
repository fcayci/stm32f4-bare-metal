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
 *    These steps are for SPI master mode
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
int main(void);
void spi_write(uint8_t reg, uint8_t data);
uint8_t spi_read(uint8_t reg);

/*
 * write spi function customized for lis302dl
 */
void spi_write(uint8_t reg, uint8_t data)
{
    GPIOE->ODR &= ~(1U << 3); // enable
    // bit 15 is 0 for write for lis302dl
    uint32_t frame = 0;
    frame = data;
    frame |= (uint16_t)(reg << 8);
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
    GPIOE->ODR &= ~(1U << 3); // enable
    // bit 15 is 1 for read for lis302dl
    uint16_t frame = 0;
    frame |= (uint16_t)(reg << 8);
    frame |= (1 << 15); // read bit
    // Send data
    SPI1->DR = frame;
    // wait until tx buf is empty (TXE flag)
    while (!(SPI1->SR & (1 << 1)));
    // wait until rx buf is not empty (RXNE flag)
    while (!(SPI1->SR & (1 << 0)));

    uint8_t b = (uint8_t)SPI1->DR;
    GPIOE->ODR |= (1 << 3); // disable
    return b;
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
