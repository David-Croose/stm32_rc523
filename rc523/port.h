#ifndef __PORT_H_
#define __PORT_H_

#include "stm32f7xx_hal.h"

static void rc523_select(char en)
{
    if(en > 0)
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    }
}

static void rc523_reset(char en)
{
    if(en > 0)
    {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
    }
}

static uint8_t rc523_readwrite_byte(uint8_t byte)
{
    extern SPI_HandleTypeDef hspi3;
    uint8_t rbyte, wbyte = byte;

    if(HAL_SPI_TransmitReceive(&hspi3, &wbyte, &rbyte, 1, 10) != HAL_OK)
    {
        rbyte = 0xFF;
    }

    return rbyte;
}

//////////////////////////////////////////////////////////////////////////////

#define SetCsLow()      rc523_select(1)
#define SetCsHigh()     rc523_select(0)

#define SpiReadWriteByte(x)     rc523_readwrite_byte((x))

#define GPIO_WriteBit(x)        rc523_reset((x))

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned int u32;
typedef char s8;
typedef short s16;
typedef int s32;


#endif
