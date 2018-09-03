#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "stm32f30x.h"

#include "spi.h"


void SPI::init()
{
    SPI_InitTypeDef spiInit;

    // Enable SPI clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI2, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15; // SCK MISO MOSI 
    GPIO_Init(GPIOB, &GPIO_InitStructure);   
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_5);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_5);  
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_5);  
  
    // Init SPI hardware
    SPI_I2S_DeInit(SPI2);

    spiInit.SPI_Mode = SPI_Mode_Master;
    spiInit.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spiInit.SPI_DataSize = SPI_DataSize_8b;
    spiInit.SPI_NSS = SPI_NSS_Soft;
    spiInit.SPI_FirstBit = SPI_FirstBit_MSB;
    spiInit.SPI_CRCPolynomial = 7;
    spiInit.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    spiInit.SPI_CPOL = SPI_CPOL_High;
    spiInit.SPI_CPHA = SPI_CPHA_2Edge;

    SPI_RxFIFOThresholdConfig(SPI2, SPI_RxFIFOThreshold_QF);

    SPI_Init(SPI2, &spiInit);
    SPI_Cmd(SPI2, ENABLE);

}


bool SPI::transferByte(uint8_t* out, uint8_t in)
{
    uint16_t spiTimeout = SPI_TIMEOUT;

    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET)
        if ((spiTimeout--) == 0)
            return false;

    SPI_SendData8(SPI2, in);

    spiTimeout = SPI_TIMEOUT;
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET)
        if ((spiTimeout--) == 0)
            return false;

    if(out != NULL)
    {
        *out = SPI_ReceiveData8(SPI2);
    }
    return true;
}



bool SPI::transfer(uint8_t *out, const uint8_t *in, int len)
{
    uint16_t spiTimeout = SPI_TIMEOUT;

    uint8_t b;
    SPI2->DR;
    while (len--) {
        b = in ? *(in++) : 0xFF;
        while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) {
            if ((spiTimeout--) == 0)
                return false;
        }
        SPI_SendData8(SPI2, b);
        spiTimeout = 1000;
        while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET) {
            if ((spiTimeout--) == 0)
                return false;
        }
        b = SPI_ReceiveData8(SPI2);
        if (out)
            *(out++) = b;
    }

    return true;
}

void SPI::setDivisor(uint16_t divisor)
{
#define BR_CLEAR_MASK 0xFFC7

    uint16_t tempRegister;

    SPI_Cmd(SPI2, DISABLE);

    tempRegister = SPI2->CR1;

    switch (divisor) {
    case 2:
        tempRegister &= BR_CLEAR_MASK;
        tempRegister |= SPI_BaudRatePrescaler_2;
        break;

    case 4:
        tempRegister &= BR_CLEAR_MASK;
        tempRegister |= SPI_BaudRatePrescaler_4;
        break;

    case 8:
        tempRegister &= BR_CLEAR_MASK;
        tempRegister |= SPI_BaudRatePrescaler_8;
        break;

    case 16:
        tempRegister &= BR_CLEAR_MASK;
        tempRegister |= SPI_BaudRatePrescaler_16;
        break;

    case 32:
        tempRegister &= BR_CLEAR_MASK;
        tempRegister |= SPI_BaudRatePrescaler_32;
        break;

    case 64:
        tempRegister &= BR_CLEAR_MASK;
        tempRegister |= SPI_BaudRatePrescaler_64;
        break;

    case 128:
        tempRegister &= BR_CLEAR_MASK;
        tempRegister |= SPI_BaudRatePrescaler_128;
        break;

    case 256:
        tempRegister &= BR_CLEAR_MASK;
        tempRegister |= SPI_BaudRatePrescaler_256;
        break;
    }

    SPI2->CR1 = tempRegister;

    SPI_Cmd(SPI2, ENABLE);
}
