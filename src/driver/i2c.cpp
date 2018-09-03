#include <stdbool.h>
#include <stdint.h>

#include "stm32f30x.h"


#include "i2c.h"


void I2C::init()
{
    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
    RCC_I2CCLKConfig(RCC_I2C1CLK_SYSCLK);
   
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
    
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; // SCL
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7; // SDA
    GPIO_Init(GPIOB, &GPIO_InitStructure);    
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_4);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_4);    

    I2C_InitTypeDef i2cInit;
    i2cInit.I2C_Mode = I2C_Mode_I2C,
    i2cInit.I2C_AnalogFilter = I2C_AnalogFilter_Enable,
    i2cInit.I2C_DigitalFilter = 0x00,
    i2cInit.I2C_OwnAddress1 = 0x00,
    i2cInit.I2C_Ack = I2C_Ack_Enable,
    i2cInit.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit,
    i2cInit.I2C_Timing = (0x00E0257A);

    I2C_Init(I2C1, &i2cInit);

    I2C_StretchClockCmd(I2C1, ENABLE);
 
    I2C_Cmd(I2C1, ENABLE);
}

bool I2C::write(uint8_t addr_, uint8_t reg, uint8_t data)
{
    addr_ <<= 1;


    /* Test on BUSY Flag */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2C1, I2C_ISR_BUSY) != RESET) {
        if ((i2cTimeout--) == 0) {
            return false;
        }
    }

    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2C1, addr_, 1, I2C_Reload_Mode, I2C_Generate_Start_Write);

    /* Wait until TXIS flag is set */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET) {
        if ((i2cTimeout--) == 0) {
            return false;
        }
    }

    /* Send Register address */
    I2C_SendData(I2C1, (uint8_t) reg);

    /* Wait until TCR flag is set */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2C1, I2C_ISR_TCR) == RESET)
    {
        if ((i2cTimeout--) == 0) {
            return false;
        }
    }

    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2C1, addr_, 1, I2C_AutoEnd_Mode, I2C_No_StartStop);

    /* Wait until TXIS flag is set */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET) {
        if ((i2cTimeout--) == 0) {
            return false;
        }
    }

    /* Write data to TXDR */
    I2C_SendData(I2C1, data);

    /* Wait until STOPF flag is set */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2C1, I2C_ISR_STOPF) == RESET) {
        if ((i2cTimeout--) == 0) {
            return false;
        }
    }

    /* Clear STOPF flag */
    I2C_ClearFlag(I2C1, I2C_ICR_STOPCF);

    return true;
}

bool I2C::read(uint8_t addr_, uint8_t reg, uint8_t len, uint8_t* buf)
{
    addr_ <<= 1;

    /* Test on BUSY Flag */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2C1, I2C_ISR_BUSY) != RESET) {
        if ((i2cTimeout--) == 0) {
            return false;
        }
    }

    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2C1, addr_, 1, I2C_SoftEnd_Mode, I2C_Generate_Start_Write);

    /* Wait until TXIS flag is set */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2C1, I2C_ISR_TXIS) == RESET) {
        if ((i2cTimeout--) == 0) {
            return false;
        }
    }

    /* Send Register address */
    I2C_SendData(I2C1, (uint8_t) reg);

    /* Wait until TC flag is set */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2C1, I2C_ISR_TC) == RESET) {
        if ((i2cTimeout--) == 0) {
            return false;
        }
    }

    /* Configure slave address, nbytes, reload, end mode and start or stop generation */
    I2C_TransferHandling(I2C1, addr_, len, I2C_AutoEnd_Mode, I2C_Generate_Start_Read);

    /* Wait until all data are received */
    while (len) {
        /* Wait until RXNE flag is set */
        i2cTimeout = I2C_LONG_TIMEOUT;
        while (I2C_GetFlagStatus(I2C1, I2C_ISR_RXNE) == RESET) {
            if ((i2cTimeout--) == 0) {
                return false;
            }
        }

        /* Read data from RXDR */
        *buf = I2C_ReceiveData(I2C1);
        /* Point to the next location where the byte read will be saved */
        buf++;

        /* Decrement the read bytes counter */
        len--;
    }

    /* Wait until STOPF flag is set */
    i2cTimeout = I2C_LONG_TIMEOUT;
    while (I2C_GetFlagStatus(I2C1, I2C_ISR_STOPF) == RESET) {
        if ((i2cTimeout--) == 0) {
            return false;
        }
    }

    /* Clear STOPF flag */
    I2C_ClearFlag(I2C1, I2C_ICR_STOPCF);

    /* If all operations OK */
    return true;
}










