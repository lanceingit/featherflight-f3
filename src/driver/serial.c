/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011-2014  Bill Nesbitt
*/

#include <string.h>
#include "stm32f30x.h"
#include "serial.h"


static serialPort_t serialPort1;


static void serialUart1Init(void);


static void serialIRQHandler(serialPort_t *s);




static void serialUart1Init(void) 
{
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStruct;
	
    // Enable USART1 clock
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	
    GPIO_InitStructure.GPIO_Pin = (GPIO_Pin_9 | GPIO_Pin_10);   
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;            
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;        
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_7);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_7);

    NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    
    NVIC_Init(&NVIC_InitStruct);
}




serialPort_t * serialOpen(USART_TypeDef *USARTx, uint32_t baud, uint8_t* rxBuf, uint16_t rxBufSize, uint8_t* txBuf, uint16_t txBufSize) 
{
    serialPort_t *s = NULL;
    USART_InitTypeDef USART_InitStructure;
    
    if (USARTx == USART1) {
		serialUart1Init();
        s = &serialPort1;
    } 
    
    s->USARTx = USARTx;
    s->baudRate = baud;
    s->rxBufSize = rxBufSize;
    s->rxBuf = rxBuf;
    s->txBufSize = txBufSize;
    s->txBuf = txBuf;
    
    Fifo_Create(&s->rxFifo, rxBuf, rxBufSize);
    Fifo_Create(&s->txFifo, txBuf, txBufSize);


    // reduce oversampling to allow for higher baud rates
    USART_OverSampling8Cmd(s->USARTx, ENABLE);
	
    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = s->baudRate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(s->USARTx, &USART_InitStructure);

    USART_Cmd(s->USARTx, ENABLE);    
		
    USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USARTx, USART_IT_TXE, ENABLE);
    
    return s;
}

void serialWrite(serialPort_t *s, unsigned char ch) 
{
    Fifo_WriteForce(&s->txFifo, ch);

    USART_ITConfig(s->USARTx, USART_IT_TXE, ENABLE);
}

void serialWriteMass(serialPort_t *s, unsigned char* buf, uint16_t len) 
{
    uint16_t i;
    for(i=0; i<len; i++)
    {
        Fifo_WriteForce(&s->txFifo, buf[i]);
    }

    USART_ITConfig(s->USARTx, USART_IT_TXE, ENABLE);
}

bool serialAvailable(serialPort_t *s) 
{
	return !Fifo_IsEmpty(&s->rxFifo);
}

uint8_t serialRead(serialPort_t *s) {
    uint8_t ch;
    
    Fifo_Read(&s->rxFifo, &ch);
    
    return ch;
}


//
// Interrupt handlers
//
static void serialIRQHandler(serialPort_t *s) 
{
    uint16_t SR = s->USARTx->ISR;
	
    if (SR & USART_FLAG_RXNE) {
        Fifo_WriteForce(&s->rxFifo, s->USARTx->RDR);
    }

    if (SR & USART_FLAG_TXE) {
        if (!Fifo_IsEmpty(&s->txFifo)) {
            uint8_t ch;
            Fifo_Read(&s->txFifo, &ch);
            s->USARTx->TDR = ch;      
	    }
        // EOT
        else {
            USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
        }
    }
}


void USART1_IRQHandler(void) 
{
    serialIRQHandler(&serialPort1);
}

