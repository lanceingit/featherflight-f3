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

#pragma once

#ifdef __cplusplus
 extern "C" {
#endif 

#include "stm32f30x.h"
#include "fifo.h"
#include <stdbool.h>


typedef struct {
    uint32_t baudRate;

    uint16_t txBufSize;
    uint8_t *txBuf;
    Fifo txFifo;

    uint16_t rxBufSize;
    uint8_t *rxBuf;
    Fifo rxFifo;

    USART_TypeDef *USARTx;
} serialPort_t;

serialPort_t * serialOpen(USART_TypeDef *USARTx, uint32_t baud, uint8_t* rxBuf, uint16_t rxBufSize, uint8_t* txBuf, uint16_t txBufSize);
void serialWrite(serialPort_t *s, unsigned char ch);
void serialWriteMass(serialPort_t *s, unsigned char* buf, uint16_t len);
bool serialAvailable(serialPort_t *s);
uint8_t serialRead(serialPort_t *s);


#ifdef __cplusplus
}
#endif



