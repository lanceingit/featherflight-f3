/**
 *
 * say something about project
 *
 * fifo.h
 *
 * v1.0
 *
 * say something about file
 */


#ifndef __FIFO_H_
#define __FIFO_H_

#ifdef __cplusplus
 extern "C" {
#endif 
     
     
#include <stdbool.h>

typedef struct
{
    uint16_t head;
    uint16_t tail;
    uint8_t *data;
    uint16_t size;
    uint16_t cnt;
}
Fifo;


void Fifo_Create(Fifo *fifo, uint8_t *buf, uint16_t size);

bool Fifo_Write(Fifo *fifo, uint8_t c);
void Fifo_WriteForce(Fifo *fifo, uint8_t c);
bool Fifo_Read(Fifo *fifo, uint8_t* c);

bool Fifo_IsEmpty(Fifo *fifo);
uint16_t Fifo_GetCount(Fifo *fifo);

uint8_t* Fifo_GetTail(Fifo *fifo);
void Fifo_SetTail(Fifo *fifo, uint8_t* newTail);

uint16_t Fifo_GetTailIndex(Fifo *fifo);
void Fifo_SetTailIndex(Fifo *fifo, uint16_t newTailIndex);

#ifdef __cplusplus
}
#endif

#endif /* __FIFO_H_ */
