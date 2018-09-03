/**
 *
 * say something about project
 *
 * fifo.c
 *
 * v1.0
 *
 * say something about file
 */
#include <stdbool.h>
#include <stdint.h>

#include "fifo.h"


 void vPortSetBASEPRI(void)
{
    asm volatile
	(
		"cpsie i   		\n"	
	);
}


 void vPortRaiseBASEPRI( void )
{
	asm volatile
	(
		"	cpsid i   											\n"	\
	);
}



void Fifo_Create(Fifo *fifo, uint8_t *buf, uint16_t size)
{
    fifo->head = 0;
    fifo->tail = 0;
    fifo->data = buf;
    fifo->size = size;
}


bool Fifo_Write(Fifo *fifo, uint8_t c)
{
    if(fifo->head + 1 == fifo->tail)
    {
        return false;
    }
vPortRaiseBASEPRI();    
    fifo->data[fifo->head] = c;
    fifo->head++;
    fifo->cnt++;
    if(fifo->head >= fifo->size)
    {
        fifo->head = 0;
    }
vPortSetBASEPRI();
    
    return true;
}


void Fifo_WriteForce(Fifo *fifo, uint8_t c)
{
vPortRaiseBASEPRI();     
    if(fifo->head + 1 == fifo->tail)
    {
        fifo->tail = fifo->head;
    }
    fifo->data[fifo->head] = c;
    fifo->head++;
    fifo->cnt++;
    if(fifo->head >= fifo->size)
    {
        fifo->head = 0;
        if(fifo->tail == 0)
        {
            fifo->tail = fifo->size -1;
        }
    }
vPortSetBASEPRI();    
}


bool Fifo_Read(Fifo *fifo, uint8_t* c)
{
    if(fifo->head == fifo->tail)
    {
        return false;
    }
vPortRaiseBASEPRI(); 
    *c = fifo->data[fifo->tail];
    fifo->tail++;
    fifo->cnt--;
    if(fifo->tail >= fifo->size)
    {
        fifo->tail = 0;
    }
vPortSetBASEPRI();
    return true;
}


bool Fifo_IsEmpty(Fifo *fifo)
{
    return (fifo->head == fifo->tail);
}

uint16_t Fifo_GetCount(Fifo *fifo)
{
	return fifo->cnt;
}

uint16_t Fifo_GetTailIndex(Fifo *fifo)
{
    return fifo->tail;
}


void Fifo_SetTailIndex(Fifo *fifo, uint16_t newTailIndex)
{
    fifo->tail = newTailIndex;
}


uint8_t* Fifo_GetTail(Fifo *fifo)
{
    return fifo->data+fifo->tail;
}


#define IS_TAIL_BEHAND_HEAD (fifo->tail < fifo->head)
#define IS_TAIL_FRONT_HEAD  (fifo->tail > fifo->head)
#define IS_BEYOND_HEAD(x)   (((x)>fifo->head && (x)<fifo->tail && IS_TAIL_FRONT_HEAD) \
                          || ((x)>fifo->head && (x)>fifo->tail && IS_TAIL_BEHAND_HEAD))

void Fifo_SetTail(Fifo *fifo, uint8_t* newTail)
{
    uint16_t newIndex;

    newIndex = newTail - fifo->data;
    if(newIndex > fifo->size)
    {
        newIndex -= (fifo->size-1);
    }

    if(IS_TAIL_BEHAND_HEAD)
    {
        if(newIndex > fifo->head)
        {
            fifo->tail = fifo->head;
        }
        else
        {
            fifo->tail = newIndex;
        }
    }
    else if(IS_TAIL_FRONT_HEAD)
    {
        if(IS_BEYOND_HEAD(newIndex))
        {
            fifo->tail = fifo->head;
        }
        else
        {
            fifo->tail = newIndex;
        }
    }
    else //end to end
    {
        fifo->tail = fifo->head;
    }
}
