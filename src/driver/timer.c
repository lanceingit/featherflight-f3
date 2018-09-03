/**
 *
 * timer.c
 *
 * simple timer, delay and time block function 
 */

#include <stdbool.h>

#include "stm32f30x.h"

#include "timer.h"


static volatile uint64_t _timerCnt = 0;


void Timer_init()
{
    TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    TIM_DeInit(TIM7);
        
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
    TIM_TimeBaseStructure.TIM_Period = 10-1;                //10us 
    TIM_TimeBaseStructure.TIM_Prescaler = 72;       
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;    
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);    
    
    TIM_ARRPreloadConfig(TIM7, DISABLE);
    
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
    
    TIM_Cmd(TIM7, ENABLE);
}


void Timer_disable(void)
{
    NVIC_DisableIRQ(TIM7_IRQn);    
}


static void Timer_irs(void)
{    
    TIM_ClearFlag(TIM7, TIM_IT_Update);
    _timerCnt++;
}

uint64_t Timer_create(uint64_t us)
{
    return (_timerCnt + (us/10) - 1);
}


bool Timer_isTimeout(uint64_t t)
{
    if(t >= _timerCnt)
    {
        return false;
    }
    else
    {
        return true;
    }
}


void Timer_delayUs(uint64_t us)
{
    volatile uint64_t wait;

    wait = Timer_create(us);
    while (!Timer_isTimeout(wait));
}

uint64_t Timer_getTime()
{
	return _timerCnt*10;
}

uint64_t Timer_elapsedTime(uint64_t* t)
{
	return _timerCnt*10 - *t;
}

    
void TIM7_IRQHandler(void)
{
    Timer_irs();
}


