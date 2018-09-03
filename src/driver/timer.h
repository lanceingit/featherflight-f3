/**
 *
 * timer.h
 *
 * simple timer, delay and time block function 
 */


#pragma once

#ifdef __cplusplus
 extern "C" {
#endif

void Timer_init(void);
void Timer_disable(void);

uint64_t Timer_create(uint64_t us);
bool Timer_isTimeout(uint64_t t);
void Timer_delayUs(uint64_t us);
uint64_t Timer_getTime(void);
uint64_t Timer_elapsedTime(uint64_t* t);

#ifdef __cplusplus
}
#endif



     

