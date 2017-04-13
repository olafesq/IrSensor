/*
 * timer.h
 *
 *  Created on: Mar 18, 2017
 *      Author: Olaf
 */

#ifndef TIMER_H_
#define TIMER_H_

void simple_delay(uint32_t us){ /* timing is not guaranteed :) */
	/* simple delay loop */
	while (us--) {
		asm volatile ("nop");
	}
}

void systickInit (uint16_t frequency) //Initialize system tick interrupt, frequency @ for ex 1000ms
{
   RCC_ClocksTypeDef RCC_Clocks;
   RCC_GetClocksFreq (&RCC_Clocks);
   (void) SysTick_Config (RCC_Clocks.HCLK_Frequency / frequency);
}

uint32_t ticks;
extern "C" void SysTick_Handler(){ ticks++;} //increase ticks on interrupt, has to be in C code

uint32_t millis (){ return ticks;}// return the system clock as milliseconds

uint16_t minutes(){	return millis()/(60*1000);}

void delay_ms (uint32_t t){ //delay in ms, based on systicks
  uint32_t start, end;
  start = millis();
  end = start + t;
  if (start < end) {
    while ((millis() >= start) && (millis() < end)) {
      // do nothing
    }
  } else {
    while ((millis() >= start) || (millis() < end)) {
      // do nothing
    };
  }
}



#endif /* TIMER_H_ */
