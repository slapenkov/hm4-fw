/*
 * syncCycle.c
 *
 *  Created on: 1 окт. 2015 г.
 *      Author: Gray
 */

#include "syncCycle.h"

/*
 * SYSTICK callback processing
 * */
void HAL_SYSTICK_Callback(void){
	/* Calls every ticks */
	Measure();
}

/*
 * Function implementations
 * */
void Measure(void){
	//TODO implement
}
