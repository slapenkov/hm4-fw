/*
 * syncCycle.c
 *
 *  Created on: 1 ���. 2015 �.
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
