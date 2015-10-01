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
void HAL_SYSTICK_Callback(void) {
	/* Calls every ticks */
	Measure();
	LedsProcessing();
}

/*
 * Function implementations
 * */
void Measure(void) {
	//TODO implement
	switch (measureState) {
	case measureIdle:
		break;
	case measureStart:
		break;
	case measureAquisition1:
		//read ADC result
		//filtering
		//calculate next DAC
		//set DAC
		//start ADC
		break;
	case measureAquisition2:
		break;
	case measureAquisition3:
		break;
	case measureParamEstimate:
		break;
	};
}


void LedsProcessing(void){

}
