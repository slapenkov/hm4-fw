/*
 * syncCycle.c
 *
 *  Created on: 1 окт. 2015 г.
 *      Author: Gray
 */

#include "syncCycle.h"

/*
 * Function implementations
 * */
void Measure(void) {
	//TODO implement
	switch (measureState) {
	case measureIdle:
		break;
	case measureStart:
		//inputBufferIdx = 0; //reset input buffer index

		//set DAC

		measureState = measureAquisition1;	//next state
		break;
	case measureAquisition1:
		HAL_ADC_Start(&hadc1);	//start conversion
		HAL_ADC_PollForConversion(&hadc1,1); //wait for conversion complete
		//inputBuffer[inputBufferIdx] = HAL_ADC_GetValue(&hadc1);	//read current ADC result
		//calculate next DAC
		//TODO use Bresenham's line algorithm
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

void LedsProcessing(void) {

}
