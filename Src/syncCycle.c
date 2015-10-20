/*
 * syncCycle.c
 *
 *  Created on: 1 окт. 2015 г.
 *      Author: Gray
 */
//#include "stm32f1xx_hal.h"
#include "syncCycle.h"

/* Variables */
int16_t sawVoltage = 0;
int16_t inputBufferIdx = 0;	//input buffer index
int16_t inputBuffer[DATAPOINTS];
int16_t filterBuffer[FILTWIN];
int16_t filterA[FILTWIN];
int16_t filterB[FILTWIN];
int16_t alphaBuffer[FILTWIN];
int16_t betaBuffer[FILTWIN];
int16_t gammaBufer[FILTWIN];
int16_t temperature1Buffer[FILTWIN];
int16_t temperature2Buffer[FILTWIN];
int16_t temperatureInternalBuffer[FILTWIN];

int16_t sawA0 = 0;
int16_t sawA1 = 0;

int deltaSaw = 0;
int errorSaw = 0;
int deltaErrSaw = 0;

/*
 * Function implementations
 * */
void Measure(void) {
	//TODO implement
	switch (measureState) {
	case measureIdle:
		break;
	case measureStart:
		inputBufferIdx = 0; //reset input buffer index
		deltaSaw = deltaErrSaw = sawA1;
		errorSaw = 0;
		sawVoltage = sawA0;

		//set DAC
		SetSaw(sawVoltage);

		measureState = measureAquisition1;	//next state
		break;
	case measureAquisition1:
		HAL_ADC_Start(&hadc1);	//start conversion
		HAL_ADC_PollForConversion(&hadc1, 1); //wait for conversion complete
		inputBuffer[inputBufferIdx] = HAL_ADC_GetValue(&hadc1);	//read current ADC result

		//calculate next DAC
		//TODO use Bresenham's line algorithm and check it
		errorSaw += deltaErrSaw;
		if ((errorSaw << 1) >= DATAPOINTS) {
			sawVoltage += 1;
			errorSaw += DATAPOINTS;
		}
		//set DAC
		SetSaw(sawVoltage);
		//check is cycle over
		if (inputBufferIdx > DATAPOINTS) {
			measureState = measureParamEstimate;
		} else
			inputBufferIdx++;
		break;
	case measureParamEstimate:
		break;
	};
}

void LedsProcessing(void) {

}

void SetSaw(int16_t sawVoltage) {

}
