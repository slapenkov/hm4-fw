/*
 * syncCycle.h
 *
 *  Created on: 1 окт. 2015 г.
 *      Author: Gray
 */

#ifndef INC_SYNCCYCLE_H_
#define INC_SYNCCYCLE_H_

/*
 * Includes
 * */

#include "stm32f1xx_hal.h"

/*
 * Definitions
 * */
#define DATAPOINTS 2048
#define FILTWIN	64

/*
 * Global variables
 * */
enum measureStates {
	measureIdle,
	measureStart,
	measureAquisition1,
	measureAquisition2,
	measureAquisition3,
	measureParamEstimate
} measureState;

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

//int16_t inputBufferIdx=0;	//input buffer index

extern ADC_HandleTypeDef hadc1;


/*
 * Function prototypes
 * */

/*
 * Private prototypes
 * */
/* Measure private functions */

/* Measure cycle and states processing*/
void Measure(void);
/* Processing indicator leds */
void LedsProcessing(void);

#endif /* INC_SYNCCYCLE_H_ */
