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

/* Convert saw voltage into dac level and setup saw dac */
void SetSaw(int16_t sawVoltage);

#endif /* INC_SYNCCYCLE_H_ */
