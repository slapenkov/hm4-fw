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


/*
 * Global variables
 * */
enum measureStates{
	measureIdle,
	measureStart,
	measureAquisition1,
	measureAquisition2,
	measureAquisition3,
	measureParamEstimate
} measureState;

/*
 * Function prototypes
 * */


/*
 * Private prototypes
 * */
/* Measure private functions */

/* Measure cycle and states processing*/
void Measure(void);


#endif /* INC_SYNCCYCLE_H_ */
