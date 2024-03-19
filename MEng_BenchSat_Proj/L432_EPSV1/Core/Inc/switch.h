/*
 *  EPS Switches Driver
 *
 *  Author: 	Sam Jeffery
 *  Created: 	21 March 2022
 *  Version: 	1.0
 */

#ifndef INC_SWITCH_H_
#define INC_SWITCH_H_

#include "main.h"

/*
 * DEFINES
 */


/*
 * Driver STRUCT
 */

typedef struct{

	GPIO_TypeDef *PORT;

	uint8_t pins;

	uint8_t status; 	// 00bbbbbb = 00 (3V3S3) (3V3S2) (3V3S1) (5VS3) (5VS2) (5VS1)

	uint8_t set; 		// 00bbbbbb = 00 (3V3S3) (3V3S2) (3V3S1) (5VS3) (5VS2) (5VS1)

	uint8_t mask;		// 00bbbbbb = 00 (3V3S3) (3V3S2) (3V3S1) (5VS3) (5VS2) (5VS1)

} switches;


/*
 * INITIALISATION
 */

uint8_t SWITCH_Initialise();

/*
 * High Level Functions
 */

void SWITCH_SetCH1( switches *dev );

void SWITCH_Clear( switches *dev );

void SWITCH_SetALL( switches *dev, uint8_t input );

void SWITCH_SetSpecific( switches *dev, uint8_t mask, uint8_t onoff );

/*
 * LOW-LEVEL FUNCTIONS
 */

void SWITCH_setPins( switches *dev );

void SWITCH_updatePins( switches *dev );

/*
 * UNIT TESTS
 */

void SWITCH_ON_OFF( switches *dev );



#endif /* INC_SWITCH_H_ */
