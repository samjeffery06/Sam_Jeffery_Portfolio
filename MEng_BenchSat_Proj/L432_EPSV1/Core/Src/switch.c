/*
 *  EPS Switches Driver
 *
 *  Author: 	Sam Jeffery
 *  Created: 	21 March 2022
 *  Version: 	1.0
 */

#include "switch.h"

/*
 * INITIALISATION
 */

uint8_t SWITCH_Initialise( switches *dev )
{
	// Initialisation specific to application, driver will need to be updated
	dev->PORT = GPIOA;

	dev->pins = 0xFA;

	dev->set = 0;

	dev->status = 0;

	return 0;
}

/*
 * High Level Functions
 */

void SWITCH_SetCH1( switches *dev )
{
	dev->set = 0x9;

	SWITCH_setPins( dev );

	dev->status = dev->set;

}

void SWITCH_Clear( switches *dev )
{
	dev->set = 0x0;

	SWITCH_setPins( dev );

	dev->status = dev->set;

}

void SWITCH_SetALL( switches *dev, uint8_t input )
{
	dev->set = input;

	SWITCH_setPins( dev );

	dev->status = dev->set;
}

/*
 * LOW-LEVEL FUNCTIONS
 */

void SWITCH_setPins( switches *dev )
{
	/*
	 * Requires Set
	 * Specific to the pin configuration on EPS V2.0 Board
	 */

	dev->PORT->ODR = (dev->PORT->ODR & ~dev->pins) | ((dev->set & 0x3E)<<2) | (dev->set & 0x1)<<1;

}

void SWITCH_SetSpecific( switches *dev, uint8_t mask, uint8_t onoff )
{
	mask &= 0x3F;

	onoff &= 0x3F;

	dev->set = onoff & mask;

	dev->mask = mask;

	SWITCH_updatePins( dev );

	dev->status = (dev->status & ~mask) | dev->set;
}

/*
 * LOW-LEVEL FUNCTIONS
 */

void SWITCH_updatePins( switches *dev )
{
	/*
	 * Requires Set
	 * Specific to the pin configuration on EPS V2.0 Board
	 */

	// Set PHASE
	uint32_t temp = dev->PORT->ODR;

	temp &= ~(((dev->mask & 0x3E)<<2) | ((dev->mask & 0x1)<<1));

	temp |= (((dev->set & 0x3E)<<2) | (dev->set & 0x1)<<1);

	dev->PORT->ODR = temp;

}

/*
 * UNIT TESTS
 */

void SWITCH_ON_OFF( switches *dev )
{
	SWITCH_SetCH1( dev );

	HAL_Delay(5000);

	SWITCH_Clear( dev );
}

