/*
 *  Solenoid GPIO Driver
 *
 *  Author: 	Sam Jeffery
 *  Created: 	21 March 2022
 *  Version 	1.0
 */


#ifndef INC_SOLENOID_H_
#define INC_SOLENOID_H_

#include "main.h"

/*
 * DEFINES
 */

/*
 * Driver STRUCT
 */


typedef struct{

	uint32_t end_time;

	uint8_t move_selected;
	/*
	 * 0: Not currently moving
	 * 1: Forward
	 * 2: Backward
	 * 3: Clockwise
	 * 4: Anti_Clockwise
	 */

} solenoid;

/*
 * MAIN FUNCTION
 */

uint8_t Solenoid_Move_Start( solenoid *dev, uint8_t move_select, uint8_t time_select );

void Process_Solenoid( solenoid *dev );

/*
 * LOW-LEVEL FUNCTIONS
 */

uint8_t Solenoid_Set_Pins(uint8_t option);

void Solenoid_Reset_Pins();


/*
 * UNIT TESTS
 */

void Solenoid_Translate_Forward(uint8_t time);

void Solenoid_Translate_Backward(uint8_t time);

void Solenoid_Rot_CLOCK(uint8_t time);

void Solenoid_Rot_ANTI(uint8_t time);

#endif /* INC_SOLENOID_H_ */
