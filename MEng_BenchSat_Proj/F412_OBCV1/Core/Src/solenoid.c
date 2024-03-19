/*
 *  Solenoid GPIO Driver
 *
 *  Author: 	Sam Jeffery
 *  Created: 	21 March 2022
 *  Version 	1.0
 */

#include "solenoid.h"

/*
 * GROUND STATION INTERACTION FUNCTIONS
 */

uint8_t Solenoid_Move_Start( solenoid *dev, uint8_t move_select, uint8_t time_select )
{
	if (dev->move_selected == 0) //Check solenoid is not currently moving
	{
		if (move_select == 0 || time_select == 0)
		{
			return 0;
		}
		dev->move_selected = move_select;
		dev->end_time = HAL_GetTick() + 200*time_select;
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		return Solenoid_Set_Pins(dev->move_selected);
	}
	return 1;
}


void Process_Solenoid( solenoid *dev )
{
	if (dev->move_selected != 0)
	{
		if (HAL_GetTick() > dev->end_time)
		{
			Solenoid_Reset_Pins();
			dev->move_selected = 0;
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		}
	}
}


/*
 * LOW-LEVEL FUNCTIONS
 */

uint8_t Solenoid_Set_Pins(uint8_t option)
{
	switch (option)
	{
	case 1:
		HAL_GPIO_WritePin(SOL_N1_GPIO_Port, SOL_N1_Pin, 1);
		HAL_GPIO_WritePin(SOL_N2_GPIO_Port, SOL_N2_Pin, 1);
		break;
	case 2:
		HAL_GPIO_WritePin(SOL_N3_GPIO_Port, SOL_N3_Pin, 1);
		HAL_GPIO_WritePin(SOL_N4_GPIO_Port, SOL_N4_Pin, 1);
		break;
	case 3:
		HAL_GPIO_WritePin(SOL_N1_GPIO_Port, SOL_N1_Pin, 1);
		HAL_GPIO_WritePin(SOL_N3_GPIO_Port, SOL_N3_Pin, 1);
		break;
	case 4:
		HAL_GPIO_WritePin(SOL_N2_GPIO_Port, SOL_N2_Pin, 1);
		HAL_GPIO_WritePin(SOL_N4_GPIO_Port, SOL_N4_Pin, 1);
		break;
	default:
		return 1;
	}
	return 0;
}

void Solenoid_Reset_Pins()
{
	HAL_GPIO_WritePin(SOL_N1_GPIO_Port, SOL_N1_Pin, 0);
	HAL_GPIO_WritePin(SOL_N2_GPIO_Port, SOL_N2_Pin, 0);
	HAL_GPIO_WritePin(SOL_N3_GPIO_Port, SOL_N3_Pin, 0);
	HAL_GPIO_WritePin(SOL_N4_GPIO_Port, SOL_N4_Pin, 0);
}

/*
 * UNIT TESTS
 */

void Solenoid_Translate_Forward(uint8_t time)
{
	Solenoid_Set_Pins(1);
	for (uint8_t i = 0; i < time*4; i++)
	{
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		HAL_Delay(250);
	}
	Solenoid_Reset_Pins();
}

void Solenoid_Translate_Backward(uint8_t time)
{
	Solenoid_Set_Pins(2);
	for (uint8_t i = 0; i < time*4; i++)
	{
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		HAL_Delay(250);
	}
	Solenoid_Reset_Pins();
}

void Solenoid_Rot_CLOCK(uint8_t time)
{
	Solenoid_Set_Pins(3);
	for (uint8_t i = 0; i < time*4; i++)
	{
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		HAL_Delay(250);
	}
	Solenoid_Reset_Pins();
}

void Solenoid_Rot_ANTI(uint8_t time)
{
	Solenoid_Set_Pins(4);
	for (uint8_t i = 0; i < time*4; i++)
	{
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		HAL_Delay(250);
	}
	Solenoid_Reset_Pins();
}
