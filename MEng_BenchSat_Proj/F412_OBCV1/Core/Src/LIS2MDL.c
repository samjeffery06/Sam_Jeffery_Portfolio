/*
 *  LIS2MDL Compass/Magnetometer I2C Driver
 *
 *  Author: Sam Jeffery
 *  Created: 9 March 2022
 */

#include "LIS2MDL.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

/*
 * INITIALISATION
 */

uint8_t LIS2MDL_Initialise( LIS2MDL *dev, I2C_HandleTypeDef *i2cHandle, uint8_t calibrate, uint8_t* button_flag )
{
	/* Set STRUCT parameters */
	dev->i2cHandle 		= i2cHandle;

	dev->compass[0]		= 0;
	dev->compass[1]		= 0;
	dev->compass[2]		= 0;

//	dev->offsets[0] 	= -1019;
//	dev->offsets[1] 	= 288;
//	dev->offsets[2] 	= 0;
//
	dev->offsets[0] 	= 0;
	dev->offsets[1] 	= 0;
	dev->offsets[2] 	= 0;

	dev->heading		= 0;

	uint8_t errorNum = 0;
	HAL_StatusTypeDef status;

	/*
	 * Check WHO_AM_I
	 */
	uint8_t regData;

	status = LIS2MDL_ReadRegister( dev, LIS2MDL_WHO_AM_I_ADDR, &regData);

	errorNum += ( status != HAL_OK );

	if ( regData != LIS2MDL_WHO_AM_I)
	{
		return 255;
	}

	/*
	 * SET DEFAULT HARD_IRON OFFSETS
	 */



	/*
	 * SOFT REBOOT
	 */
//	regData = 0xA1;
//	status = LIS2MDL_WriteRegister( dev, LIS2MDL_CFG_REG_A, &regData );
//	errorNum += ( status != HAL_OK );
//	HAL_Delay(5);
//	regData = 0xC1;
//	status = LIS2MDL_WriteRegister( dev, LIS2MDL_CFG_REG_A, &regData );
//	HAL_Delay(20);
//	status = LIS2MDL_ReadRegister(dev, LIS2MDL_CFG_REG_A, &regData);
//	errorNum += ( status != HAL_OK );

	/*
	 * Initialization of LIS2MDL
	 */

	regData = 0x82; // MD to continous mode

	status = LIS2MDL_WriteRegister(dev, LIS2MDL_CFG_REG_A, &regData); // Write data to address

	errorNum += ( status != HAL_OK );

//	regData = 0x00; // Set DRDY_on_PIN
//
//	status = LIS2MDL_WriteRegister(dev, LIS2MDL_CFG_REG_B, &regData); // Write data to address
//
//	errorNum += ( status != HAL_OK );

	regData = 0x01; // Set DRDY_on_PIN

	status = LIS2MDL_WriteRegister(dev, LIS2MDL_CFG_REG_C, &regData); // Write data to address

	errorNum += ( status != HAL_OK );

	if (calibrate == 1)
		LIS2MDL_SETUP( dev, button_flag );
	else if (calibrate == 2)
	{
		LIS2MDL_SetOffsets( dev );

		while (!*button_flag)
		{
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			HAL_Delay(50);
		}
		*button_flag = 0;

		LIS2MDL_SetRefHeading( dev );
	}
	else
	{
		LIS2MDL_SetOffsets( dev );
		LIS2MDL_SetRefHeading( dev );
	}

	return errorNum;

}

/*
 * DATA AQUISITION
 */

HAL_StatusTypeDef LIS2MDL_ReadTemperature( LIS2MDL *dev )
{
	uint8_t regData[2];

	HAL_StatusTypeDef status = LIS2MDL_ReadRegisters(dev, LIS2MDL_TEMP_OUT_L_REG, regData, 2);

	int16_t tempRaw = ((regData[1] << 8) | regData[0]);

	dev->temp_C = tempRaw;

	return status;
}

HAL_StatusTypeDef LIS2MDL_ReadCompass( LIS2MDL *dev )
{
	HAL_StatusTypeDef status;

	status = LIS2MDL_GetData( dev );

	float fheading = atan2(dev->compass[1],dev->compass[0])*(180/M_PI);

	if (fheading < 0)
		dev->heading = (uint16_t) (fheading + 360);
	else
		dev->heading = (uint16_t) fheading;

	return status;
}

/*
 * HIGH-LEVEL-FUNCTIONS
 */

HAL_StatusTypeDef LIS2MDL_GetData( LIS2MDL *dev )
{
	uint8_t regData[6];

	HAL_StatusTypeDef status;

	regData[0] = 0x81;

	LIS2MDL_WriteRegister(dev, LIS2MDL_CFG_REG_A, regData);

//	uint8_t data_ready_flag = 0;

	do
	{
		status = LIS2MDL_ReadRegister(dev, LIS2MDL_STATUS_REG, regData);
	}
	while (!((regData[0]>>3) & 1));

	status = LIS2MDL_ReadRegisters(dev, LIS2MDL_OUTX_L_REG, regData, 6);

	dev->compass[0] = ((regData[1] << 8) | regData[0]);

	dev->compass[1] = ((regData[3] << 8) | regData[2]);

	dev->compass[2] = ((regData[5] << 8) | regData[4]);

	return status;
}

HAL_StatusTypeDef LIS2MDL_SetOffsets( LIS2MDL *dev )
{
	uint8_t offsets[6];

	for (uint8_t i = 0; i < 2; i++)
	{
		offsets[i*2] = dev->offsets[i] & 0x00FF;
		offsets[(i*2)+1] = (dev->offsets[i]>>8) & 0x00FF;
	}
	return LIS2MDL_WriteRegisters(dev, LIS2MDL_OFFSET_X_REG_L, offsets, 6); // Write data to address
}

HAL_StatusTypeDef LIS2MDL_CalibrateCompass( LIS2MDL *dev )
{
	for (uint8_t i = 0; i < 3; i++)
	{
		dev->offsets[i] = 0;
	}

	LIS2MDL_SetOffsets( dev );

	LIS2MDL_GetData( dev );

	LIS2MDL_GetData( dev );

	for( uint8_t i = 0; i < 3; i++ )
	{
		dev->compass_max[i] = dev->compass[i];
		dev->compass_min[i] = dev->compass[i];
	}

	uint32_t startTime = HAL_GetTick();

	while (HAL_GetTick() - startTime < 20000){

		LIS2MDL_GetData( dev );

		for( uint8_t i = 0; i < 3; i++ )
		{
			// UPDATE MAX VALUES
			if ( dev->compass[i] > dev->compass_max[i] )
			{
				dev->compass_max[i] = dev->compass[i];
			}
			// UPDATE MIN VALUES
			if ( dev->compass[i] < dev->compass_min[i] )
			{
				dev->compass_min[i] = dev->compass[i];
			}
		}
		HAL_Delay(10);
	}

	for (uint8_t i = 0; i < 3; i++)
	{
		dev->offsets[i] = ( dev->compass_max[i] + dev->compass_min[i] ) / 2;
	}

	LIS2MDL_SetOffsets( dev );

	return HAL_OK;
}

void LIS2MDL_SetRefHeading( LIS2MDL *dev )
{
	LIS2MDL_ReadCompass( dev );
	dev->ref_heading = dev->heading;
}

void LIS2MDL_SETUP( LIS2MDL *dev, uint8_t *button_flag )
{
	while (!*button_flag)
	{
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		HAL_Delay(50);
	}
	*button_flag = 0;
	LIS2MDL_CalibrateCompass( dev );

	while (!*button_flag)
	{
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		HAL_Delay(25);
	}
	*button_flag = 0;
	LIS2MDL_SetRefHeading( dev );
}

/*
 * UNIT TESTS
 */

void LIS2MDL_PrintHeading( LIS2MDL *dev, UART_HandleTypeDef *uart )
{
	char msg[20];

	LIS2MDL_ReadTemperature( dev );

	uint8_t uart_buf_len = sprintf( msg, "TempRaw: %d\r\n", dev->temp_C );
	HAL_UART_Transmit( uart, (uint8_t*) msg, uart_buf_len, HAL_MAX_DELAY );

	LIS2MDL_ReadCompass( dev );

	uart_buf_len = sprintf( msg, "Heading: %d\r\n", dev->heading );
	HAL_UART_Transmit( uart, (uint8_t*) msg, uart_buf_len, HAL_MAX_DELAY );

	uart_buf_len = sprintf( msg, "COMP_X: %d\r\n", dev->compass[0] );
	HAL_UART_Transmit( uart, (uint8_t*) msg, uart_buf_len, HAL_MAX_DELAY );

	uart_buf_len = sprintf( msg, "COMP_Y: %d\r\n", dev->compass[1] );
	HAL_UART_Transmit( uart, (uint8_t*) msg, uart_buf_len, HAL_MAX_DELAY );

	uart_buf_len = sprintf( msg, "COMP_Z: %d\r\n", dev->compass[2] );
	HAL_UART_Transmit( uart, (uint8_t*) msg, uart_buf_len, HAL_MAX_DELAY );

}

void LIS2MDL_UART_VALUES( LIS2MDL *dev, UART_HandleTypeDef *huart )
{
	uint8_t send_buf[7];

	send_buf[0] = 36;

	for (uint8_t i = 0; i < 2; i++)
	{
		LIS2MDL_ReadCompass( dev );
		for (uint8_t j = 0; j < 3; j++)
		{
			send_buf[2*j+1] = dev->compass[j] & 0xFF;
			send_buf[2*j+2] = (dev->compass[j]>>8) & 0xFF;
		}

		HAL_UART_Transmit(huart, send_buf, 7, 10);
		HAL_Delay(250);
	}

}

uint8_t LIS2MDL_SELF_TEST( LIS2MDL *dev, I2C_HandleTypeDef *i2cHandle, UART_HandleTypeDef *huart )
{
	dev->i2cHandle 		= i2cHandle;

	dev->compass[0]		= 0.0f;
	dev->compass[1]		= 0.0f;
	dev->compass[2]		= 0.0f;

	uint8_t regData;

	uint8_t errorNum = 0;
	HAL_StatusTypeDef status;

	status = LIS2MDL_ReadRegister( dev, LIS2MDL_WHO_AM_I_ADDR, &regData);

	errorNum += ( status != HAL_OK );

	if ( regData != LIS2MDL_WHO_AM_I)
	{
		return 255;
	}

	regData = 0x8C;

	status = LIS2MDL_WriteRegister(dev, LIS2MDL_CFG_REG_A, &regData); // Write data to address

	errorNum += ( status != HAL_OK );

	regData = 0x02;

	status = LIS2MDL_WriteRegister(dev, LIS2MDL_CFG_REG_B, &regData); // Write data to address

	errorNum += ( status != HAL_OK );

	regData = 0x10;

	status = LIS2MDL_WriteRegister(dev, LIS2MDL_CFG_REG_C, &regData); // Write data to address

	errorNum += ( status != HAL_OK );

	HAL_Delay(20);

	uint8_t flag = 0;

	uint8_t rxData[6];

	while (!flag)
	{
		status = LIS2MDL_ReadRegister(dev, LIS2MDL_STATUS_REG, &regData);

		if ((regData >> 3) & 1)
		{
			status = LIS2MDL_ReadRegisters(dev, LIS2MDL_OUTX_L_REG, rxData, 6);
			flag = 1;
		}
	}

	flag = 0;

	for (uint8_t i = 0; i < 50; i++)
	{
		while (!flag)
		{
//			HAL_Delay(20);
			status = LIS2MDL_ReadRegister(dev, LIS2MDL_STATUS_REG, &regData);
			flag = (regData >> 3) & 1;
		}
		status = LIS2MDL_ReadRegisters(dev, LIS2MDL_OUTX_L_REG, rxData, 6);

		flag = 0;

		HAL_UART_Transmit(huart, (uint8_t*)"$", 1, 10);
		HAL_UART_Transmit(huart, rxData, 6, 10);

	}

	regData = 0x12;

	status = LIS2MDL_WriteRegister(dev, LIS2MDL_CFG_REG_C, &regData); // Write data to address

	errorNum += ( status != HAL_OK );

	while (!flag)
	{
		status = LIS2MDL_ReadRegister(dev, LIS2MDL_STATUS_REG, &regData);

		if ((regData >> 3) & 1)
		{
			status = LIS2MDL_ReadRegisters(dev, LIS2MDL_OUTX_L_REG, rxData, 6);
			flag = 1;
		}
	}

	flag = 0;

	for (uint8_t i = 0; i < 50; i++)
	{
		while (!flag)
		{
			status = LIS2MDL_ReadRegister(dev, LIS2MDL_STATUS_REG, &regData);
			flag = (regData >> 3) & 1;
		}
		status = LIS2MDL_ReadRegisters(dev, LIS2MDL_OUTX_L_REG, rxData, 6);

		flag = 0;

		HAL_UART_Transmit(huart, (uint8_t*)"$", 1, 10);
		HAL_UART_Transmit(huart, rxData, 6, 10);

	}

	regData = 0x10;

	status = LIS2MDL_WriteRegister(dev, LIS2MDL_CFG_REG_C, &regData); // Write data to address

	errorNum += ( status != HAL_OK );

	regData = 0x83;
	status = LIS2MDL_WriteRegister(dev, LIS2MDL_CFG_REG_A, &regData); // Write data to address

	errorNum += ( status != HAL_OK );

	return 0;
}


/*
 * LOW-LEVEL FUNCTIONS
 */

HAL_StatusTypeDef LIS2MDL_WriteRegister( LIS2MDL *dev, uint8_t reg, uint8_t *data )
{
	return HAL_I2C_Mem_Write( dev->i2cHandle, LIS2MDL_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY );
}

HAL_StatusTypeDef LIS2MDL_WriteRegisters( LIS2MDL *dev, uint8_t reg, uint8_t *data, uint8_t length )
{
	return HAL_I2C_Mem_Write( dev->i2cHandle, LIS2MDL_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY );
}

HAL_StatusTypeDef LIS2MDL_ReadRegister( LIS2MDL *dev, uint8_t reg, uint8_t *data  )
{
	return HAL_I2C_Mem_Read( dev->i2cHandle, LIS2MDL_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY );
}

HAL_StatusTypeDef LIS2MDL_ReadRegisters( LIS2MDL *dev, uint8_t reg, uint8_t *data, uint8_t length )
{
	return HAL_I2C_Mem_Read( dev->i2cHandle, LIS2MDL_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY );
}


