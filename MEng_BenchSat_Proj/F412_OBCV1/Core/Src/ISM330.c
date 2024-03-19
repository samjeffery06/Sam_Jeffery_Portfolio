/*
 *  ISM330 Accelerometer and Gyroscope I2C Driver
 *
 *  Author: 	Sam Jeffery
 *  Created: 	9 March 2022
 *  Version: 	1.0
 */

#include "ISM330.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

/*
 * INITIALISATION
 */

uint8_t ISM330_Initialise( ISM330 *dev, I2C_HandleTypeDef *i2cHandle )
{
	/* Set STRUCT parameters */

    dev->i2cHandle 	= i2cHandle;

//	dev->acc[0]			= 0.0f;
//	dev->acc[1]			= 0.0f;
//	dev->acc[2]			= 0.0f;
//
//	dev->gyro[0]		= 0.0f;
//	dev->gyro[1]		= 0.0f;
//	dev->gyro[2]		= 0.0f;

//	dev->temp_C			= 0.0f;

	uint8_t errorNum = 0;
	HAL_StatusTypeDef status;

	/*
	 * Check WHO_AM_I
	 */
	uint8_t regData;

	status = ISM330_ReadRegister( dev, ISM330_WHO_AM_I_ADDR, &regData);

	errorNum += ( status != HAL_OK );

	if ( regData != ISM330_WHO_AM_I)
	{
		return 255;
	}

	/*
	 * Initialization of ISM330
	 */

	/*
	 * Interrupt Enabling, NOT USED CURRENTLY
	 *
	regData = 0x01; // Activate INT1_DRDY_XL

	status = ISM330_WriteRegister( dev, ISM330_INT1_CTRL, &regData ); // Write data to address

	errorNum += ( status != HAL_OK );

	regData = 0x02; // Activate INT2_DRDY_G

	status = ISM330_WriteRegister( dev, ISM330_INT2_CTRL, &regData ); // Write data to address

	errorNum += ( status != HAL_OK );

	*
	*/

//	regData = 0x10; //12.5 Hz, +-2g, default filtering -- FOR LOW POWER SET (CTRL6_C)

	regData = 0x60; //416 Hz , +-2g, HIGH PERFORMANCE

	status = ISM330_WriteRegister( dev, ISM330_CTRL1_XL, &regData ); // Write data to address

	errorNum += ( status != HAL_OK );

//	regData = 0x10; //12.5 Hz , +-250dps -- FOR LOW POWER SET (CTRL7_G)

	regData = 0x60; //416 Hz , +-250dps HIGH PERFORMANCE

	status = ISM330_WriteRegister( dev, ISM330_CTRL2_G, &regData ); // Write data to address

	errorNum += ( status != HAL_OK );

	regData = 0xE2; //Leaves DEN values as default, Sets Device_CONF

	status = ISM330_WriteRegister( dev, ISM330_CTRL9_XL, &regData ); // Write data to address

	errorNum += ( status != HAL_OK );

//	regData = 0x20; //Set timestamps
//
//	status = ISM330_WriteRegister( dev, ISM330_CTRL9_XL, &regData ); // Write data to address
//
//	errorNum += ( status != HAL_OK );

	regData = 0x00; // CONTROL WEIGHTING OF OFFSETS

	status = ISM330_WriteRegister( dev, ISM330_CTRL6_C, &regData ); // Write data to address

	errorNum += ( status != HAL_OK );

	regData = 0x02; //ENABLE OFFSETS

	status = ISM330_WriteRegister( dev, ISM330_CTRL7_G, &regData ); // Write data to address

	errorNum += ( status != HAL_OK );

//	int8_t regData2;

//	regData = -5; //SET X OFFSET
//
//	status = ISM330_WriteRegister( dev, ISM330_X_OFS_USR, &regData ); // Write data to address
//
//	errorNum += ( status != HAL_OK );
//
//	regData = -22; //SET Y OFFSET
//
//	status = ISM330_WriteRegister( dev, ISM330_Y_OFS_USR, &regData ); // Write data to address

	errorNum += ( status != HAL_OK );

	regData = 31; //SET Z OFFSET

	status = ISM330_WriteRegister( dev, ISM330_Z_OFS_USR, &regData ); // Write data to address

	errorNum += ( status != HAL_OK );

	return errorNum;
}

/*
 * DATA AQUISITION
 */

HAL_StatusTypeDef ISM330_ReadTemperature( ISM330 *dev )
{
	uint8_t regData[2];
	HAL_StatusTypeDef status;

	do
	{
		status = ISM330_ReadRegister(dev, ISM330_STATUS_REG, regData);
		if (status != HAL_OK)
		{
			return status;
		}
	}
	while (!(regData[0]>>2 & 1));

	status = ISM330_ReadRegisters(dev, ISM330_OUT_TEMP_L, regData, 2);

	dev->temp_C_raw = (regData[1] << 8) | regData[0];

//	dev->temp_C = (MATH);

	return status;
}

HAL_StatusTypeDef ISM330_ReadGyro( ISM330 *dev )
{
	HAL_StatusTypeDef status;

	uint8_t regData[6];

	do
	{
		status = ISM330_ReadRegister(dev, ISM330_STATUS_REG, regData);
		if (status != HAL_OK)
		{
			return status;
		}
	}
	while (!(regData[0]>>1 & 1));

	status = ISM330_ReadRegisters(dev, ISM330_OUTX_L_G, regData, 6);

	dev->gyro_raw[0] = ((regData[1] << 8) | regData[0]);

	dev->gyro_raw[1] = ((regData[3] << 8) | regData[2]);

	dev->gyro_raw[2] = ((regData[5] << 8) | regData[4]);

//	dev->gyro[0] = (MATH);
//
//	dev->gyro[1] = (MATH);
//
//	dev->gyro[2] = (MATH);

	return status;
}

HAL_StatusTypeDef ISM330_ReadAcc( ISM330 *dev )
{
	uint8_t regData[6];

	HAL_StatusTypeDef status;

	do
	{
		status = ISM330_ReadRegister(dev, ISM330_STATUS_REG, regData);
		if (status != HAL_OK)
		{
			return status;
		}
	}
	while (!(regData[0] & 1));

	status = ISM330_ReadRegisters(dev, ISM330_OUTX_L_A, regData, 6);

	dev->acc_raw[0] = ((regData[1] << 8) | regData[0]);

	dev->acc_raw[1] = ((regData[3] << 8) | regData[2]);

	dev->acc_raw[2] = ((regData[5] << 8) | regData[4]);

	/*
	 * Range is +-2g, 16bit signed integer => 15bit for 2g
	 * Acceleration (m/s^2) = (ISM reading)*(9.81m/s^2)*2/32768
	 */

//	dev->acc[0] = dev->acc_raw[0]*0.0005987488;
//
//	dev->acc[1] = dev->acc_raw[1]*0.0005987488;
//
//	dev->acc[2] = dev->acc_raw[2]*0.0005987488;

	return status;
}

/*
 * UNIT TESTS
 */
/*
void ISM330_PrintReadings( ISM330 *dev, UART_HandleTypeDef *uart )
{
	char msg[20];

	uint8_t uart_buf_len;

	ISM330_ReadTemperature( dev );

	uart_buf_len = sprintf(msg, "TempRaw: %.0f\r\n", dev->temp_C);
	HAL_UART_Transmit(uart, (uint8_t*) msg, uart_buf_len, HAL_MAX_DELAY);

	ISM330_ReadAcc( dev );

	uart_buf_len = sprintf(msg, "ACC_X: %.4f\r\n", dev->acc[0]);
	HAL_UART_Transmit(uart, (uint8_t*) msg, uart_buf_len, HAL_MAX_DELAY);

	uart_buf_len = sprintf(msg, "ACC_Y: %.4f\r\n", dev->acc[1]);
	HAL_UART_Transmit(uart, (uint8_t*) msg, uart_buf_len, HAL_MAX_DELAY);

	uart_buf_len = sprintf(msg, "ACC_Z: %.4f\r\n", dev->acc[2]);
	HAL_UART_Transmit(uart, (uint8_t*) msg, uart_buf_len, HAL_MAX_DELAY);

	ISM330_ReadGyro( dev );

	uart_buf_len = sprintf(msg, "GYRO_X: %.0f\r\n", dev->gyro[0]);
	HAL_UART_Transmit(uart, (uint8_t*) msg, uart_buf_len, HAL_MAX_DELAY);

	uart_buf_len = sprintf(msg, "GYRO_Y: %.0f\r\n", dev->gyro[1]);
	HAL_UART_Transmit(uart, (uint8_t*) msg, uart_buf_len, HAL_MAX_DELAY);

  	uart_buf_len = sprintf(msg, "GYRO_Z: %.0f\r\n", dev->gyro[2]);
	HAL_UART_Transmit(uart, (uint8_t*) msg, uart_buf_len, HAL_MAX_DELAY);


//	Set up print for sq root of squares to get magnitude of acc vector

	float total = sqrtf( dev->acc[0]*dev->acc[0] + dev->acc[1]*dev->acc[1] + dev->acc[2]*dev->acc[2] );
//	float total = 9.81;

	uart_buf_len = sprintf(msg, "Sum of Sq: %.4f\r\n", total);
	HAL_UART_Transmit(uart, (uint8_t*) msg, uart_buf_len, HAL_MAX_DELAY);


}
*/

/*
 * LOW-LEVEL FUNCTIONS
 */

HAL_StatusTypeDef ISM330_WriteRegister( ISM330 *dev, uint8_t reg, uint8_t *data )
{
	return HAL_I2C_Mem_Write( dev->i2cHandle, ISM330_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY );
}

HAL_StatusTypeDef ISM330_WriteRegisters( ISM330 *dev, uint8_t reg, uint8_t *data, uint8_t length )
{
	return HAL_I2C_Mem_Write( dev->i2cHandle, ISM330_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY );
}

HAL_StatusTypeDef ISM330_ReadRegister( ISM330 *dev, uint8_t reg, uint8_t *data  )
{
	return HAL_I2C_Mem_Read( dev->i2cHandle, ISM330_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY );
}

HAL_StatusTypeDef ISM330_ReadRegisters( ISM330 *dev, uint8_t reg, uint8_t *data, uint8_t length )
{
	return HAL_I2C_Mem_Read( dev->i2cHandle, ISM330_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, data, length, HAL_MAX_DELAY );
}
