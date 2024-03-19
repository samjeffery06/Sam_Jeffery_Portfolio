/*
 *  ISM330 Accelerometer and Gyroscope I2C Driver
 *
 *  Author: 	Sam Jeffery
 *  Created: 	9 March 2022
 *  Version:	1.0
 */


#ifndef INC_ISM330_H_
#define INC_ISM330_H_

#include "main.h" /* Needed for I2C */

/*
 * DEFINES
 */

#define ISM330_I2C_ADDR 	(0x6A << 1)

#define ISM330_WHO_AM_I 		0x6B

/*
 * REGISTERS
 */

#define ISM330_FUNC_CFG_ACCESS		0x01
#define ISM330_PIN_CTRL				0x02
#define ISM330_FIFO_CTRL1			0x07
#define ISM330_FIFO_CTRL2			0x08
#define ISM330_FIFO_CTRL3			0x09
#define ISM330_FIFO_CTRL4			0x0A
#define ISM330_INT1_CTRL			0x0D
#define ISM330_INT2_CTRL			0x0E
#define ISM330_WHO_AM_I_ADDR		0x0F
#define ISM330_CTRL1_XL				0x10
#define ISM330_CTRL2_G				0x11
#define ISM330_CTRL6_C				0x15
#define ISM330_CTRL7_G				0x16
#define ISM330_CTRL9_XL				0x18
#define ISM330_CTRL10_C				0x19
#define ISM330_STATUS_REG			0x1E
#define ISM330_OUT_TEMP_L			0x20
#define ISM330_OUT_TEMP_H			0x21
#define ISM330_OUTX_L_G				0x22
#define ISM330_OUTX_H_G				0x23
#define ISM330_OUTY_L_G				0x24
#define ISM330_OUTY_H_G				0x25
#define ISM330_OUTZ_L_G				0x26
#define ISM330_OUTZ_H_G				0x27
#define ISM330_OUTX_L_A				0x28
#define ISM330_OUTX_H_A				0x29
#define ISM330_OUTY_L_A				0x2A
#define ISM330_OUTY_H_A				0x2B
#define ISM330_OUTZ_L_A				0x2C
#define ISM330_OUTZ_H_A				0x2D
#define ISM330_X_OFS_USR			0x73
#define ISM330_Y_OFS_USR			0x74
#define ISM330_Z_OFS_USR			0x75



/*
 * SENSOR STRUCT
 */

typedef struct
{
	/* I2C handle */
	I2C_HandleTypeDef *i2cHandle;

	/*
	 * Accelerometer/Gyroscope Data (X, Y, Z)
	 */

	int16_t acc_raw[3];

//	float acc[3];

	int16_t gyro_raw[3];

//	float gyro[3];

	/* Temperature data in deg C */

	int16_t temp_C_raw;

//	float temp_C;

} ISM330;

/*
 * INITIALISATION
 */

uint8_t ISM330_Initialise( ISM330 *dev, I2C_HandleTypeDef *i2cHandle );

/*
 * DATA AQUISITION
 */

HAL_StatusTypeDef ISM330_ReadTemperature( ISM330 *dev );
HAL_StatusTypeDef ISM330_ReadGyro( ISM330 *dev );
HAL_StatusTypeDef ISM330_ReadAcc( ISM330 *dev );

/*
 * UNIT TESTS
 */

void ISM330_PrintReadings( ISM330 *dev, UART_HandleTypeDef *uart );

/*
 * LOW-LEVEL FUNCTIONS
 */

HAL_StatusTypeDef ISM330_WriteRegister( ISM330 *dev, uint8_t reg, uint8_t *data );
HAL_StatusTypeDef ISM330_WriteRegisters( ISM330 *dev, uint8_t reg, uint8_t *data, uint8_t length );

HAL_StatusTypeDef ISM330_ReadRegister( ISM330 *dev, uint8_t reg, uint8_t *data  );
HAL_StatusTypeDef ISM330_ReadRegisters( ISM330 *dev, uint8_t reg, uint8_t *data, uint8_t length );

#endif /* INC_ISM330_H_ */
