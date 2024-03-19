/*
 *  LIS2MDL Compass/Magnetometer I2C Driver
 *
 *  Author: Sam Jeffery
 *  Created: 9 March 2022
 */

#ifndef INC_LIS2MDL_H_
#define INC_LIS2MDL_H_

#include "main.h" /* Needed for I2C */

/*
 * DEFINES
 */

#define LIS2MDL_I2C_ADDR (0x1E << 1)

#define LIS2MDL_WHO_AM_I 0x40

/*
 * REGISTERS
 */

#define LIS2MDL_OFFSET_X_REG_L		0x45
#define LIS2MDL_OFFSET_X_REG_H		0x46
#define LIS2MDL_OFFSET_Y_REG_L		0x47
#define LIS2MDL_OFFSET_Y_REG_H		0x48
#define LIS2MDL_OFFSET_Z_REG_L		0x49
#define LIS2MDL_OFFSET_Z_REG_H		0x4A
#define LIS2MDL_WHO_AM_I_ADDR		0x4F
#define LIS2MDL_CFG_REG_A			0x60
#define LIS2MDL_CFG_REG_B			0x61
#define LIS2MDL_CFG_REG_C			0x62
#define LIS2MDL_INT_CRTL_REG		0x63
#define LIS2MDL_INT_SOURCE_REG		0x64
#define LIS2MDL_INT_THS_L_REG		0x65
#define LIS2MDL_INT_THS_H_REG		0x66
#define LIS2MDL_STATUS_REG			0x67
#define LIS2MDL_OUTX_L_REG			0x68
#define LIS2MDL_OUTX_H_REG			0x69
#define LIS2MDL_OUTY_L_REG			0x6A
#define LIS2MDL_OUTY_H_REG			0x6B
#define LIS2MDL_OUTZ_L_REG			0x6C
#define LIS2MDL_OUTZ_H_REG			0x6D
#define LIS2MDL_TEMP_OUT_L_REG		0x6E
#define LIS2MDL_TEMP_OUT_H_REG		0x6F

/*
 * SENSOR STRUCT
 */

typedef struct
{
	/* I2C handle */
	I2C_HandleTypeDef *i2cHandle;

	/* Compass Data (X, Y, Z) */
	int16_t compass[3];

	uint16_t heading;

	/* Calibration */
	int16_t compass_max[3];

	int16_t compass_min[3];

	int16_t offsets[3];

	uint16_t ref_heading;

	/* Temperature data in deg C */
	int16_t temp_C;

} LIS2MDL;

/*
 * INITIALISATION
 */

uint8_t LIS2MDL_SELF_TEST( LIS2MDL *dev, I2C_HandleTypeDef *i2cHandle, UART_HandleTypeDef *huart );

uint8_t LIS2MDL_Initialise( LIS2MDL *dev, I2C_HandleTypeDef *i2cHandle, uint8_t calibrate, uint8_t* button_flag );

/*
 * DATA AQUISITION
 */

HAL_StatusTypeDef LIS2MDL_ReadTemperature( LIS2MDL *dev );
HAL_StatusTypeDef LIS2MDL_ReadCompass( LIS2MDL *dev );


/*
 * HIGH-LEVEL-FUNCTIONS
 */

HAL_StatusTypeDef LIS2MDL_GetData( LIS2MDL *dev );
HAL_StatusTypeDef LIS2MDL_SetOffsets( LIS2MDL *dev );
HAL_StatusTypeDef LIS2MDL_CalibrateCompass( LIS2MDL *dev );
void LIS2MDL_SetRefHeading( LIS2MDL *dev );
void LIS2MDL_SETUP( LIS2MDL *dev, uint8_t *button_flag );

/*
 * UNIT TESTS
 */

void LIS2MDL_PrintHeading( LIS2MDL *dev, UART_HandleTypeDef *uart );

void LIS2MDL_UART_VALUES( LIS2MDL *dev, UART_HandleTypeDef *huart );

/*
 * LOW-LEVEL FUNCTIONS
 */

HAL_StatusTypeDef LIS2MDL_WriteRegister( LIS2MDL *dev, uint8_t reg, uint8_t *data );
HAL_StatusTypeDef LIS2MDL_WriteRegisters( LIS2MDL *dev, uint8_t reg, uint8_t *data, uint8_t length );

HAL_StatusTypeDef LIS2MDL_ReadRegister( LIS2MDL *dev, uint8_t reg, uint8_t *data  );
HAL_StatusTypeDef LIS2MDL_ReadRegisters( LIS2MDL *dev, uint8_t reg, uint8_t *data, uint8_t length );



#endif /* INC_LIS2MDL_H_ */
