/*
 *  ADS7828 Driver
 *
 *  Author: 	Sam Jeffery
 *  Created: 	21 March 2022
 *  Version: 	1.0
 */

#ifndef INC_ADS7828_H_
#define INC_ADS7828_H_

#include "main.h"

/*
 * DEFINES
 */

#define ADS7828_I2C_ADDR 	0x48<<1
#define ADS7828_CONFIG 		0xDC //This one can be made to be changeable

/*
 * Driver STRUCT
 */

typedef struct{

	I2C_HandleTypeDef *i2cHandle;
	/* Addresses for current readings
	0 = 5VSYS 	1 = 5VCH1 	2 = 5VCH2 	3 = 5VCH3
	7 = 3V3SYS 	4 = 3V3CH1 	5 = 3V3CH2 	6 = 3V3 CH3
	*/
	uint8_t i2c_err;

	uint16_t raw_Data[8];

	float current_output[8];

} ADS7828;


/*
 * INITIALISATION
 */

void ADS7828_Initialise( ADS7828 *dev, I2C_HandleTypeDef *I2CHandle );

/*
 * DATA AQUISITION
 */

uint8_t ADS7828_readSelected( ADS7828 *dev, uint8_t conv );

uint8_t ADS7828_readADC( ADS7828 *dev, uint8_t inputNo );

uint8_t ADS7828_convertRaw( ADS7828 *dev, uint8_t inputNo );

/*
 * LOW-LEVEL FUNCTIONS
 */

HAL_StatusTypeDef ADS7828_WriteRegister( ADS7828 *dev, uint8_t *data );

HAL_StatusTypeDef ADS7828_ReadRegister( ADS7828 *dev, uint8_t *data  );

/*
 * UNIT TESTS
 */


#endif /* INC_ADS7828_H_ */
