/*
 *  ADS7828 Driver
 *
 *  Author: 	Sam Jeffery
 *  Created: 	21 March 2022
 *  Version: 	1.0
 */

#include "ADS7828.h"

/*
 * INITIALISATION
 */

void ADS7828_Initialise( ADS7828 *dev, I2C_HandleTypeDef *I2CHandle )
{
	dev->i2cHandle = I2CHandle;

	dev->i2c_err = ADS7828_readADC( dev, 0x1 );


	// Put in check for I2C Connect
}

/*
 * DATA AQUISITION
 */

uint8_t ADS7828_readSelected( ADS7828 *dev, uint8_t conv )
{
	if (dev->i2c_err > 0)
	{
		return 1;
	}
	/* Addresses for current readings
	0 = 5VSYS 	1 = 5VCH1 	2 = 5VCH2 	3 = 5VCH3
	7 = 3V3SYS 	4 = 3V3CH1 	5 = 3V3CH2 	6 = 3V3 CH3
	*/
	uint8_t bit = 0x01;
	for ( uint8_t i = 0; i < 8; i++ )
	{
		if( (conv & bit)>>i == 1 )
		{
			ADS7828_readADC( dev, i );
			ADS7828_convertRaw( dev, i );
		}
		bit *= 2;
	}
	return 0;
}

uint8_t ADS7828_readADC( ADS7828 *dev, uint8_t inputNo )
{
	if (dev->i2c_err > 0)
	{
		return 1;
	}
	uint8_t addata[2];
	uint8_t cmd = 0x8F;
	uint8_t errors = 0;

	if ( inputNo%2 == 0 )
		cmd |= (inputNo/2)<<4;
	else
		cmd |= (((inputNo-1)/2)<<4) | 0x40;

	errors += ( ADS7828_WriteRegister(dev, &cmd) != HAL_OK );

	errors += ( ADS7828_ReadRegister(dev, addata) != HAL_OK );

	dev->raw_Data[inputNo] = ((addata[0] & 0x0F)<<8) | addata[1];

	return errors;
}

uint8_t ADS7828_convertRaw( ADS7828 *dev, uint8_t inputNo )
{
	dev->current_output[inputNo] = (3.3*dev->current_output[inputNo]/4095);
	return 0;
}

/*
 * LOW-LEVEL FUNCTIONS
 */

HAL_StatusTypeDef ADS7828_WriteRegister( ADS7828 *dev, uint8_t *data )
{
	return HAL_I2C_Master_Transmit( dev->i2cHandle, ADS7828_I2C_ADDR, data, 1, 100 );
}

HAL_StatusTypeDef ADS7828_ReadRegister( ADS7828 *dev, uint8_t *data  )
{
	return HAL_I2C_Master_Receive( dev->i2cHandle, ADS7828_I2C_ADDR, data, 2, 100 );
}


/*
 * UNIT TESTS
 */
