/*
 *  EPS Voltage Monitoring ADC Driver
 *
 *  Author: 	Sam Jeffery
 *  Created: 	21 March 2022
 *  Version: 	1.0
 */

#include "stmADC.h"
#include <stdio.h>
/*
 * INITIALISATION
 */

void ADC_Initialise( stmADC *dev, ADC_HandleTypeDef *adc )
{
	dev->adcHandle = adc;

	dev->raw_adc[0] = 0;
	dev->raw_adc[1] = 0;
	dev->raw_adc[2] = 0;
	dev->raw_adc[3] = 0;

}

/*
 * DATA AQUISITION
 */

void ADC_GetALL( stmADC *dev )
{
	ADC_GetTemp( dev );
	ADC_GetVBat( dev );
	ADC_Get5V( dev );
	ADC_Get3V3( dev );
}

void ADC_GetTemp( stmADC *dev )
{
	ADC_Select_CHTemp( dev );
	HAL_ADC_Start( dev->adcHandle );
	HAL_ADC_PollForConversion(dev->adcHandle, HAL_MAX_DELAY);
	dev->raw_adc[0] = HAL_ADC_GetValue( dev->adcHandle );
	HAL_ADC_Stop( dev->adcHandle);
	dev->temp = ((3.3*dev->raw_adc[0]/4095 - V25)/Avg_Slope) + 25;

}

void ADC_GetVBat( stmADC *dev )
{
	ADC_Select_VBat( dev );
	HAL_ADC_Start( dev->adcHandle );
	HAL_ADC_PollForConversion(dev->adcHandle, HAL_MAX_DELAY);
	dev->raw_adc[1] = HAL_ADC_GetValue( dev->adcHandle );
	HAL_ADC_Stop( dev->adcHandle);
//	dev->voltage[0] = dev->raw_adc[1]*0.00411; // Factor = (Voltage Max/ADCSaturation) x (R1bat+R2)/R2) x Correction factor = 3.3/4095 x (120+33)/33 x 1.1 = 0.00411
}

void ADC_Get5V( stmADC *dev )
{
	ADC_Select_5V( dev );
	HAL_ADC_Start( dev->adcHandle );
	HAL_ADC_PollForConversion(dev->adcHandle, HAL_MAX_DELAY);
	dev->raw_adc[2] = HAL_ADC_GetValue( dev->adcHandle );
	HAL_ADC_Stop( dev->adcHandle);
//	dev->voltage[1] = dev->raw_adc[2]*0.00271; // Factor = (Voltage Max/ADCSaturation) x (R1bat+R2)/R2) x Correction factor = 3.3/4095 x (68+33)/33 x 1.1 = 0.00271
}

void ADC_Get3V3( stmADC *dev )
{
	ADC_Select_3V3( dev );
	HAL_ADC_Start( dev->adcHandle );
	HAL_ADC_PollForConversion(dev->adcHandle, HAL_MAX_DELAY);
	dev->raw_adc[3] = HAL_ADC_GetValue( dev->adcHandle );
	HAL_ADC_Stop( dev->adcHandle);
//	dev->voltage[2] = Mdev->raw_adc[3]*0.00273; // Factor = (Voltage Max/ADCSaturation) x (R1bat+R2)/R2) x Correction factor = 3.3/4095 x (68+33)/33 x 1.1 = 0.00271
}

/*
 * LOW-LEVEL FUNCTIONS
 */

uint8_t ADC_Select_CHTemp ( stmADC *dev )
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5; //Selected for balance of accuracy and time
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel( dev->adcHandle, &sConfig) != HAL_OK)
	{
		return 0;
	}
	return 1;
}

uint8_t ADC_Select_3V3 ( stmADC *dev )
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel( dev->adcHandle, &sConfig) != HAL_OK)
	{
		return 0;
	}
	return 1;
}

uint8_t ADC_Select_VBat ( stmADC *dev )
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_15;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel( dev->adcHandle, &sConfig) != HAL_OK)
	{
		return 0;
	}
	return 1;
}

uint8_t ADC_Select_5V ( stmADC *dev )
{
	ADC_ChannelConfTypeDef sConfig = {0};
	sConfig.Channel = ADC_CHANNEL_16;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel( dev->adcHandle, &sConfig) != HAL_OK)
	{
		return 0;
	}
	return 1;
}

/*
 * UNIT TESTS
 */

void ADC_DisplayReadings( stmADC *dev, UART_HandleTypeDef *output, uint8_t *msg, uint8_t msg_len )
{
	ADC_GetALL( dev );
	msg_len = sprintf((char*)msg, "Temp: %d, Vsys: %d, 5VRead: %d, 3V3Read: %d \n\r", dev->raw_adc[0], dev->raw_adc[1], dev->raw_adc[2], dev->raw_adc[3]);
	HAL_UART_Transmit(output, msg, msg_len, HAL_MAX_DELAY);
	msg_len = sprintf((char*)msg, "Temp: %f, Vsys: %f, 5VRead: %f, 3V3Read: %f \n\r", dev->temp, dev->voltage[0], dev->voltage[1], dev->voltage[2]);
	HAL_UART_Transmit(output, msg, msg_len, HAL_MAX_DELAY);

}
