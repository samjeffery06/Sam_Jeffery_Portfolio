/*
 *  NRX1 Driver
 *
 *  Author: 	Sam Jeffery
 *  Created: 	21 March 2022
 *  Version: 	1.0
 */

#include <NRX1TIM.h>
#include <stdio.h>

/*
 * INITIALISATION
 */

void NRX1_Initialise_TIM( NRX1_TIM *dev, GPIO_TypeDef *RX_GPIO, uint16_t RX_Pin, ADC_HandleTypeDef *adcHandle, TIM_HandleTypeDef *htim )
{
	dev->RX_GPIO = RX_GPIO;

	dev->RX_Pin = RX_Pin;

	dev->adcHandle = adcHandle;

	HAL_TIM_Base_Start_IT(htim);

//	dev->count = 0;
}

/*
 * DATA AQUISITION
 */

void NRX1_TIMRX( NRX1_TIM *dev )
{
	uint8_t val = 0;
	uint8_t change = 0;
	uint8_t save_run = 0;
	if (HAL_GPIO_ReadPin(dev->RX_GPIO, dev->RX_Pin))
	{
		if (dev->zeros_in_a_row)
		{
			save_run = dev->zeros_in_a_row;
			val = 0;
			change = 1;
		}
		dev->zeros_in_a_row = 0;
		dev->ones_in_a_row++;
	}
	else
	{
		if (dev->ones_in_a_row)
		{
			save_run = dev->ones_in_a_row;
			val = 1;
			change = 1;
		}
		dev->ones_in_a_row = 0;
		dev->zeros_in_a_row++;
	}
	if (change)
	{
		if (!dev->RX_state) // State = 0: Normal running, first flag not received
		{
			if (save_run > (SAMPLES_PER_BIT*5 + SAMPLES_PER_BIT/2) && save_run < (SAMPLES_PER_BIT*6 + SAMPLES_PER_BIT/2))
			{
				dev->RX_state = 1;
				dev->adc_RSSI = 1;
			}
		}
		else if(dev->RX_state == 1) // State 1: Transition State, first flag received but message not started, checking message correct
		{
			if ((SAMPLES_PER_BIT/2) < save_run && save_run < (SAMPLES_PER_BIT + SAMPLES_PER_BIT/2))
			{
				dev->RX_state = 2;
			}
			else
			{
				dev->RX_state = 0;
			}
		}
		else if ( dev->RX_state == 2 ) // State 2: Receiving Message
		{
			if (save_run > (SAMPLES_PER_BIT*5 + SAMPLES_PER_BIT/2-1) && dev->packetsize >= 24)
			{
				if (check_crc(dev))
				{
					dev->flag_end = 1;
					dev->count[0]++;
//					if (dev->packetsize < 100)
//					{
//						dev->count[0]++;
//					}
//					else if (dev->packetsize < 350)
//					{
//						dev->count[2]++;
//					}
//					else
//					{
//						dev->count[4]++;
//					}
					//SEND ACK
				}
				else
				{
					//SEND NACK
					dev->flag_end = 0;
					dev->count[1]++;
//					if (dev->packetsize < 100)
//					{
//						dev->count[1]++;
//					}
//					else if (dev->packetsize < 350)
//					{
//						dev->count[3]++;
//					}
//					else
//					{
//						dev->count[5]++;
//					}
				}

				dev->RX_state = 0;
				dev->last_run = 0;
				dev->adc_RSSI = 0;
				dev->packetsize = 0;
			}
			else if (save_run < (SAMPLES_PER_BIT*6 + SAMPLES_PER_BIT/2))
			{
				NRX1_addBits_TIM(dev, val, save_run);
			}
		}
		else if (dev->RX_state == 3) // State 3: Error State, Message too long or error in receive
		{
			// SEND NACK
			dev->packetsize = 0;
			dev->RX_state = 0;
			dev->last_run = 0;
			dev->adc_RSSI = 0;
		}
	}

}

void NRX1_addBits_TIM( NRX1_TIM *dev, uint8_t bit, uint8_t save_run)
{
	uint8_t loops = (save_run+(SAMPLES_PER_BIT/2)-1)/SAMPLES_PER_BIT; // Plus 3 for the under counted case (ie 9 samples instead of 10, still counts as 2 1s
	if (loops > 5)
	{
		dev->RX_state = 3;
		return;
	}
	if (loops == 0)
	{
		loops = 1;
	}
	if (dev->last_run == 5) // Accounts bit stuffing
	{
		dev->last_run = loops;
		loops--;
	}
	else
		dev->last_run = loops;

	for (uint8_t i = 0; i < loops; i++)
	{
		dev->buffer[dev->packetsize >> 3] &= ~(1 << (dev->packetsize & 0x7)); // CLEAR POSITION
		dev->buffer[dev->packetsize >> 3] |= bit << (dev->packetsize & 0x7);  // SET POSITION
		dev->packetsize++;
	}
	if (dev->packetsize >= 24 && dev->packetsize < 32) // CONFIRM THAT SYNC HAS BEEN RECEIVED CORRECTLY
	{
		if (dev->buffer[0] != 83 || dev->buffer[1] != 65 || dev->buffer[2] != 84)
		{
			dev->RX_state = 3;
		}
	}
	if (dev->packetsize > 512) // CHECK IF PACKET HAS REACHED MAX SIZE
	{
		dev->RX_state = 3;
	}
}

uint8_t check_crc( NRX1_TIM *dev )
{
	uint16_t crc = 0;
	for(uint16_t i = 0; i < dev->packetsize-17; i++)
	{
		update_crc(&crc, (dev->buffer[i>>3] >> (i & 0x7)) & 0x1);
	}

	if (crc == (dev->buffer[(dev->packetsize>>3)-2] << 8) + dev->buffer[(dev->packetsize>>3)-1])
		return 1;
	else
		return 0;
}

void update_crc( uint16_t *crc, uint8_t bit)
{
	*crc ^= bit;
	if (*crc & 1)
		*crc = (*crc >> 1) ^ CRC_POLY;
	else
		*crc = *crc >> 1;
}


/*
 * LOW-LEVEL FUNCTIONS
 */



void getRSSI_TIM( NRX1_TIM *dev )
{
	HAL_ADC_Start( dev->adcHandle );

	HAL_ADC_PollForConversion(dev->adcHandle, HAL_MAX_DELAY);

	dev->adc_RSSI = HAL_ADC_GetValue( dev->adcHandle );

	HAL_ADC_Stop( dev->adcHandle );
}

/*
 * UNIT TESTS
 */
