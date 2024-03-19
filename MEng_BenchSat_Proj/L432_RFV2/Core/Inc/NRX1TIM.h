/*
 *  NRX1 Driver
 *
 *  Author: 	Sam Jeffery
 *  Created: 	21 March 2022
 *  Version: 	1.0
 */

#ifndef INC_NRX1_H_
#define INC_NRX1_H_

#include "main.h"


/*
 * Code to be put into USER CODE 0:
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == &htim16)
	{
		NRX1_TIMRX( &vhf_TIM );
	}
}
 */

/*
 * DEFINES
 */

#define SAMPLES_PER_BIT 8

#define CRC_POLY		0x8408

#define RSSI_BACKGND 55 // Sampled background noise on NRX1 in ESL



/*
 * Driver STRUCT
 */

typedef struct{

	GPIO_TypeDef *RX_GPIO;

	uint16_t RX_Pin;

	uint8_t buffer[64];

	uint16_t packetsize;

	uint8_t RX_state;

	uint8_t flag_end;

	uint8_t ones_in_a_row;

	uint8_t zeros_in_a_row;

	uint8_t last_run;

	ADC_HandleTypeDef *adcHandle;

	uint8_t count[2];

	uint8_t adc_RSSI;

} NRX1_TIM;


/*
 * INITIALISATION
 */

void NRX1_Initialise_TIM( NRX1_TIM *dev, GPIO_TypeDef *RX_GPIO, uint16_t RX_Pin, ADC_HandleTypeDef *adcHandle, TIM_HandleTypeDef *htim );

/*
 * DATA AQUISITION
 */

void NRX1_addBits_TIM( NRX1_TIM *dev, uint8_t bit, uint8_t save_run );

void NRX1_TIMRX( NRX1_TIM *dev );

void getRSSI_TIM( NRX1_TIM *dev );

uint8_t check_crc( NRX1_TIM *dev );

void update_crc( uint16_t *crc, uint8_t bit);

void compare_count( NRX1_TIM *dev );

#endif /* INC_NRX1_H_ */
