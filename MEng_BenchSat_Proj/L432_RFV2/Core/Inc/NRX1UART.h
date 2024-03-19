/*
 *  NRX1 Driver
 *
 *  Author: 	Sam Jeffery
 *  Created: 	21 March 2022
 *  Version: 	1.0
 */

#ifndef INC_NRX1UART_H_
#define INC_NRX1UART_H_

#include "main.h"


/*
 * Code to be put into USER CODE 0:
 void HAL_UART_RxCpltCallback( UART_HandleTypeDef *huart )
{
	NRX1_InterruptRX( &vhf );
}
 */

/*
 * DEFINES
 */

#define RSSI_BACKGND 55 // Sampled background noise on NRX1 in ESL

/*
 * Driver STRUCT
 */

typedef struct{

	UART_HandleTypeDef *uartHandle;

	uint8_t buffer[128];

	uint8_t flag_start;

	uint8_t flag_end;

	uint8_t flag_sync;

	uint8_t count;

	uint32_t rx_time;

	uint8_t UART_RX;

	ADC_HandleTypeDef *adcHandle;

	uint8_t adc_RSSI;

} NRX1_UART;


/*
 * INITIALISATION
 */

void NRX1_Initialise_UART( NRX1_UART *dev, UART_HandleTypeDef *uartHandle, ADC_HandleTypeDef *adcHandle );

/*
 * DATA AQUISITION
 */

/*
 * LOW-LEVEL FUNCTIONS
 */

void NRX1_UARTInterruptRX( NRX1_UART *dev );

void getRSSI_UART( NRX1_UART *dev );

/*
 * UNIT TESTS
 */

void NRX1_UART_MSG_OUTPUT( NRX1_UART *dev, UART_HandleTypeDef *huartOut);

#endif /* INC_NRX1UART_H_ */
