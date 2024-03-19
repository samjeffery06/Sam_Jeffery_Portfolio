/*
 *  NRX1 Driver
 *
 *  Author: 	Sam Jeffery
 *  Created: 	21 March 2022
 *  Version: 	1.0
 */

#include <NRX1UART.h>
#include <stdio.h>

/*
 * INITIALISATION
 */

void NRX1_Initialise_UART( NRX1_UART *dev, UART_HandleTypeDef *uartHandle, ADC_HandleTypeDef *adcHandle )
{
	dev->uartHandle = uartHandle;

	dev->adcHandle = adcHandle;

	HAL_UART_Receive_IT(dev->uartHandle, &dev->UART_RX, 1);

}

/*
 * DATA AQUISITION
 */

/*
 * LOW-LEVEL FUNCTIONS
 */

void NRX1_UARTInterruptRX( NRX1_UART *dev )
{
	if (dev->UART_RX == '*')
	{
		dev->flag_sync = 1;
	}
	else if (dev->flag_start && dev->UART_RX == '~')
	{
		dev->rx_time = HAL_GetTick() - dev->rx_time;
		dev->flag_start = 0;
		dev->flag_end = 1;
		HAL_GPIO_WritePin(NRX1_LD_GPIO_Port, NRX1_LD_Pin, 0);
	}
	else if (dev->flag_sync == 1)
	{
		if(dev->UART_RX == 'S')
		{
			HAL_GPIO_WritePin(NRX1_LD_GPIO_Port, NRX1_LD_Pin, 1);
			dev->rx_time = HAL_GetTick();
			dev->flag_start = 1;
			dev->count = 0;
			getRSSI_UART( dev );
		}
		dev->flag_sync = 0;
	}
	if (dev->flag_start)
	{
		/*
		 * Rough solution for fact that 0s aren't being sent correctly
		 * GS adds a 0xFF before all 0s so they are sent correctly
		 * On RFM we overwrite previous RX byte when 0 comes through
		 */
		if (dev->UART_RX == 0)
			dev->count--;
		dev->buffer[dev->count] = dev->UART_RX;
		dev->count++;
	}
	HAL_UART_Receive_IT(dev->uartHandle, &dev->UART_RX, 1);
}

void getRSSI_UART( NRX1_UART *dev )
{
	HAL_ADC_Start( dev->adcHandle );

	HAL_ADC_PollForConversion(dev->adcHandle, HAL_MAX_DELAY);

	dev->adc_RSSI = HAL_ADC_GetValue( dev->adcHandle );

	HAL_ADC_Stop( dev->adcHandle );
}

/*
 * UNIT TESTS
 */

void NRX1_UART_MSG_OUTPUT( NRX1_UART *dev, UART_HandleTypeDef *huartOut)
{
	if ( dev->flag_end )
	{
		uint8_t uart_buf_len;
		uint8_t tx_msg[20];

		HAL_UART_Transmit( huartOut, dev->buffer, dev->count, HAL_MAX_DELAY );

		uart_buf_len = sprintf( (char*) tx_msg, "\n\rRX Time %lu\n\r", dev->rx_time );
		HAL_UART_Transmit( huartOut, tx_msg, uart_buf_len, HAL_MAX_DELAY );

		float output = (float) dev->adc_RSSI/RSSI_BACKGND;
		uart_buf_len = sprintf( (char*) tx_msg, "RSSI/BGND %.2f\n\r", output );
		HAL_UART_Transmit( huartOut, tx_msg, uart_buf_len, HAL_MAX_DELAY );

		dev->flag_end = 0;
		dev->count = 0;
	}
}
