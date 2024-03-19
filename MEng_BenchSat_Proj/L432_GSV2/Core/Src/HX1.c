/*
 *  HX1 Narrowband Transmitter Driver
 *
 *  Author: Sam Jeffery
 *  Created: 14 March 2022
 */

#include "HX1.h"

/*
 * INITIALISATION
 */

void HX1_Initialise( HX1 *dev, UART_HandleTypeDef *uartHandleRF, UART_HandleTypeDef *uartHandleGS, GPIO_TypeDef *EN_GPIO, uint16_t EN_Pin )
{
	// Set UART handle that will be used to transmit messages
	dev->uartHandleRF	=	uartHandleRF;

	// Set UART handle to communicate with GS
	dev->uartHandleGS	= 	uartHandleGS;

	// Set up Enable pin for the transmitter
	dev->EN_GPIO	=	EN_GPIO;

	dev->EN_Pin		=	EN_Pin;

//	HAL_UART_Receive_IT( dev->uartHandleGS, dev->rx_gs, GS_MSG_SIZE);

	HAL_UART_Receive_IT( dev->uartHandleGS, dev->rx_gs, 2);

}

/*
 * HIGH LEVEL FUNCTIONS
 */

void HX1_UARTGS( HX1 *dev)
{
	// Check for start CHAR
	if ( dev->rx_gs[0] == 0x55 )
	{
		// Increase start CHAR to show message rx
		dev->rx_gs[0]++;
		// Set Interrupt to RX rest of message
		HAL_UART_Receive_IT( dev->uartHandleGS, &dev->rx_gs[2], dev->rx_gs[1]+1);
	}
	else if (dev->rx_gs[0] == 0x56)
	{
		// Check if message is fully sent
		if (dev->rx_gs[dev->rx_gs[1]+2] == 0xAA)
		{
			//Set UART message rx flag
			dev->rx_flag = 1;
		}
		//Reset Interrupt to RX 2 bytes
		HAL_UART_Receive_IT( dev->uartHandleGS, dev->rx_gs, 2);
	}
	else if (dev->rx_gs[0] == 0x80 && dev->rx_gs[1] == 0x80)
	{
		// For handshake, no time pressure here so can take time in interrupt
		HAL_UART_Receive_IT( dev->uartHandleGS, dev->rx_gs, 2);
		HAL_UART_Transmit( dev->uartHandleGS, (uint8_t*) "$\n", 2, HAL_MAX_DELAY );
	}
}

HAL_StatusTypeDef HX1_PROCESS( HX1 *dev )
{
	if ( dev->rx_flag )
	{
		HX1_SendMessage( dev, &dev->rx_gs[2], dev->rx_gs[1] );
		if (dev->rx_gs[2] == 0)
			HAL_UART_Transmit( dev->uartHandleGS, (uint8_t*) "TEL\r\n", 5, HAL_MAX_DELAY );
		else if (dev->rx_gs[2] == 1)
			HAL_UART_Transmit( dev->uartHandleGS, (uint8_t*) "CMD\r\n", 5, HAL_MAX_DELAY );
		else if (dev->rx_gs[2] == 2)
			HAL_UART_Transmit( dev->uartHandleGS, (uint8_t*) "DIR\r\n", 5, HAL_MAX_DELAY );
		dev->rx_flag = 0;
	}
	return HAL_OK;
}


HAL_StatusTypeDef HX1_UARTRX( HX1 *dev )
{

	if (dev->rx_gs[1] == 0)
	{
		HAL_UART_Transmit( dev->uartHandleGS, (uint8_t*) "TEL\r\n", 5, HAL_MAX_DELAY );
		HX1_RequestTelemetry( dev, dev->rx_gs[2] );
	}
	else if (dev->rx_gs[1] == 3)
	{
		HAL_UART_Transmit( dev->uartHandleGS, (uint8_t*) "DIR\r\n", 5, HAL_MAX_DELAY );
		HX1_SendDirect( dev, dev->rx_gs );
	}
	else if ( dev->rx_gs[1] == 0x80 )
	{
		HAL_UART_Transmit( dev->uartHandleGS, (uint8_t*) "$\n", 2, HAL_MAX_DELAY );
	}
	HAL_UART_Receive_IT( dev->uartHandleGS, dev->rx_gs, GS_MSG_SIZE);

	return HAL_OK;
}

HAL_StatusTypeDef HX1_RequestTelemetry( HX1 *dev, uint8_t ID)
{
	uint8_t telemetry[2];
	telemetry[0] = 0;
	telemetry[1] = ID;

	return HX1_SendMessage(dev, telemetry, 2);
}

HAL_StatusTypeDef HX1_SendDirect( HX1 *dev, uint8_t* info )
{
	return HX1_SendMessage( dev, &info[1], info[3] );
}

/*
 * LOW LEVEL FUNCTIONS
 */

HAL_StatusTypeDef HX1_SendMessage( HX1 *dev, uint8_t *dataString, uint8_t dataLength )
{
	HAL_GPIO_WritePin( dev->EN_GPIO, dev->EN_Pin, GPIO_PIN_SET );
	startMessage( dev );
	addString( dev, dataString, dataLength);
	endMessage( dev );
	HAL_UART_Transmit(dev->uartHandleRF, dev->message, dev->messageLength, HAL_MAX_DELAY);
	HAL_GPIO_WritePin( dev->EN_GPIO, dev->EN_Pin, GPIO_PIN_RESET );
	return HAL_OK;
}

void startMessage( HX1 *dev )
{
	// Send PREAMBLE
	dev->messageLength = 0;
	for (; dev->messageLength < PREAMBLE_LEN; dev->messageLength++)
	{
		dev->message[dev->messageLength] = FLAG_TYPE;
	}

	// Send SYNC
	for (; dev->messageLength < SYNC_LEN + PREAMBLE_LEN; dev->messageLength++)
	{
		if ((dev->messageLength-PREAMBLE_LEN)%2 == 0)
		{
			dev->message[dev->messageLength] = SYNC1;
		}
		else
		{
			dev->message[dev->messageLength] = SYNC2;
		}
	}

	// Send ID
	addString(dev, (uint8_t *) "SAT", 3);

}

void endMessage( HX1 *dev )
{
	// ADD Checksum

	// Send Postamble
	for (uint8_t i = 0; i < POSTAMBLE_LEN; i++)
	{
		dev->message[dev->messageLength] = FLAG_TYPE;
		dev->messageLength++;
	}
}

void addString( HX1 *dev, uint8_t *string, uint8_t length )
{
	for (uint8_t i = 0; i < length; i++)
	{
		dev->message[dev->messageLength] = string[i];
		dev->messageLength++;
	}
}

/*
 * UNIT TESTS
 */

void HX1_TransmitIncreasing( HX1 *dev )
{
	char msg[17] = "Message Number 0 ";
	char no[10] = "0123456789";
	for (uint8_t i = 0; i<10; i++)
	{
		msg[15] = no[i];
		HX1_SendMessage( dev,(uint8_t*) msg, 17 );
		HAL_Delay(1000);
	}
}

void HX1_ALL_TEL_REQ( HX1 *dev )
{
	for (uint8_t i = 0; i<9; i++)
	{
		HX1_RequestTelemetry( dev, i );
		HAL_Delay(2000);
	}
}
