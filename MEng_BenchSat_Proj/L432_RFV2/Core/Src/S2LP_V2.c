/*
 *  S2LP UHF Tranciever SPI Driver
 *
 *  Author: 	Sam Jeffery
 *  Created: 	15 March 2022
 *  Version 	2.0
 */


#include "S2LP_V2.h"

/*
 * INITIALISATION
 */

uint8_t S2LP_Initialise( S2LP *dev, SPI_HandleTypeDef *spiHandle, GPIO_TypeDef *CS_GPIO, uint16_t CS_Pin, GPIO_TypeDef *SDN_GPIO, uint16_t SDN_Pin )
{
	/* Setup SPI handle to communicate with S2LP */
	dev->spiHandle	= spiHandle;

	/* Setup Chip Select GPIO */
	dev->CS_GPIO 	= CS_GPIO;

	dev->CS_Pin		= CS_Pin;

	/* Setup Shutdown GPIO */
	dev->SDN_GPIO 	= CS_GPIO;

	dev->SDN_Pin	= CS_Pin;

	/* Write CS pin high -- SPI not selected */
	HAL_GPIO_WritePin(CS_GPIO, CS_Pin, GPIO_PIN_SET);

	/* Toggle SDN to reset S2LP */
	HAL_GPIO_WritePin(SDN_GPIO, SDN_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(SDN_GPIO, SDN_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);

	/* Create buffer for SPI messages */
	uint8_t init_buf[8];

	/* Check Connections are correct and device is communicating */

	S2LP_Read( dev, DEVICE_INFO1_ADDR, 2, init_buf );

	if ((init_buf[0] != 0x03) | (init_buf[1] != 0x91))
	{
		return 255;
	}

	/* Default setup for S2LP registers listed below */

	/*
	 * Select Frequency
	 * Currently 437MHz
	 * See Communications Spreadsheet
	 */

//	436.975MHz
	init_buf[0] = 0x52; 	// SYNT3 [7:5] PLL_CP_ISEL, 4 BS, [3:0] Setting PLL programmable divider
	init_buf[1] = 0x2F; 	// SYNT2 Setting PLL programmable divider
	init_buf[2] = 0x53; 	// SYNT1 Setting PLL programmable divider
	init_buf[3] = 0xF8; 	// SYNT0 Setting PLL programmable divider

//	437MHz
//	init_buf[0] = 0x52; 	// SYNT3 [7:5] PLL_CP_ISEL, 4 BS, [3:0] Setting PLL programmable divider
//	init_buf[1] = 0x2F; 	// SYNT2 Setting PLL programmable divider
//	init_buf[2] = 0x5C; 	// SYNT1 Setting PLL programmable divider
//	init_buf[3] = 0x29; 	// SYNT0 Setting PLL programmable divider

	S2LP_Write( dev, SYNT3, 4, init_buf);

	/*
	 * Select datarate, modulation and frequency deviation
	 * Currently 9600bps, 2FSK, 3KHz
	 */
	init_buf[0] = 0x92; 	// MOD4 MSB of DATARATE_M
	init_buf[1] = 0xA7; 	// MOD3 LSB of DATARATE_M
	init_buf[2] = 0x05; 	// MOD2 [7:4] MOD_TYPE = 0x0, [3:0] DATARATE_E = [CHANGE] -- THIS VALUE CHANGES THE DATA RATE WELL
	init_buf[3] = 0x00; 	// MOD1 [7:4] unrelated, [3:0] FDEV_E
	init_buf[4] = 0xFB; 	// MOD0 FDEV_M

//	FOR 300bps
//	init_buf[0] = 0xC9; 	// MOD4 MSB of DATARATE_M
//	init_buf[1] = 0x54; 	// MOD3 LSB of DATARATE_M
//	init_buf[2] = 0x00; 	// MOD2 [7:4] MOD_TYPE = 0x0, [3:0] DATARATE_E = 0x2
//	init_buf[3] = 0x00; 	// MOD1 [7:4] unrelated, [3:0] FDEV_E
//	init_buf[4] = 0xFB; 	// MOD0 FDEV_M

	S2LP_Write( dev, MOD4, 5, init_buf );

	/*
	 * Select packet characteristics
	 */

	init_buf[0] = 0x80; 	// PCKTCTRL6 0x80 initial value
	init_buf[1] = 0xC0; 	// PCKTCTRL5 0x10 initial value
	init_buf[2] = 0x00; 	// PCKTCTRL4
	init_buf[3] = 0x04; 	// PCKTCTRL3 Select Normal mode RX Mode
	init_buf[4] = 0x00; 	// PCKTCTRL2
//	init_buf[5] = 0x40; 	// PCKTCTRL1 Normal TX Mode, ENABLE WHITENING MODE
	init_buf[5] = 0x00;		// PCKTCTRL1 Normal TX Mode, No WHITENING, NO CRC
//	init_buf[5] = 0x10; 	// PCKTCTRL1 Normal TX Mode, ENABLE WHITENING MODE, NO CRC
	init_buf[6] = 0x00; 	// PCKTLEN1
	init_buf[7] = 0x01; 	// PCKTLEN0 SET DEFAULT

	S2LP_Write( dev, PCKTCTRL6, 8, init_buf );

	init_buf[0] = 0x04; 	// Set Postamble length

	S2LP_Write( dev, PCKT_PSTMBL, 1, init_buf );

	/*
	 * Select RX State
	 */

	init_buf[0] = 0x00; 	// Protocol2

	S2LP_Write( dev, PROTOCOL2, 1, init_buf);

//	init_buf[0] = 0x01; 	// PCKT Options
	init_buf[0] = 0x00; 	// PCKT Options IGNORE CRC ON RX

	S2LP_Write( dev, PCKT_FLT_OPTIONS, 1, init_buf);

	init_buf[0] = 0x55; 	// ANT_SELECT_CONF

	S2LP_Write( dev, ANT_SELECT_CONF, 1, init_buf);

	/*
	 * Select interrupt options on S2LP GPIO pins
	 */

	init_buf[0] = 0x02; 	// GPIO0_CONF
	init_buf[1] = 0xA2; 	// GPIO1_CONF
	init_buf[2] = 0xA2; 	// GPIO2_CONF
	init_buf[3] = 0xA2; 	// GPIO3_CONF

	S2LP_Write( dev, GPIO0_CONF, 4, init_buf );

	init_buf[0] = 0x00; 	// IRQ_MASK3
	init_buf[1] = 0x00; 	// IRQ_MASK2
	init_buf[2] = 0x00; 	// IRQ_MASK1
//	init_buf[3] = 0x05; 	// IRQ_MASK0 RX DATA READY & TX DATA SENT
	init_buf[3] = 0x04; 	// IRQ_MASK0 TX DATA SENT

	S2LP_Write( dev, IRQ_MASK3, 4, init_buf );

	return 0;
}

/*
 * HIGH-LEVEL FUNCTIONS
 */

uint8_t S2LP_TransmitMessage( S2LP *dev, uint8_t msg_len, uint8_t *msg )
{
	/*
	 * Transmit message function, care must be taken to leave enough time
	 * for the S2LP to send the message before a new message is sent
	 */


	msg_len = S2LP_AX25Packet(dev, msg, msg_len);

	while (dev->TX_STATE)
		HAL_Delay(5);

	dev->TX_STATE = 1;

	S2LP_Command( dev, CMD_SABORT );

	S2LP_Command(dev, CMD_FLUSHTXFIFO);

	S2LP_SetMsgLen(dev, msg_len);

	S2LP_Write( dev, FIFO_ADDRESS, msg_len, dev->TX_BUFF );

	S2LP_Command( dev, CMD_LOCKTX );

	HAL_Delay(5);

	S2LP_Command( dev, CMD_TX );

	return 0;

}

uint8_t S2LP_AX25Packet(S2LP *dev, uint8_t *msg, uint8_t msg_len)
{
	uint8_t bit;
	uint8_t ones_row = 0;
	uint8_t zeros_row = 0;
	uint16_t crc = 0;

	dev->TX_BUFF[0] = AX_FLAG;
	uint16_t packetsize = 8;

	for (uint8_t i = 0; i < msg_len+2; i++)
	{
		for (uint8_t j = 0; j < 8; j++)
		{
			if (i < msg_len)
			{
				bit = (msg[i] >> j) & 0x1;
				S2LP_update_crc(&crc, bit);
			}
			else
			{
				bit = (crc >> (j + 8*(i-msg_len))) & 0x1;
			}

			if (bit)
			{
				ones_row++;
				zeros_row = 0;
			}
			else
			{
				zeros_row++;
				ones_row = 0;
			}
			dev->TX_BUFF[packetsize >> 3] &= ~(1 << (packetsize & 7));
			dev->TX_BUFF[packetsize >> 3] |= bit << (packetsize & 7);
			packetsize++;
			if (ones_row == 5)
			{
				dev->TX_BUFF[packetsize >> 3] &= ~(1 << (packetsize & 7));
				ones_row = 0;
				zeros_row = 1;
				packetsize++;
			}
			else if(zeros_row == 5)
			{
				dev->TX_BUFF[packetsize >> 3] |= 1 << (packetsize & 7);
				zeros_row = 0;
				ones_row = 1;
				packetsize++;
			}
		}
	}

	for (uint8_t i = 0; i < 8; i++)
	{
		bit = (AX_FLAG >> i) & 0x1;
		dev->TX_BUFF[packetsize >> 3] &= ~(1 << (packetsize & 7));
		dev->TX_BUFF[packetsize >> 3] |= bit << (packetsize & 7);
		packetsize++;
	}

	while (packetsize%8)
	{
		dev->TX_BUFF[packetsize >> 3] &= ~(1 << (packetsize & 7));
		packetsize++;
	}

	return packetsize/8;

}

uint8_t S2LP_SetMsgLen( S2LP *dev, uint8_t msg_len )
{

	S2LP_Write( dev, PCKTLEN0, 1, &msg_len );

	return 0;
}

uint8_t S2LP_SetBaudRate( S2LP *dev, uint8_t option)
{
	uint8_t init_buf[3];
	init_buf[0] = 0x92; 	// MOD4 MSB of DATARATE_M
	init_buf[1] = 0xA7; 	// MOD3 LSB of DATARATE_M

	if (option == 0) 		// Set BR = 300BPS
	{
		init_buf[0] = 0xC9; 	// MOD4 MSB of DATARATE_M
		init_buf[1] = 0x54; 	// MOD3 LSB of DATARATE_M
		init_buf[2] = 0x00; 	// MOD2 [7:4] MOD_TYPE = 0x0, [3:0] DATARATE_E = 0x0
	}
	else if (option == 1)		// Set BR = 600BPS
		init_buf[2] = 0x01;		// MOD2 [7:4] MOD_TYPE = 0x0, [3:0] DATARATE_E = 0x1
	else if (option == 2) 		// Set BR = 1200BPS
		init_buf[2] = 0x02; 	// MOD2 [7:4] MOD_TYPE = 0x0, [3:0] DATARATE_E = 0x2
	else if (option == 3) 		// Set BR = 2400BPS
		init_buf[2] = 0x03; 	// MOD2 [7:4] MOD_TYPE = 0x0, [3:0] DATARATE_E = 0x3
	else if (option == 4)		// Set BR = 4800 BPS
		init_buf[2] = 0x04; 	// MOD2 [7:4] MOD_TYPE = 0x0, [3:0] DATARATE_E = 0x4
	else if (option == 5)		// Set BR = 9600 BPS
		init_buf[2] = 0x05; 	// MOD2 [7:4] MOD_TYPE = 0x0, [3:0] DATARATE_E = 0x5
	else if (option == 6)		// Set BR = 19200 BPS
		init_buf[2] = 0x06; 	// MOD2 [7:4] MOD_TYPE = 0x0, [3:0] DATARATE_E = 0x6
	else if (option == 7)		// Set BR = 38400 BPS
		init_buf[2] = 0x07; 	// MOD2 [7:4] MOD_TYPE = 0x0, [3:0] DATARATE_E = 0x7
	else if (option == 8)		// Set BR = 76800 BPS
		init_buf[2] = 0x08; 	// MOD2 [7:4] MOD_TYPE = 0x0, [3:0] DATARATE_E = 0x7
	else if (option == 9)		// Set BR = 7200 BPS
	{
		init_buf[0] = 0x2D; 	// MOD4 MSB of DATARATE_M
		init_buf[1] = 0xFD; 	// MOD3 LSB of DATARATE_M
		init_buf[2] = 0x05; 	// MOD2 [7:4] MOD_TYPE = 0x0, [3:0] DATARATE_E = 0x0
	}
	else if (option == 10)		// Set BR = 8400 BPS
	{
		init_buf[0] = 0x60; 	// MOD4 MSB of DATARATE_M
		init_buf[1] = 0x52; 	// MOD3 LSB of DATARATE_M
		init_buf[2] = 0x05; 	// MOD2 [7:4] MOD_TYPE = 0x0, [3:0] DATARATE_E = 0x0
	}
	else if (option == 11)		// Set BR = 7600 BPS
	{
		init_buf[0] = 0x3E; 	// MOD4 MSB of DATARATE_M
		init_buf[1] = 0xC4; 	// MOD3 LSB of DATARATE_M
		init_buf[2] = 0x05; 	// MOD2 [7:4] MOD_TYPE = 0x0, [3:0] DATARATE_E = 0x0
	}
	else if (option == 12)		// Set BR = 6400 BPS
		{
			init_buf[0] = 0xC; 	// MOD4 MSB of DATARATE_M
			init_buf[1] = 0x6F; 	// MOD3 LSB of DATARATE_M
			init_buf[2] = 0x05; 	// MOD2 [7:4] MOD_TYPE = 0x0, [3:0] DATARATE_E = 0x0
		}
	else
		return 0xFF;

	S2LP_Write( dev, MOD4, 3, init_buf );

	return option;

}

uint8_t S2LP_SetFDev( S2LP *dev, uint8_t option)
{
	uint8_t init_buf[2];

	if (option == 1) 	// Set FDev = 5KHz
	{
		init_buf[0] = 0x01; 	// MOD1 [7:4] unrelated, [3:0] FDEV_E
		init_buf[1] = 0xA3; 	// MOD0 FDEV_M
	}
	else if (option == 2) 	// Set FDev = 20KHz
	{
		init_buf[0] = 0x03; 	// MOD1 [7:4] unrelated, [3:0] FDEV_E
		init_buf[1] = 0xA3; 	// MOD0 FDEV_M
	}
	else 			// Set FDev = 3KHz
	{
		init_buf[0] = 0x00; 	// MOD1 [7:4] unrelated, [3:0] FDEV_E
		init_buf[1] = 0xFB; 	// MOD0 FDEV_M
	}

	S2LP_Write( dev, MOD1, 2, init_buf );

	return option;
}

uint8_t S2LP_SetFreq( S2LP *dev, uint8_t option)
{
	uint8_t init_buf[4];

	init_buf[0] = 0x52; 	// SYNT3 [7:5] PLL_CP_ISEL, 4 BS, [3:0] Setting PLL programmable divider
	init_buf[1] = 0x2F; 	// SYNT2 Setting PLL programmable divider
	init_buf[2] = 0x5C; 	// SYNT1 Setting PLL programmable divider
	init_buf[3] = 0x29; 	// SYNT0 Setting PLL programmable divider

	S2LP_Write( dev, SYNT3, 4, init_buf);

	return option;
}

/*
 * UNIT TESTS
 */

uint32_t S2LP_MAXTransmitSpeed( S2LP *dev, uint32_t no )
{
	if (dev->TX_STATE == 0 && no < 20 )
//	if (dev->TX_STATE == 0 && dev->sendStart)
//	if (dev->TX_STATE == 0)
	{
		uint8_t test_msg_len = 15;
		uint8_t message[test_msg_len];
		for (uint8_t i = 0; i < test_msg_len; i++)
			message[i] = i;
		message[0] = no && 0xFF;

		S2LP_TransmitMessage(dev, test_msg_len, message);
		no++;
	}
	return no;

}

// Get STATUS

// Transmit Increasing Values
uint8_t S2LP_TransmitIncreasing( S2LP *dev, uint8_t len )
{
	/*
	 * Unit test for S2LP transmit function
	 * Transmits messages with increasing IDs
	 */

	uint8_t unit_test[] = "Unit Test ID: 0";
	for (uint8_t i = 0; i < len; i++)
	{
		unit_test[14] = i;
		S2LP_TransmitMessage( dev, 15, unit_test);
		HAL_Delay(3000);
	}
	return 0;
}

uint8_t S2LP_TI_2BR( S2LP *dev )
{
	S2LP_TransmitIncreasing( dev, 5 );
	S2LP_SetBaudRate( dev, 1 );
	HAL_Delay(100);
	S2LP_TransmitIncreasing( dev, 5 );
	S2LP_SetBaudRate( dev, 0 );

	return 0;
}

void S2LP_TransmitTest( S2LP *dev )
{
	uint8_t unit_test[50];
	for (uint8_t i = 0; i < 50; i++)
		unit_test[i] = i;

	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);
	for (uint8_t i = 4; i < 7; i++)
	{
		S2LP_SetBaudRate(dev, i);
		HAL_Delay(1000);
		for (uint8_t j = 0; j < 5; j++)
		{
			unit_test[0] = j;
			S2LP_TransmitMessage( dev, 50, unit_test);
			HAL_Delay(3000);
		}
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		HAL_Delay(1000);
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
	}
}


/*
 * LOW-LEVEL FUNCTIONS
 */

void S2LP_update_crc( uint16_t *crc, uint8_t bit)
{
	*crc ^= bit;
	if (*crc & 1)
		*crc = (*crc >> 1) ^ CRC_POLY;
	else
		*crc = *crc >> 1;
}

HAL_StatusTypeDef S2LP_Command( S2LP *dev, uint8_t command)
{
	uint8_t header[2] = {CMD_HEADER, command};

	HAL_GPIO_WritePin( dev->CS_GPIO, dev->CS_Pin, GPIO_PIN_RESET );
	HAL_StatusTypeDef status = HAL_SPI_TransmitReceive( dev->spiHandle, (uint8_t *) header, (uint8_t *) dev->MC_STATE, 2, HAL_MAX_DELAY );
	HAL_GPIO_WritePin( dev->CS_GPIO, dev->CS_Pin, GPIO_PIN_SET );

	return status;
}

HAL_StatusTypeDef S2LP_Read( S2LP *dev, uint8_t address, uint8_t numBytes, uint8_t* recBuffer)
{
	uint8_t header[2] = {READ_HEADER, address};

	HAL_GPIO_WritePin( dev->CS_GPIO, dev->CS_Pin, GPIO_PIN_RESET );
	HAL_SPI_TransmitReceive( dev->spiHandle, header, dev->MC_STATE, 2, HAL_MAX_DELAY );
	HAL_StatusTypeDef status = HAL_SPI_Receive( dev->spiHandle, recBuffer , numBytes, HAL_MAX_DELAY );
	HAL_GPIO_WritePin( dev->CS_GPIO, dev->CS_Pin, GPIO_PIN_SET );

	return status;
}

HAL_StatusTypeDef S2LP_Write( S2LP *dev, uint8_t address, uint8_t numBytes, uint8_t* sendBuffer)
{
	uint8_t header[2] = {WRITE_HEADER, address};

	HAL_GPIO_WritePin( dev->CS_GPIO, dev->CS_Pin, GPIO_PIN_RESET );
	HAL_SPI_TransmitReceive( dev->spiHandle, header, dev->MC_STATE, 2, HAL_MAX_DELAY );
	HAL_StatusTypeDef status = HAL_SPI_Transmit( dev->spiHandle, sendBuffer , numBytes, HAL_MAX_DELAY );
	HAL_GPIO_WritePin( dev->CS_GPIO, dev->CS_Pin, GPIO_PIN_SET );

	return status;
}

