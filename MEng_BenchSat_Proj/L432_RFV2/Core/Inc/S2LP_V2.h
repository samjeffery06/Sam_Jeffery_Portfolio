/*
 *  S2LP UHF Transceiver SPI Driver
 *
 *  Author: 	Sam Jeffery
 *  Created: 	15 March 2022
 *  Version: 	2.0
 */

#ifndef INC_S2LP_V2_H_
#define INC_S2LP_V2_H_

#include "main.h" /* Needed for SPI and GPIO */

/*
 * DEFINES
 */


//#define S2LP_RX_FIFO_SIZE   128
//#define S2LP_TX_FIFO_SIZE   128
#define RCS_S2LP_BUF_SIZE	64

#define XTAL_FREQUENCY     	50000000U

/*
 * AX25 Packet Details
 */

#define PRELEN 				10
#define AX_FLAG				0x7E
#define CRC_POLY			0x8408

/* SPI Message Types */
#define WRITE_HEADER 0x00
#define READ_HEADER 0x01
#define CMD_HEADER 0x80

/* Commands */
#define CMD_TX 				0x60 	// Send the S2LP to TX state for transmission
#define CMD_RX 				0x61 	// Send the S2LP to RX state for reception
#define CMD_READY 			0x62 	// Go to READY state
#define CMD_STANDBY 		0x63 	// Go to STANDBY state
#define CMD_SLEEP 			0x64 	// Go to SLEEP state
#define CMD_LOCKRX 			0x65 	// Go to LOCK state by using the RX configuration of the synthesizer
#define CMD_LOCKTX 			0x66  	// Go to LOCK state by using the TX configuration of the synthesizer
#define CMD_SABORT 			0x67 	// Exit from TX or RX states and go to READY state
#define CMD_LDC_RELOAD 		0x68 	// Reload the LDC timer with a pre-programmed value stored in registers
#define CMD_SRES 			0x70 	// Reset the S2LP state machine and registers values
#define CMD_FLUSHRXFIFO 	0x71 	// Clean the RX FIFO
#define CMD_FLUSHTXFIFO 	0x72 	// Clean the TX FIFO
#define CMD_SEQUENCE_UPDATE 0x73 	// Reload the packet sequence counter with the value stored in register

/*
 * REGISTERS
 */

#define FIFO_ADDRESS 		0xFF

#define GPIO0_CONF 			0x00 	// First GPIO address
#define SYNT3 				0x05 	// First synthesizer address
#define MOD4 				0x0E
#define MOD1				0x11
#define PCKTCTRL6			0x2B
#define PCKTLEN1			0x31
#define PCKTLEN0			0x32
#define PROTOCOL2			0x39
#define PCKT_FLT_OPTIONS	0x40
#define ANT_SELECT_CONF		0x1F
#define IRQ_MASK3			0x50
#define PA_POWER0			0x62
#define IRQ_STATUS3			0xFA
#define IRQ_STATUS0			0xFD
#define PCKT_PSTMBL			0x38

#define TX_FIFO_STATUS		0x8F
#define RX_FIFO_STATUS		0x90

#define DEVICE_INFO0_ADDR	0xF1 	// Version Number
#define DEVICE_INFO1_ADDR	0xF0 	// Part Number


/*
 * TRANSCEIVER STRUCTS
 */

typedef struct
{
	/* SPI handle */
	SPI_HandleTypeDef *spiHandle;

	/* GPIO handles */
	GPIO_TypeDef *CS_GPIO;

	uint16_t CS_Pin;

	GPIO_TypeDef *SDN_GPIO;

	uint16_t SDN_Pin;

	/*
	 * SPI Status
	 * MC_STATE is read 1 and then 0
	 * MC_STATE[0] is MC_STATE1 and MC_STATE[1] is MC_STATE0
	 */

	uint8_t MC_STATE[2];

	uint8_t TX_STATE;

	uint8_t TX_BUFF[RCS_S2LP_BUF_SIZE];

//	uint8_t sendStart;

//	uint8_t MSG_POINTER;

} S2LP;

/*
 * INITIALISATION
 */

uint8_t S2LP_Initialise( S2LP *dev,
					SPI_HandleTypeDef *spiHandle,
					GPIO_TypeDef *CS_GPIO,
					uint16_t CS_Pin,
					GPIO_TypeDef *SDN_GPIO,
					uint16_t SDN_Pin );

/*
 * HIGH-LEVEL FUNCTIONS
 */

uint8_t S2LP_TransmitMessage( S2LP *dev, uint8_t msg_len, uint8_t *msg );

uint8_t S2LP_TransmitAX25(S2LP *dev, uint8_t *msg, uint8_t msg_len);

uint8_t S2LP_AX25Packet(S2LP *dev, uint8_t *msg, uint8_t msg_len);

uint8_t S2LP_SetMsgLen( S2LP *dev, uint8_t msg_len );

uint8_t S2LP_SetBaudRate( S2LP *dev, uint8_t option);

uint8_t S2LP_SetFDev( S2LP *dev, uint8_t option);

uint8_t S2LP_SetFreq( S2LP *dev, uint8_t option);

/*
 * UNIT TESTS
 */

uint32_t S2LP_MAXTransmitSpeed( S2LP *dev, uint32_t no );

uint8_t S2LP_TransmitIncreasing( S2LP *dev, uint8_t len );

uint8_t S2LP_TI_2BR( S2LP *dev );

void S2LP_TransmitTest( S2LP *dev );

/*
 * LOW-LEVEL FUNCTIONS
 */

void S2LP_update_crc( uint16_t *crc, uint8_t bit);

HAL_StatusTypeDef S2LP_Command( S2LP *dev, uint8_t command);
HAL_StatusTypeDef S2LP_Read( S2LP *dev, uint8_t address, uint8_t numBytes, uint8_t* recBuffer);
HAL_StatusTypeDef S2LP_Write( S2LP *dev, uint8_t address, uint8_t numBytes, uint8_t* sendBuffer);


#endif /* INC_S2LP_V2_H_ */
