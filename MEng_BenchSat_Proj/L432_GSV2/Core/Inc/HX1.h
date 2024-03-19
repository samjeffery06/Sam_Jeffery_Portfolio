/*
 *  HX1 Narrowband Transmitter Driver
 *
 *  Author: Sam Jeffery
 *  Created: 14 March 2022
 */



#ifndef INC_HX1_H_
#define INC_HX1_H_

#include "stm32l4xx_hal.h" /* Needed for UART -- Possibly GPIOs*/

/*
 * DEFINES
 */

#define MAX_MSG_SIZE 	128

#define GS_MSG_SIZE		32

#define SYNC_LEN 		9  		// Must be odd
#define PREAMBLE_LEN	24
#define POSTAMBLE_LEN	5

#define FLAG_TYPE		0x7E	// ~
#define SYNC1			0x2A	// *
#define SYNC2			0x55	// U

/*
 * Message Struct
 */

typedef struct
{
	/* UART handle for RF Link*/
	UART_HandleTypeDef *uartHandleRF;

	/* UART handle for GS Comm link*/
	UART_HandleTypeDef *uartHandleGS;

	uint16_t EN_Pin;

	GPIO_TypeDef *EN_GPIO;

	uint8_t messageLength;

	uint8_t message[MAX_MSG_SIZE];

	uint8_t rx_gs[GS_MSG_SIZE];

	uint8_t rx_flag;

} HX1;

/*
 * INITIALISATION
 */

void HX1_Initialise( HX1 *dev, UART_HandleTypeDef *uartHandleRF, UART_HandleTypeDef *uartHandleGS, GPIO_TypeDef *EN_GPIO, uint16_t EN_Pin );

/*
 * HIGH LEVEL FUNCTIONS
 */
void HX1_UARTGS( HX1 *dev);

HAL_StatusTypeDef HX1_PROCESS( HX1 *dev );

HAL_StatusTypeDef HX1_UARTRX( HX1 *dev );

HAL_StatusTypeDef HX1_RequestTelemetry( HX1 *dev, uint8_t ID);

HAL_StatusTypeDef HX1_SendDirect( HX1 *dev, uint8_t* info );

/*
 * LOW LEVEL FUNCTIONS
 */

HAL_StatusTypeDef HX1_SendMessage( HX1 *dev, uint8_t *dataString, uint8_t dataLength );
void startMessage( HX1 *dev );
void endMessage( HX1 *dev );
void addString( HX1 *dev, uint8_t *string, uint8_t length );

/*
 * UNIT TEST
 */

void HX1_TransmitIncreasing( HX1 *dev );

#endif /* INC_HX1_H_ */
