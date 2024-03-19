/*
 *  EPS General Driver
 *
 *  Author: 	Sam Jeffery
 *  Created: 	27 May 2022
 *  Version: 	1.0
 */


#include "process.h"

/*
 * INITIALISATION
 */


/*
 * HIGH-LEVEL FUNCTIONS
 */

void EPS_PROCESS( CANBUS *canbus, switches *sw, stmADC *volt, ADS7828 *curr )
{
	for (uint8_t i = 0; i < OTHER_NODES; i++)
	{
		if (canbus->RX_received_flag[i] == 1)
		{
			/*
			msg_typ: canbus->RXstack[i][0] & 0x3;
			source: canbus->RXstack[i][1] & 0xF;
			len: canbus->RXstack[i][2];
			data start: canbus->RXstack[i][3]
			 */
			switch(canbus->RXstack[i][0] & 0x3)
			{
			case 0:
				// TELEMETRY CASE

				break;
			case 1:
				// COMMAND CASE
				switch (canbus->RXstack[i][3])
				{
				case TASK_SW_CLEAR:
					SWITCH_SetSpecific(sw, 0xFF, 0);
					SEND_ACKNACK(canbus, canbus->RXstack[i][3], 0, canbus->RXstack[i][1] & 0xF);
					break;
				default:
					SEND_ACKNACK(canbus, canbus->RXstack[i][3], 1, canbus->RXstack[i][1] & 0xF); // IF ID NOT RECOGNISED SEND NACK
					break;
				}
				break;
			case 2:
				// DIRECT CASE
				switch (canbus->RXstack[i][3]) // IN DIR CASE First data is TASK ID
				{
				case TASK_SWITCH:
					SWITCH_SetSpecific(sw, canbus->RXstack[i][4], canbus->RXstack[i][5]);
					SEND_ACKNACK(canbus, canbus->RXstack[i][3], 0, canbus->RXstack[i][1] & 0xF);
					break;
				default:
					SEND_ACKNACK(canbus, canbus->RXstack[i][3], 1, canbus->RXstack[i][1] & 0xF); // IF ID NOT RECOGNISED SEND NACK
					break;
				}
				break;
			}

			// Clear flags
			CAN_CLEAR( canbus, i );
		}
	}
}

void EPS_PROCESS_OLD( CANBUS *canbus, TELEMETRY *tel );

/*
 * LOW-LEVEL FUNCTIONS
 */

void CAN_PROCESS( CANBUS *canbus, TELEMETRY *tel );

/*
 * UNIT TESTS
 */

void EPS_Power_Test( UART_HandleTypeDef *huart, stmADC *volt, ADS7828 *curr )
{
	uint16_t output[7];
	ADS7828_readSelected( curr, 0xDB );
//	ADC_GetALL( volt );

	output[0] = curr->raw_Data[0];
	output[1] = curr->raw_Data[1];
	output[2] = curr->raw_Data[3];
	output[3] = curr->raw_Data[7];
	output[4] = curr->raw_Data[4];
	output[5] = curr->raw_Data[6];

//	output[6] = volt->raw_adc[1];
//	output[7] = volt->raw_adc[2];
//	output[8] = volt->raw_adc[3];

	HAL_UART_Transmit(huart,(uint8_t*) "$", 1, 10);
	HAL_UART_Transmit(huart, (uint8_t*) output, 12, 10);
//	HAL_UART_Transmit(huart, (uint8_t*) output, 18, 10);
	HAL_UART_Transmit(huart, (uint8_t*) "\r\n", 2, 10);

}

void EPS_SYS_CURR_Test( UART_HandleTypeDef *huart, ADS7828 *curr )
{
	uint16_t output[2];
	ADS7828_readSelected( curr, 0xDB );

	output[0] = curr->raw_Data[0];
	output[1] = curr->raw_Data[7];

	HAL_UART_Transmit(huart,(uint8_t*) "$", 1, 10);
	HAL_UART_Transmit(huart, (uint8_t*) output, 4, 10);
	HAL_UART_Transmit(huart, (uint8_t*) "\r\n", 2, 10);

}

void EPS_SYS_CURR_SEND( CANBUS *canbus, ADS7828 *curr )
{
	uint8_t output[6];
	ADS7828_readSelected( curr, 0x81 );

	output[0] = TEL_SYS_CURR;
	output[1] = TEL_SENDING;
	output[2] = curr->raw_Data[0] & 0xFF;
	output[3] = (curr->raw_Data[0]>>8) & 0xFF;
	output[4] = curr->raw_Data[7] & 0xFF;
	output[5] = (curr->raw_Data[7]>>8) & 0xFF;

	//Set Up CAN ID
	CAN_TX_ID TX_ID = CAN_setID( canbus->nodeID, SAT_OBC, MSG_TEL );

	// Add message to CAN
	CAN_Transmit( canbus, TX_ID, output, 6 );


}
