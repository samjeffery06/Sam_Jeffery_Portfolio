/*
 *  RF2 General Driver
 *
 *  Author: 	Sam Jeffery
 *  Created: 	11 April 2022
 *  Version: 	1.0
 */

#include "process.h"

/*
 * INITIALISATION
 */


/*
 * HIGH-LEVEL FUNCTIONS
 */
//void RF_PROCESS( S2LP *s2lp, NRX1_TIM *nrx1, CANBUS *canbus, TELEMETRY *tel, UART_HandleTypeDef *huart )
void RF_PROCESS( S2LP *s2lp, NRX1_TIM *nrx1, CANBUS *canbus, TELEMETRY *tel )
{
	/*
	 * RECEIVE MESSAGE STEP
	 * Read in message on VHF channel and handle accordingly
	 */
	// Assuming that we always process the last message before the next one

	CAN_TX_ID TX_ID;

	if ( nrx1->flag_end )
	{
		switch (nrx1->buffer[3])
		{
		case 0:
			/*
			 * TEL request
			 * NRX buffer starts SAT then message
			 * buffer[3]: MSG type (in this case: TEL -> 0)
			 * buffer[4]: NODE TO REQUEST
			 * buffer[5]: TEL_ID
			 * buffer[6]: RESPONSE /SET = 1 /REQUEST = 0 /INTERNAL UPDATE 2
			 */
			if (nrx1->buffer[6] == 0) // GS ALWAYS SENDS REQUEST
			{
				if (nrx1->buffer[4] == SAT_RCS) // BASIC RF TELEMETRY REQUEST
				{
					RF_TRANSMIT_TEL( s2lp, tel, nrx1->buffer[5] );
				}
				else // REQUESTING DATA FROM SPECIFIC NODE
				{
					TX_ID = CAN_setID( SAT_RCS, nrx1->buffer[4], MSG_TEL );
					CAN_Transmit( canbus, TX_ID, (uint8_t*) &nrx1->buffer[5], 2 ); // SENDS [ID][REQ]
				}
			}
			break;
		case 1:
			/*
			 * TELECOMMAND
			 * NRX buffer starts SAT then message
			 * buffer[3]: MSG type (in this case: CMD -> 1)
			 * buffer[4]: NODE TO CMD
			 * buffer[5]: CMD_ID
			 */
			if ( nrx1->buffer[4] == SAT_RCS )
			{
				if (nrx1->buffer[5] == TASK_RFUNIT)
				{
					for (uint8_t i = 0; i < 10; i++)
					{
						HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
						HAL_Delay(100);
					}
					RF_TRANSMIT_ACK( s2lp, SAT_RCS, TASK_RFUNIT, 0 );
				}
				else if( nrx1->buffer[5] == 23 )
				{
//					s2lp->sendStart = 1;
//					nrx1->count++;
					RF_TRANSMIT_ACK( s2lp, SAT_RCS, 23, 0 );
					for (uint8_t i = 0; i < 4; i++)
					{
						HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
						HAL_Delay(100);
					}
					RF_TRANSMIT_ACK( s2lp, SAT_RCS, 23, 0 );
//					if (nrx1->count[0] == 20)
//					{
//						nrx1->count[0]++;
////						s2lp->sendStart = 0;
//					}
				}
			}
			else
			{
				TX_ID = CAN_setID( SAT_RCS, nrx1->buffer[4], MSG_CMD );
				CAN_Transmit( canbus, TX_ID, (uint8_t*) &nrx1->buffer[5], 1 );
			}
			break;
		case 2:
			/*
			 * Direct Message Command
			 * NRX buffer starts SAT then message
			 * buffer[3]: msg type (in this case: DIR -> 2)
			 * buffer[4]: DESTINATION NODE ID
			 * buffer[5]: MESSAGE LENGTH
			 */

			//Set Up CAN ID
			TX_ID = CAN_setID( SAT_RCS, nrx1->buffer[4], MSG_DIR );

			// Add message to CAN
			CAN_Transmit( canbus, TX_ID, (uint8_t*) &nrx1->buffer[6], nrx1->buffer[5] );
			break;
		}
		nrx1->flag_end = 0;
	}

	/*
	 * TRANSMIT BEACON STEP
	 * Regularly transmit beacon
	 */
}

void CAN_PROCESS( S2LP *s2lp, CANBUS *canbus, TELEMETRY *tel )
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
					// UPDATE
					switch (canbus->RXstack[i][4])
					{
						case 0:
							//REQUEST
							// Unlikely to be a request for data
							break;
						case 1:
							// RESPONSE to REQUEST
							/*
							 * Transmit message direct to ground
							 * OTHER SYSTEMS CAN USE THIS TO DIRECTLY TRANSMIT TELEMETRY TO THE GROUND
							 * EVEN IF NOT REQUESTED
							 */
							S2LP_TransmitMessage(s2lp, canbus->RXstack[i][2] + 3, &canbus->RXstack[i][0]);
						case 2:
							// UPDATE MESSAGE
							/*
							 * Update TEL Storage
							 */
						break;
					}
				case 1:
					// COMMAND CASE

					break;
				case 2:
					// DIRECT CASE
					S2LP_TransmitMessage(s2lp, canbus->RXstack[i][2] + 3, &canbus->RXstack[i][0]);
					break;
			}

			// Clear flags
			CAN_CLEAR( canbus, i );
		}
	}
}


/*
 * LOW-LEVEL FUNCTIONS
 */

void RF_TRANSMIT_ACK( S2LP *s2lp, uint8_t source, uint8_t taskID, uint8_t acknack )
{
	uint8_t ack_buf[7];
	ack_buf[0] = MSG_DIR;
	ack_buf[1] = source;
	ack_buf[2] = 3;
	ack_buf[3] = TASK_ACK;
	ack_buf[4] = taskID;
	ack_buf[5] = acknack;
	S2LP_TransmitMessage( s2lp, 6, ack_buf );
}

void RF_TRANSMIT_TEL( S2LP *s2lp, TELEMETRY *tel, uint8_t tel_no )
{
	uint8_t TEL_MSG[27] = "TEL";
	uint8_t TEL_LEN;
	switch( tel_no )
	{
		case 0:
			/*
			 * ACC DATA UINT16_t[3]
			 * TEL:(uint16_t):(uint16_t):(uint16_t) = 3 + 1 + 2 + 1 + 2 + 1 + 2 = 12
			 */
			TEL_LEN = 12;
			for ( uint8_t i = 0; i < 3; i++ )
			{
				TEL_MSG[3*i+4] = (tel->OBC_acc[i] >> 8) & 0x00FF;
				TEL_MSG[3*i+5] = (tel->OBC_acc[i]) & 0x00FF;
				TEL_MSG[3*i+3] = 58; // Set : between values
			}
//			S2LP_SetMsgLen( s2lp, TEL_LEN );
			S2LP_TransmitMessage( s2lp, TEL_LEN, TEL_MSG );
			break;

		case 1:
			/*
			 * GYRO DATA UINT16_t[3]
			 * TEL:(uint16_t):(uint16_t):(uint16_t) = 3 + 1 + 2 + 1 + 2 + 1 + 2 = 12
			 */
			TEL_LEN = 12;
			for ( uint8_t i = 0; i < 3; i++ )
			{
				TEL_MSG[3*i+4] = (tel->OBC_gyro[i] >> 8) & 0x00FF;
				TEL_MSG[3*i+5] = (tel->OBC_gyro[i]) & 0x00FF;
				TEL_MSG[3*i+3] = 58; // Set : between values
			}
//			S2LP_SetMsgLen( s2lp, TEL_LEN );
			S2LP_TransmitMessage( s2lp, TEL_LEN, TEL_MSG );
			break;

		case 2:
			/*
			 * COMP DATA UINT16_t -- Value in degrees
			 * TEL:(uint16_t) = 3 + 1 + 2 = 6
			 */
			TEL_LEN = 6;
			TEL_MSG[5] = (tel->OBC_comp >> 8) & 0x00FF;
			TEL_MSG[6] = (tel->OBC_comp) & 0x00FF;
			TEL_MSG[4] = 58; // Set : between values

//			S2LP_SetMsgLen( s2lp, TEL_LEN );
			S2LP_TransmitMessage( s2lp, TEL_LEN, TEL_MSG );
			break;

		case 3:
			/*
			 * TEMPERATURE DATA UINT16_t
			 * TEL:(uint16_t) = 3 + 1 + 2 = 6
			 */
			TEL_LEN = 6;
			TEL_MSG[4] = (tel->OBC_temp >> 8) & 0x00FF;
			TEL_MSG[5] = (tel->OBC_temp) & 0x00FF;
			TEL_MSG[3] = 58; // Set : between values

//			S2LP_SetMsgLen( s2lp, TEL_LEN );
			S2LP_TransmitMessage( s2lp, TEL_LEN, TEL_MSG );
			break;

		case 4:
			/*
			 * POSITION AND POSE DATA UINT16_t[3]
			 * X, Y, ANGLE
			 * TEL:(uint16_t):(uint16_t):(uint16_t) = 3 + 1 + 2 + 1 + 2 + 1 + 2 = 12
			 */
			TEL_LEN = 12;
			for ( uint8_t i = 0; i < 3; i++ )
			{
				TEL_MSG[3*i+4] = (tel->ADCS_position_pose[i] >> 8) & 0x00FF;
				TEL_MSG[3*i+5] = (tel->ADCS_position_pose[i]) & 0x00FF;
				TEL_MSG[3*i+3] = 58; // Set : between values
			}
//			S2LP_SetMsgLen( s2lp, TEL_LEN );
			S2LP_TransmitMessage( s2lp, TEL_LEN, TEL_MSG );
			break;

		case 5:
			/*
			 * SWITCH DATA UINT8_t
			 * TEL:(uint8_t) = 3 + 1 + 1
			 */
			TEL_LEN = 5;
			TEL_MSG[4] = tel->EPS_switch;
			TEL_MSG[3] = 58; // Set : between values
//			S2LP_SetMsgLen( s2lp, TEL_LEN );
			S2LP_TransmitMessage( s2lp, TEL_LEN, TEL_MSG );
			break;

		case 6:
			/*
			 * CURRENT MEASUREMENTS DATA UINT16_t[3]
			 * TEL:(uint16_t):(uint16_t):(uint16_t):(uint16_t):(uint16_t):(uint16_t):(uint16_t):(uint16_t) = 3 + 1 + 2 + 1 + 2 + 1 + 2 + 1 + 2 + 1 + 2 + 1 + 2 + 1 + 2 + 1 + 2 = 27
			 */
			TEL_LEN = 27;
			for ( uint8_t i = 0; i < 8; i++ )
			{
				TEL_MSG[3*i+4] = (tel->EPS_current[i] >> 8) & 0x00FF;
				TEL_MSG[3*i+5] = (tel->EPS_current[i]) & 0x00FF;
				TEL_MSG[3*i+3] = 58; // Set : between values
			}
//			S2LP_SetMsgLen( s2lp, TEL_LEN );
			S2LP_TransmitMessage( s2lp, TEL_LEN, TEL_MSG );
			break;
		case 7:
			/*
			 * VOLTAGE MEASUREMENTS DATA UINT16_t[3]
			 * TEL:(uint16_t):(uint16_t):(uint16_t) = 3 + 1 + 2 + 1 + 2 + 1 + 2 = 12
			 */
			TEL_LEN = 12;
			for ( uint8_t i = 0; i < 3; i++ )
			{
				TEL_MSG[3*i+4] = (tel->EPS_voltage[i] >> 8) & 0x00FF;
				TEL_MSG[3*i+5] = (tel->EPS_voltage[i]) & 0x00FF;
				TEL_MSG[3*i+3] = 58; // Set : between values
			}
//			S2LP_SetMsgLen( s2lp, TEL_LEN );
			S2LP_TransmitMessage( s2lp, TEL_LEN, TEL_MSG );
			break;

		case 8:
			/*
			 * RSSI DATA UINT16_t
			 * TEL:(uint16_t) = 3 + 1 + 2 = 6
			 */
			TEL_LEN = 6;
			TEL_MSG[4] = (tel->RF_rssi >> 8) & 0x00FF;
			TEL_MSG[5] = (tel->RF_rssi) & 0x00FF;
			TEL_MSG[3] = 58; // Set : between values

//			S2LP_SetMsgLen( s2lp, TEL_LEN );
			S2LP_TransmitMessage( s2lp, TEL_LEN, TEL_MSG );
			break;

		default:
			RF_TRANSMIT_ACK(s2lp, SAT_RCS, TASK_TEL, 1);
			break;
	}
}

/*
 * UNIT TESTS
 */

void PROCESS_TEL_UNIT( TELEMETRY *tel )
{
	tel->OBC_acc[0] = 1;
	tel->OBC_acc[1] = 2;
	tel->OBC_acc[2] = 3;

	tel->OBC_gyro[0] = 0x4567;
	tel->OBC_gyro[1] = 0x8910;
	tel->OBC_gyro[2] = 0x1112; // pitch, roll, yaw

	tel->OBC_comp = 0x1314;

	tel->OBC_sol = 0x15;

	tel->OBC_temp = 0x1617;

	tel->ADCS_position_pose[0] = 0x1819;
	tel->ADCS_position_pose[1] = 0x2021;
	tel->ADCS_position_pose[2] = 0x2223; // X, Y, ANGLE

	tel->EPS_switch = 0x24;

	tel->EPS_current[0] = 0x4355;
	tel->EPS_current[1] = 0x5252;
	tel->EPS_current[2] = 0x454E;
	tel->EPS_current[3] = 0x5400;
	tel->EPS_current[4] = 0x4355;
	tel->EPS_current[5] = 0x5252;
	tel->EPS_current[6] = 0x454E;
	tel->EPS_current[7] = 0x5400;

	tel->EPS_voltage[0] = 0x564F;
	tel->EPS_voltage[1] = 0x4C54;
	tel->EPS_voltage[2] = 0x5321;

	tel->RF_rssi = 0x21;
}
