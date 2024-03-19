/*
 *  OBC General Driver
 *
 *  Author: 	Sam Jeffery
 *  Created: 	11 May 2022
 *  Version: 	1.0
 */

#include "process.h"

#include <stdio.h>

/*
 * INITIALISATION
 */


/*
 * CAN FUNCTIONS
 */

can_buf Process_CAN_INBOX( CANBUS *dev )
{
	can_buf temp;
	for (uint8_t i = 0; i < OTHER_NODES; i++)
	{
		if (dev->RX_received_flag[i] == 1)
		{
			/*
			msg_typ: canbus->RXstack[i][0] & 0x3;
			source: canbus->RXstack[i][1] & 0xF;
			len: canbus->RXstack[i][2];
			data start: canbus->RXstack[i][3]
			 */
			temp.msg_typ = dev->RXstack[i][0] & 0x3;
			temp.source = dev->RXstack[i][1] & 0xF;
			temp.len = dev->RXstack[i][2];
			for (uint8_t j = 0; j < temp.len; j++)
			{
				temp.buffer[j] = dev->RXstack[i][3+j];
			}
			CAN_CLEAR( dev, i );
			dev->RX_received_flag[i] = 0;
			dev->RX_message_pointer[i] = 0;
			return temp;
		}
	}
	temp.msg_typ = 4;
	return temp;
}

void Process_CAN_TX( CANBUS *dev )
{
	//
}

/*
 * GENERAL FUNCTIONS
 */

void Process_DIR_GS( CANBUS *dev, uint8_t* message, uint8_t message_len )
{
	CAN_TX_ID TX_ID = CAN_setID( SAT_OBC, SAT_RCS, MSG_DIR );
	CAN_Transmit( dev, TX_ID, message, message_len );
}

void Process_EPS_SWITCHES( CANBUS *dev, uint8_t mask, uint8_t set )
{
	uint8_t message[3];
	message[0] = 1;
	message[1] = mask;
	message[2] = set;

	CAN_TX_ID TX_ID = CAN_setID( SAT_OBC, SAT_EPS, MSG_DIR );
	CAN_Transmit( dev, TX_ID, (uint8_t*) message, 3 );
}

void Process_TELEMETRY_REQ( CANBUS *dev, can_buf data, ISM330 *accgyro, LIS2MDL *comp )
{

	if (data.buffer[1] == 0)
	{
		// Process TELEMETRY
		uint8_t TEL_MSG[8];
		uint8_t TEL_LEN;
		switch(data.buffer[0])
		{
			case 0:
				// Get Acceleration Data
				ISM330_ReadAcc(accgyro);
				/*
				 * ACC DATA UINT16_t[3]
				 * [TEL_ID][SEND_DATA][ACC1u][ACC1l][ACC2u][ACC2l][ACC3u][ACC3l]
				 */
				TEL_LEN = 8;
				TEL_MSG[0] = data.buffer[0];  	// Set TEL_ID
				TEL_MSG[1] = 1;					// Sending data

				for ( uint8_t i = 0; i < 3; i++ )
				{
					TEL_MSG[2*i+2] = (accgyro->acc_raw[i] >> 8) & 0x00FF;
					TEL_MSG[2*i+3] = (accgyro->acc_raw[i]) & 0x00FF;
				}
				break;
			case 1:
				// Get Gyroscope Data
				ISM330_ReadGyro(accgyro);
				/*
				 * GYRO DATA UINT16_t[3]
				 * [TEL_ID][SEND_DATA][GYRO1u][GYRO1l][GYRO2u][GYRO2l][GYRO3u][GYRO3l]
				 */
				TEL_LEN = 8;
				TEL_MSG[0] = data.buffer[0];  	// Set TEL_ID
				TEL_MSG[1] = 1;					// Sending data
				for ( uint8_t i = 0; i < 3; i++ )
				{
					TEL_MSG[2*i+2] = (accgyro->gyro_raw[i] >> 8) & 0x00FF;
					TEL_MSG[2*i+3] = (accgyro->gyro_raw[i]) & 0x00FF;
				}
				break;
			case 2:
				// Get Compass Data
				LIS2MDL_ReadCompass(comp);
				/*
				 * COMP DATA UINT16_t[3]
				 * [TEL_ID][SEND_DATA][HEADu][HEADl]
				 */
				TEL_LEN = 4;
				TEL_MSG[0] = data.buffer[0];  	// Set TEL_ID
				TEL_MSG[1] = 1;					// Sending data
				TEL_MSG[2] = (comp->heading >> 8) & 0x00FF;
				TEL_MSG[3] = (comp->heading) & 0x00FF;
				break;
			case 3:
				// Get Temperature Data
				ISM330_ReadTemperature(accgyro);
				/*
				 * COMP DATA UINT16_t[3]
				 * [TEL_ID][SEND_DATA][TEMPu][TEMPl]
				 */
				TEL_LEN = 4;
				TEL_MSG[0] = data.buffer[0];  	// Set TEL_ID
				TEL_MSG[1] = 1;					// Sending data
				TEL_MSG[2] = (accgyro->temp_C_raw >> 8) & 0x00FF;
				TEL_MSG[3] = (accgyro->temp_C_raw) & 0x00FF;
				break;
			case 4:
				// Get POSE
				break;
			}

		CAN_TX_ID TX_ID = CAN_setID( SAT_OBC, data.source, MSG_TEL );
		CAN_Transmit( dev, TX_ID, (uint8_t*) TEL_MSG, TEL_LEN ); // SENDS [ID][REQ]
	}
}

void Process_TELEMETRY_RX( CANBUS *dev, can_buf data, tel_rx_data *save )
{
	if (data.buffer[1] == 1)
	{
		save->tel_id = data.buffer[0];

		switch(data.buffer[0])
		{
		case 0x10:
			for (uint8_t i = 0; i < 4; i++)
			{
				save->data[i] = data.buffer[i+2];
				save->new_data = 0x10;
			}
		}
	}
}

/*
 * BEACON
 */

void Process_BEACON( CANBUS *dev, uint8_t obc_status )
{
	uint8_t beacon[6];

	uint32_t time = HAL_GetTick();

	beacon[0] = TASK_BEACON;
	beacon[1] = obc_status;
	beacon[2] = time & 0xFF;
	beacon[3] = time>>8 & 0xFF;
	beacon[4] = time>>16 & 0xFF;
	beacon[5] = time>>24 & 0xFF;

	CAN_TX_ID TX_ID = CAN_setID( SAT_OBC, SAT_RCS, MSG_DIR );
	CAN_Transmit( dev, TX_ID, beacon, 6 );
}

/*
 * LOW-LEVEL FUNCTIONS
 */


/*
 * UNIT TESTS
 */

void Process_OBC_Movement_Test( LIS2MDL *comp, SD *store, uint8_t ID )
{
	char SD_buffer[40];

//	uint8_t err = 0;
	sprintf( SD_buffer, "OBC_LOG_%d.csv", ID );

//	err = SD_Init( store, SD_buffer, 0 );
	SD_Init( store, SD_buffer, 0 );

//	err = SD_writeNew(store,"Hello");

//	err = SD_append( store, "Movement Test\nCompass Details\n");
	SD_append( store, "Movement Test\nCompass Details\n");

	sprintf( SD_buffer, "MAXX,MINX,MAXY,MINY\n%d,%d,%d,%d\n", comp->compass_max[0], comp->compass_min[0], comp->compass_max[1], comp->compass_min[1] );

	SD_append( store, SD_buffer );

	sprintf( SD_buffer, "OFFSETX,OFFSETY,REF_HEAD\n%d,%d,%d\n", comp->offsets[0], comp->offsets[1], comp->ref_heading );

	SD_append( store, SD_buffer );

	SD_append( store, "DATA START\n" );

	SD_append( store, "TIME,OBC_STATE,SOLENOID_STATE,COMP_X,COMP_Y,HEAD,ACC_X,ACC_Y,GYRO_YAW,TEL_ID,NEW_DATA,5VCurr,3V3Curr\n" );
}

void Process_OBC_Movement_Test_WRITE_DATA( uint8_t obc_state, uint8_t solenoid_state, ISM330 *acc_gyro, LIS2MDL *comp, SD *store, tel_rx_data *save )
{
	char SD_buffer[50];

	LIS2MDL_ReadCompass( comp );
	ISM330_ReadAcc( acc_gyro );
	ISM330_ReadGyro( acc_gyro );
	uint16_t currTemp5V = save->data[0] + save->data[1]*256;
	uint16_t currTemp3V3 = save->data[2] + save->data[3]*256;
	uint8_t new_data_temp = save->new_data;

	save->new_data = 0;

	sprintf( SD_buffer, "%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n", HAL_GetTick(), obc_state, solenoid_state, comp->compass[0], comp->compass[1], comp->heading, acc_gyro->acc_raw[0], acc_gyro->acc_raw[1], acc_gyro->gyro_raw[2], save->tel_id, new_data_temp, currTemp5V, currTemp3V3  );



	SD_append(store, SD_buffer);
}


/*
 * LIS2MDL
 * COMPASS PROCESSES
 */

void Process_LIS2MDL_TEST( LIS2MDL *comp, SD *store )
{
	char SD_buffer[40];

	SD_Init( store, "COMPASS_TEST1.csv", 1 );

	SD_writeNew( store, "COMPASS TEST\n");

	sprintf( SD_buffer, "MAXX,MINX,MAXY,MINY\n%d,%d,%d,%d\n", comp->compass_max[0], comp->compass_min[0], comp->compass_max[1], comp->compass_min[1] );

	SD_append( store, SD_buffer );

	sprintf( SD_buffer, "OFFSETX,OFFSETY,REF_HEAD\n%d,%d,%d\n", comp->offsets[0], comp->offsets[1], comp->ref_heading );

	SD_append( store, SD_buffer );

	SD_append( store, "TIME,COMPX,COMPY,HEAD\n" );
}

void Process_LIS2MDL_RCS_SEND( LIS2MDL *comp, CANBUS *comm )
{
	uint8_t output[9];
	LIS2MDL_ReadCompass( comp );

	output[0] = 3;
	output[1] = comp->compass[0] & 0xFF;
	output[2] = (comp->compass[0]>>8) & 0xFF;
	output[3] = comp->compass[1] & 0xFF;
	output[4] = (comp->compass[1]>>8) & 0xFF;
	output[5] = comp->heading & 0xFF;
	output[6] = (comp->heading>>8) & 0xFF;
	output[7] = comp->ref_heading & 0xFF;
	output[8] = (comp->ref_heading>>8) & 0xFF;

	Process_DIR_GS( comm, output, 9);
}

void Process_LIS2MDL_SD( LIS2MDL *comp, SD *store )
{
	char SD_buffer[20];
	LIS2MDL_ReadCompass( comp );

	sprintf( SD_buffer, "%ld,%d,%d,%d\n", HAL_GetTick(), comp->compass[0], comp->compass[1], comp->heading );
	SD_append(store, SD_buffer);

}

/*
 * ISM330
 * ACCELEROMETER AND GYRO PROCESSES
 */

void Process_ISM330_ACC_TEST( ISM330 *acc_gyro, SD *store )
{
	SD_Init( store, "ACC_TEST3.csv", 1 );

	SD_append( store, "ACC TEST\nTIME,X,Y,Z\n");
}

void Process_ISM330_ACC_SD( ISM330 *acc_gyro, SD *store )
{
	char SD_buffer[20];
	ISM330_ReadAcc( acc_gyro );

	sprintf( SD_buffer, "%ld,%d,%d,%d\n", HAL_GetTick(), acc_gyro->acc_raw[0], acc_gyro->acc_raw[1], acc_gyro->acc_raw[2] );
	SD_append(store, SD_buffer);
}

void Process_ISM330_GYRO_TEST( ISM330 *acc_gyro, SD *store )
{
	SD_Init( store, "GYRO_TEST.csv", 1 );
	SD_append( store, "GYRO TEST\nTIME,YAW,Gyro0,Gyro1\n");
}

void Process_ISM330_GYRO_SD( ISM330 *acc_gyro, SD *store )
{
	char SD_buffer[20];
	ISM330_ReadGyro( acc_gyro );

	sprintf( SD_buffer, "%ld,%d,%d,%d\n", HAL_GetTick(), acc_gyro->gyro_raw[2], acc_gyro->gyro_raw[0], acc_gyro->gyro_raw[1] );
	SD_append(store, SD_buffer);
}
