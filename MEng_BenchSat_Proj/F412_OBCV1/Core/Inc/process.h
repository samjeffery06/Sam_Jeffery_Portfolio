/*
 *  OBC General Driver
 *
 *  Author: 	Sam Jeffery
 *  Created: 	11 May 2022
 *  Version: 	1.0
 */

#ifndef INC_PROCESS_H_
#define INC_PROCESS_H_

#include "main.h"

#include "LIS2MDL.h"
#include "ISM330.h"
#include "../shared_lib/CAN.h"
#include "solenoid.h"
#include "SD.h"

/*
 * Driver STRUCT
 */

typedef struct{

	uint8_t buffer[CAN_FIFO_LEN];

	uint8_t source;

	uint8_t msg_typ;

	uint8_t len;

} can_buf;

typedef struct{

	uint8_t tel_id;

	uint8_t new_data;

	uint8_t data[10];

} tel_rx_data;

/*
 * INITIALISATION
 */


/*
 * HIGH-LEVEL FUNCTIONS
 */

can_buf Process_CAN_INBOX( CANBUS *dev );

void Process_CAN_TX( CANBUS *dev );

/*
 * GENERAL FUNCTIONS
 */

void Process_DIR_GS( CANBUS *dev, uint8_t* message, uint8_t message_len );

void Process_EPS_SWITCHES( CANBUS *dev, uint8_t mask, uint8_t set );

void Process_TELEMETRY_REQ( CANBUS *dev, can_buf data, ISM330 *accgyro, LIS2MDL *comp );

void Process_TELEMETRY_RX( CANBUS *dev, can_buf data, tel_rx_data *save );

void Process_LIS2MDL_RCS_SEND( LIS2MDL *dev, CANBUS *comm );

void Process_LIS2MDL_TEST( LIS2MDL *comp, SD *store );

void Process_LIS2MDL_SD( LIS2MDL *comp, SD *store );

void Process_ISM330_ACC_TEST( ISM330 *acc_gyro, SD *store );

void Process_ISM330_ACC_SD( ISM330 *acc_gyro, SD *store );

void Process_ISM330_GYRO_TEST( ISM330 *acc_gyro, SD *store );

void Process_ISM330_GYRO_SD( ISM330 *acc_gyro, SD *store );

/*
 * BEACON
 */

void Process_BEACON( CANBUS *dev, uint8_t obc_status );


/*
 * LOW-LEVEL FUNCTIONS
 */

/*
 * UNIT TESTS
 */

void Process_OBC_Movement_Test( LIS2MDL *comp, SD *store, uint8_t ID );

void Process_OBC_Movement_Test_WRITE_DATA( uint8_t obc_state, uint8_t solenoid_state, ISM330 *acc_gyro, LIS2MDL *comp, SD *store, tel_rx_data *save  );

#endif /* INC_PROCESS_H_ */
