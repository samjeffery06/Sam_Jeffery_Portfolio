/*
 *  RF2 General Driver
 *
 *  Author: 	Sam Jeffery
 *  Created: 	11 April 2022
 *  Version: 	1.0
 */

#ifndef INC_PROCESS_H_
#define INC_PROCESS_H_


#include "main.h"

#include "S2LP_V2.h"

//#include "NRX1UART.h"

#include "NRX1TIM.h"

#include "../shared_lib/CAN.h"


/*
 * Driver STRUCT
 */

typedef struct{

	uint16_t OBC_acc[3]; // x, y, z

	uint16_t OBC_gyro[3]; // pitch, roll, yaw

	uint16_t OBC_comp;

	uint8_t OBC_sol;

	uint16_t OBC_temp;

	uint16_t ADCS_position_pose[3]; // X, Y, ANGLE

	uint8_t EPS_switch;

	uint16_t EPS_current[8];

	uint16_t EPS_voltage[3];

	uint16_t RF_rssi;

} TELEMETRY;


/*
 * INITIALISATION
 */


/*
 * HIGH-LEVEL FUNCTIONS
 */
//void RF_PROCESS( S2LP *s2lp, NRX1_TIM *nrx1, CANBUS *canbus, TELEMETRY *tel, UART_HandleTypeDef *huart );
void RF_PROCESS( S2LP *s2lp, NRX1_TIM *nrx1, CANBUS *canbus, TELEMETRY *tel );

void CAN_PROCESS( S2LP *s2lp, CANBUS *canbus, TELEMETRY *tel );

/*
 * LOW-LEVEL FUNCTIONS
 */

void RF_TRANSMIT_ACK( S2LP *s2lp, uint8_t source, uint8_t taskID, uint8_t ACKNACK );

void RF_TRANSMIT_TEL( S2LP *s2lp, TELEMETRY *tel, uint8_t tel_no );

/*
 * UNIT TESTS
 */

void PROCESS_TEL_UNIT( TELEMETRY *tel );

#endif /* INC_PROCESS_H_ */
