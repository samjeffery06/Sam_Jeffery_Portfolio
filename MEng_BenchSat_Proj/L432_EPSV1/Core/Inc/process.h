/*
 *  EPS General Driver
 *
 *  Author: 	Sam Jeffery
 *  Created: 	27 May 2022
 *  Version: 	1.0
 */

#ifndef INC_PROCESS_H_
#define INC_PROCESS_H_

#include "main.h"

#include "switch.h"
#include "stmADC.h"
#include "ADS7828.h"
#include "../shared_lib/CAN.h"


/*
 * Driver STRUCT
 */

typedef struct{


} TELEMETRY;


/*
 * INITIALISATION
 */


/*
 * HIGH-LEVEL FUNCTIONS
 */

void EPS_PROCESS_OLD( CANBUS *canbus, TELEMETRY *tel );

void EPS_PROCESS( CANBUS *canbus, switches *sw, stmADC *volt, ADS7828 *curr );

/*
 * LOW-LEVEL FUNCTIONS
 */

void CAN_PROCESS( CANBUS *canbus, TELEMETRY *tel );

/*
 * UNIT TESTS
 */

void EPS_Power_Test( UART_HandleTypeDef *huart, stmADC *volt, ADS7828 *curr );

void EPS_SYS_CURR_Test( UART_HandleTypeDef *huart, ADS7828 *curr );

void EPS_SYS_CURR_SEND( CANBUS *canbus, ADS7828 *curr );


#endif /* INC_PROCESS_H_ */
