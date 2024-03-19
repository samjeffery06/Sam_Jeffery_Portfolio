/*
 *  EPS Voltage Monitoring ADC Driver
 *
 *  Author: 	Sam Jeffery
 *  Created: 	21 March 2022
 *  Version: 	1.0
 */

#ifndef INC_STMADC_H_
#define INC_STMADC_H_


#include "main.h"

/*
 * DEFINES
 */

#define Avg_Slope 	.0025
#define V25 		0.76
#define R1bat		120
#define R1 			68
#define R2 			33
#define ADC_CORR	1.198

/*
 * Driver STRUCT
 */

typedef struct{
	ADC_HandleTypeDef *adcHandle;

	uint16_t raw_adc[4];

	float temp;

	float voltage[3];

} stmADC;


/*
 * INITIALISATION
 */

void ADC_Initialise( stmADC *dev, ADC_HandleTypeDef *adc );

/*
 * DATA AQUISITION
 */

void ADC_GetALL( stmADC *dev );

void ADC_GetTemp( stmADC *dev );

void ADC_GetVBat( stmADC *dev );

void ADC_Get5V( stmADC *dev );

void ADC_Get3V3( stmADC *dev );

/*
 * LOW-LEVEL FUNCTIONS
 */

uint8_t ADC_Select_CHTemp ( stmADC *dev );

uint8_t ADC_Select_3V3 ( stmADC *dev );

uint8_t ADC_Select_VBat ( stmADC *dev );

uint8_t ADC_Select_5V ( stmADC *dev );

/*
 * UNIT TESTS
 */

void ADC_DisplayReadings( stmADC *dev, UART_HandleTypeDef *output, uint8_t *msg, uint8_t msg_len );


#endif /* INC_STMADC_H_ */
