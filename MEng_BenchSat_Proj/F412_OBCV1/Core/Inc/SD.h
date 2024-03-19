/*
 *  SD Read/Write Driver
 *
 *  Author: 	Sam Jeffery
 *  Created: 	20 April 2022
 *  Version 	1.0
 */

/*
 * SETUP INSTRUCTIONS (My method not the only way)
 *
 * IN IOC:
 * 	FATFS -- USER Defined
 * 	USE_LFN -- EN with static buffer
 * 	MAX_SS 4096
 *
 * IN stm32f4xx_it.c

USER CODE BEGIN 0

volatile uint8_t FatFsCnt = 0;
volatile uint8_t Timer1, Timer2;

void SDTimer_Handler(void)
{
	if(Timer1 > 0)
		Timer1--;

	if(Timer2 > 0)
		Timer2--;
}

USER CODE END 0

 * and

USER CODE BEGIN SysTick_IRQn 0

FatFsCnt++;
if(FatFsCnt >= 10)
{
	FatFsCnt = 0;
	SDTimer_Handler();
}

USER CODE END SysTick_IRQn 0

 * In INC Folder: fatfs_sd.h, SD.h
 * In SRC Folder: fatfs_sd.c, SD.c
 * Set correct SPI and chip select pins in fatfs_sd.h
 *
 * Update replace user_diskio.c with copy

 */




#ifndef INC_SD_H_
#define INC_SD_H_

#include "main.h"
#include "fatfs.h"
#include "fatfs_sd.h"

/*
 * DEFINES
 */

#define MAX_FILENAME_LEN	30

/*
 * Driver STRUCT
 */

typedef struct{

	FIL fil;

	char filename[MAX_FILENAME_LEN];

} SD;

typedef struct{

	UART_HandleTypeDef *uartHandle;

	uint32_t total, free_space;

} SD_space;


/*
 * INITIALISATION
 */

uint8_t SD_Init( SD *dev, char* filename, uint8_t mount );

uint8_t SD_Mount();

uint8_t SD_Dismount();

/*
 * HIGH LEVEL FUNCTIONS
 */
uint8_t SD_writeNew( SD *dev, char* buffer);
//uint8_t SD_writeNew( SD *dev, char* filename, char* buffer);

uint8_t SD_readAll( SD *dev, char* buffer );

uint8_t SD_append( SD *dev, char* buffer );

/*
 * LOW-LEVEL FUNCTIONS
 */

/*
 * OTHER FUNCTIONS
 */

void SD_GetSpace( SD_space *dev );

/*
 * UNIT TESTS
 */

#endif /* INC_SD_H_ */
