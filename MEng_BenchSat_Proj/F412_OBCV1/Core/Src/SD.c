/*
 *  SD Read/Write Driver
 *
 *  Author: 	Sam Jeffery
 *  Created: 	20 April 2022
 *  Version 	1.0
 */

#include "SD.h"

/*
 * INITIALISATION
 */

uint8_t SD_Init( SD *dev, char* filename, uint8_t mount )
{
	// NB only mount SD card once
	// No check for whether SD card is present
	uint8_t let = 0;
	while ( filename[let] != 0 && let < MAX_FILENAME_LEN )
	{
		dev->filename[let] = filename[let];
		let++;
	}
	if (mount != 0)
	{
		return SD_Mount();
	}

	return 0;
}

uint8_t SD_Mount()
{
	FRESULT fresult;
	FATFS fs;
	fresult = f_mount(&fs, "", 0);

	return fresult != FR_OK;
}

uint8_t SD_Dismount()
{
	FRESULT fresult;
	fresult = f_mount(NULL, "/", 1);
	return fresult != FR_OK;
}

/*
 * HIGH LEVEL FUNCTIONS
 */

uint8_t SD_writeNew( SD *dev, char* buffer )
{
//	FRESULT fs;
	uint8_t err = 0;

	err += (f_open(&dev->fil, dev->filename, FA_CREATE_ALWAYS | FA_READ | FA_WRITE) != FR_OK);

	/* Writing text */
//	fs = f_puts(buffer, &dev->fil);
	err += (f_puts(buffer, &dev->fil) != FR_OK);

	/* Close file */
	err += (f_close(&dev->fil) != FR_OK);

	return err;
}

uint8_t SD_append( SD *dev, char* buffer )
{
	uint8_t err = 0;

	err += (f_open(&dev->fil, dev->filename, FA_OPEN_APPEND | FA_WRITE) != FR_OK);

	/* Writing text */
	err += (f_puts(buffer, &dev->fil) != FR_OK);

	/* Close file */
	err += (f_close(&dev->fil) != FR_OK);

	return err;
}


uint8_t SD_readAll( SD *dev, char* buffer )
{
	UINT br = 0;
	/* Open file to read */
	f_open( &dev->fil, dev->filename, FA_READ );

	/* Read string from the file */
	uint8_t size = f_size(&dev->fil);

	while (br < size)
	{
		f_read (&dev->fil, &buffer[br], size, &br);
	}

	/* Close file */
	f_close( &dev->fil );

	return size;
}

/*
 * LOW-LEVEL FUNCTIONS
 */

/*
 * OTHER FUNCTIONS
 */

void SD_GetSpace( SD_space *dev )
{
	DWORD fre_clust;
	FATFS *pfs;
	f_getfree("", &fre_clust, &pfs);
	dev->total =  (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
	dev->free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
}

/*
 * UNIT TESTS
 */

