/*
 * File_Handling_RTOS.c
 *
 *  Created on: 14-May-2020
 *      Author: Controllerstech
 */

#include <File_Handling.h>
#include "stm32f4xx_hal.h"

/* =============================>>>>>>>> NO CHANGES AFTER THIS LINE =====================================>>>>>>> */

FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

void Mount_SD (const TCHAR* path)
{
	fresult = f_mount(&fs, path, 1);
	if (fresult != FR_OK)
	{
		Error_Handler();
	}
}

void Unmount_SD (const TCHAR* path)
{
	fresult = f_mount(NULL, path, 1);
	if (fresult != FR_OK)
	{
		Error_Handler();
	}
}

FRESULT Create_File (char *name)
{
	fresult = f_stat (name, &fno);
	if (fresult == FR_OK)
	{
	    return fresult;
	}
	else
	{
		fresult = f_open(&fil, name, FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
		if (fresult != FR_OK)
		{
		    return fresult;
		}

		fresult = f_close(&fil);
	}
    return fresult;
}

FRESULT Update_File (char *name, char *data)
{
	 /* Create a file with read write access and open it */
	fresult = f_open(&fil, name, FA_OPEN_APPEND | FA_WRITE);

	/* Writing text */
	fresult = f_write(&fil, data, strlen(data), &bw);

	/* Close file */
	fresult = f_close(&fil);

    return fresult;
}
