/*
 * File_Handling_RTOS.h
 *
 *  Created on: 14-May-2020
 *      Author: Controllerstech
 */

#ifndef FILE_HANDLING_RTOS_H_
#define FILE_HANDLING_RTOS_H_

#include "fatfs.h"
#include "string.h"
#include "stdio.h"


/* mounts the sd card*/
void Mount_SD (const TCHAR* path);

/* unmounts the sd card*/
void Unmount_SD (const TCHAR* path);

/* creates the file, if it does not exists
 * @ name : is the path to the file*/
FRESULT Create_File (char *name);

/* updates the file. write pointer is set to the end of the file
 * @ name : is the path to the file
 */
FRESULT Update_File (char *name, char *data);




#endif /* FILE_HANDLING_RTOS_H_ */
