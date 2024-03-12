/*
 * kom.h
 *
 *  Created on: Dec 11, 2022
 *      Author: user
 */

#ifndef INC_KOM_H_
#define INC_KOM_H_

#include "stm32f4xx.h"

void isidata(uint8_t urutan,char dat_[]);
uint8_t cocokan(uint8_t urutan,char dat[]);
void checkdata_(void);
void kominit(void);

#endif /* INC_KOM_H_ */
