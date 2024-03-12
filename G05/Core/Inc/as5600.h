#ifndef __AS5600_H
#define __AS5600_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal_i2c.h"

#define AS5600_ADDR			0x36 << 1
#define AS5600_ANGLE		0x0E
#define AS5600_RAW_ANGLE	0x0C

int16_t readReg2(uint8_t reg);

int32_t getCumulativePosition(void);

int32_t resetPosition(int32_t position);

int32_t resetCumulativePosition(int32_t position);

#endif /* __AS5600_H */
