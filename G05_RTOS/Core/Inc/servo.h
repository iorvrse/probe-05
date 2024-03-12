#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "stm32f4xx_hal.h"

void servogerak(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t sudut);

#endif /* INC_SERVO_H_ */
