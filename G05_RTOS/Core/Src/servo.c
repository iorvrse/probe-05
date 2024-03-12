#include "servo.h"

void servogerak(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t sudut)
{
	uint16_t regisval;
	if (sudut == 0)
	{
		regisval = 500;
	}
	else
	{
		regisval = 11.17*sudut + 490.7;
	}

	switch (channel)
	{
		case TIM_CHANNEL_1:
			htim->Instance->CCR1 = regisval;
			break;
		case TIM_CHANNEL_2:
			htim->Instance->CCR2 = regisval;
			break;
		case TIM_CHANNEL_3:
			htim->Instance->CCR3 = regisval;
			break;
		case TIM_CHANNEL_4:
			htim->Instance->CCR4 = regisval;
			break;
	}
}
