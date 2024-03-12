#include "user_task.h"
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "kom.h"
#include "BME280_STM32.h"
#include "bno055.h"
#include "as5600.h"
#include "GPS.h"
#include "servo.h"
#include "tm_stm32f4_bkpsram.h"

#include "cmsis_os.h"

extern state_t cansatState;
extern cam_t camera;

extern osThreadId_t gimbalTaskHandle;
extern osThreadId_t telemetryTaskHandle;
extern osThreadId_t cameraTaskHandle;

extern osSemaphoreId_t telemetrySemaphoreHandle;
extern osSemaphoreId_t gimbalSemaphoreHandle;

extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef hi2c2;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim10;

uint8_t flagtel = 0, flagsim = 0;
uint16_t counting = 1;
datatelemetri_t datatelemetri;
uint8_t flagrefalt;

bno055_calibration_state_t calStat;
bno055_calibration_data_t calData;
extern bno055_vector_t bno055_euler, bno055_gyro;

float Temperature, Pressure, Humidity, Spressure = 101325, refalt = 0, tempalt = 0;

lwgps_t gps;
char rxgps[128];

float gpslat,gpslong,gpsalt;
uint8_t gpssat;
char gpsdetik[3], gpsmenit[3], gpsjam[3];

#define FILTER_SIZE 5
float readings[FILTER_SIZE];
int ind_ = 0;

#define RHO 1.225
#define ZERO_SPAN 15
#define VELOCITY_OFFSET_SIZE 20

int velocity_offset = 0;
float adc_avg;
uint32_t dataadc[2];

char commandbuff[15];

void init()
{
    strcpy(datatelemetri.state, "LAUNCH_WAIT");
    READRAM();
    datatelemetri.fmode = 'F';
    strcpy(datatelemetri.echocmd, "CXON");
    gpslat = 0.0000;
    gpslong = 0.0000;
    gpsalt = 0.0;
    gpssat = 0;
    strcpy(gpsjam, "00");
    strcpy(gpsmenit, "00");
    strcpy(gpsdetik, "00");
    servogerak(&htim3, TIM_CHANNEL_1, 0);
	servogerak(&htim3, TIM_CHANNEL_3, 0);
    servogerak(&htim4, TIM_CHANNEL_1, 0);
}

void READRAM()
{
    counting = TM_BKPSRAM_Read16(PACKETCOUNT_ADR);
    switch(TM_BKPSRAM_Read8(STATEIND_ADR))
    {
		case 0:
			strcpy(datatelemetri.state, "ASCENT");
			break;
		case 1:
			strcpy(datatelemetri.state, "ROCKET_SEPARATION");
			break;
		case 2:
			strcpy(datatelemetri.state, "DESCENT");
			break;
		case 3:
			strcpy(datatelemetri.state, "HS_RELEASE");
			break;
		case 4:
			strcpy(datatelemetri.state, "LANDED");
			break;
    }
    refalt = TM_BKPSRAM_ReadFloat(REFALT_ADR);
    cansatState = TM_BKPSRAM_Read8(STATEIND_ADR);
    flagtel = TM_BKPSRAM_Read8(FLAGTEL_ADR);
    datatelemetri.hsdeploy = TM_BKPSRAM_Read8(HSDEPLOY_ADR);
    datatelemetri.pcdeploy = TM_BKPSRAM_Read8(PCDEPLOY_ADR);
}

void RESETSRAM()
{
    TM_BKPSRAM_Write16(PACKETCOUNT_ADR, counting);
    TM_BKPSRAM_WriteFloat(REFALT_ADR, refalt);
    TM_BKPSRAM_Write8(STATEIND_ADR, (uint8_t)cansatState);
    TM_BKPSRAM_Write8(HSDEPLOY_ADR, datatelemetri.hsdeploy);
    TM_BKPSRAM_Write8(PCDEPLOY_ADR, datatelemetri.pcdeploy);
}

void adcinit()
{
	// Air speed calibration
	uint32_t adc_data = 0;
	HAL_ADC_Start(&hadc1);
	for (int i = 0; i < VELOCITY_OFFSET_SIZE; ++i)
	{
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		adc_data = HAL_ADC_GetValue(&hadc1);
		velocity_offset += adc_data - 2048;
	}
	HAL_ADC_Stop(&hadc1);
	velocity_offset /= VELOCITY_OFFSET_SIZE;

	// Reinitialize ADC for multiple channel reading
	ADC_ChannelConfTypeDef sConfig = {0};

	hadc1.Init.NbrOfConversion = 2;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = 2;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	// Start ADC DMA
	HAL_ADC_Start_DMA(&hadc1, dataadc, 2);
}

void bno055_init()
{
    bno055_assignI2C(&hi2c2);
    bno055_setup();
    bno055_setOperationModeNDOF();

//    for (;;)
//    {
//		calStat = bno055_getCalibrationState();
//
//		if (calStat.gyro == 3)
//			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
//		else
//			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
//
//		if (calStat.accel == 3)
//			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
//		else
//			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);
//
//		if (calStat.mag == 3)
//			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, SET);
//		else
//			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
//
//		if (calStat.sys == 3)
//			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, SET);
//		else
//			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, RESET);
//
//		if (calStat.gyro == 3 && calStat.mag == 3 && calStat.accel == 3)
//		{
//			calData = bno055_getCalibrationData();
//			bno055_setCalibrationData(calData);
//			break;
//		}
//    }

    osDelay(500);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, RESET);
    osDelay(500);

    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
}

void gpsinit()
{
	lwgps_init(&gps);
	HAL_UART_Receive_DMA(&huart2, (uint8_t *)rxgps, sizeof(rxgps));
}

void parsegpsdata()
{
	if (lwgps_process(&gps, rxgps, strlen(rxgps)))
		HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	HAL_UART_Receive_DMA(&huart2, (uint8_t *)rxgps, sizeof(rxgps));
}

void ADC_measure()
{
	adc_avg = 0;

	/** Battery voltage measurement
	 * ADC_Ref / MAX_ADC * ADC_Value * (R1 + R2 / R2)
	 * 
	 * NOTE: R1 is 10k, R2 is 4k7 and the battery voltage is 8.4 V
	 * here we use 2.69 as ADC_Ref since the highest possible output of
	 * voltage divider is ± 2.68 V while the actual ADC_Ref voltage is 3.3 V
	 */
	float sum = 0;
	readings[ind_] = ((2.68 / 3333.5) * dataadc[0]) * (14.7 / 4.7);
	ind_ = (ind_ + 1) % FILTER_SIZE;
	for (int i = 0; i < FILTER_SIZE; i++)
	{
		sum += readings[i];
		adc_avg += dataadc[1] - velocity_offset;
	}
	datatelemetri.voltage = sum / FILTER_SIZE;
	adc_avg /= FILTER_SIZE;

	/** Differential Pressure Transfer Function (MPXV7002DP datasheet page 6)
	 * Vout = Vs * (0.2 * P(kPa) + 0.5) ± 6.25% VFSS
	 * P(Pa) = 1000 * (Vout / Vs - 0.5 ) / 0.2
	 * P(Pa) = 5000 * (Vout / Vs - 0.5)
	 * 
	 * NOTE: Vout and Vs is in ADC value
	 *
	 * Indicated Airspeed (IAS) in m/s, Air density (RHO) = 1.204
	 * v = sqrt(2 * P / RHO)
	 * v = sqrt(10000 * (Vout / Vs - 0.5) / RHO)
	 */
	if (adc_avg >= (2048 - ZERO_SPAN) && adc_avg <= (2048 + ZERO_SPAN))
	{
		datatelemetri.airspeed = 0;
	}
	else
	{
		if (adc_avg < 2048 - ZERO_SPAN)
		{
			datatelemetri.airspeed = sqrtf((-10000.0 * (adc_avg / 4096.0 - 0.5)) / RHO);
		}
		else
		{
			datatelemetri.airspeed = sqrtf((10000.0 * (adc_avg / 4096.0 - 0.5)) / RHO);
		}
	}
}

uint8_t buatcs(char dat_[])
{
    uint8_t hasil = 0, temp = 0;
    uint16_t buffhasil = 0;
    for (int i = 0; i < sizeof(datatelemetri.telemetritotal); i++)
    {
        if (dat_[i]== '\0')
        {
            break;
        }
        else
        {
            buffhasil += dat_[i];
        }
    }
    hasil = buffhasil;
    temp = buffhasil >> 8;
    hasil += temp;
    return hasil;
}

float pressuretoalt(float press)
{
    float hasil;
    if (press != 0)
    {
    	hasil = 44330.0 * (1.0 - pow((press / 1013.25), 1 / 5.255));
    }
    else
    {
    	hasil = 0;
    }
    return hasil;
}

void wakturtc(uint8_t timebuff, char datat[])
{
    if (timebuff < 10)
    {
    	sprintf(datat,"0%d",timebuff);
    }
    else
    {
    	sprintf(datat,"%d",timebuff);
    }
}

void Settime(uint8_t jam_, uint8_t menit_, uint8_t detik_)
{
    RTC_TimeTypeDef sTime = {0};

    sTime.Hours = jam_;
    sTime.Minutes = menit_;
    sTime.Seconds = detik_;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;

    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
    {
    	Error_Handler();
    }

    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2);
}

void rtcbackup()
{
    if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1)!= 0x32F2)
    {
    	Settime(0,0,0);
    }
}

void CX()
{
    isidata(4, commandbuff);
    if ((commandbuff[0] == 'O') && (commandbuff[1] == 'N'))
    {
    	osSemaphoreRelease(telemetrySemaphoreHandle);
    	flagtel = 1;
		strcpy(datatelemetri.echocmd, "CXON");
    }
    else if ((commandbuff[0] == 'O') && (commandbuff[1] == 'F'))
    {
    	HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, RESET);
    	osSemaphoreAcquire(telemetrySemaphoreHandle, osWaitForever);
		flagtel = 0;
		strcpy(datatelemetri.echocmd, "CXOFF");
    }
	TM_BKPSRAM_Write8(FLAGTEL_ADR, flagtel);
}

void CAL()
{
    if (flagsim == 0)
    {
    	refalt = pressuretoalt(Pressure / 100);
    }
    flagsim = 0;
    flagrefalt = 0;
    counting  = 0;
    datatelemetri.packetcount = counting;
    datatelemetri.hsdeploy = 'N';
    datatelemetri.pcdeploy = 'N';
    cansatState = LAUNCH_WAIT;
    strcpy(datatelemetri.state, "LAUNCH_WAIT");
    strcpy(datatelemetri.echocmd, "CAL");

    RESETSRAM();

    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, RESET);
    resetPosition(0);
	resetCumulativePosition(0);
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
    servogerak(&htim3, TIM_CHANNEL_1, 0);
	servogerak(&htim3, TIM_CHANNEL_3, 0);
    servogerak(&htim4, TIM_CHANNEL_1, 0);
}

void ST()
{
    isidata(4, commandbuff);
    uint8_t bufjam, bufmenit, bufdetik;
    char bufjam_[3], bufmenit_[3], bufdetik_[3];
    if (commandbuff[0] == 'G')
    {
		bufjam = (uint8_t)atoi(gpsjam);
		bufmenit = (uint8_t)atoi(gpsmenit);
		bufdetik = (uint8_t)atoi(gpsdetik);
		Settime(bufjam, bufmenit, bufdetik);
		strcpy(datatelemetri.echocmd, "STGPS");
    }
    else
    {
		sprintf(bufjam_, "%c%c", commandbuff[0], commandbuff[1]);
		bufjam = (uint8_t)atoi(bufjam_);
		sprintf(bufmenit_, "%c%c", commandbuff[3], commandbuff[4]);
		bufmenit = (uint8_t)atoi(bufmenit_);
		sprintf(bufdetik_, "%c%c", commandbuff[6], commandbuff[7]);
		bufdetik = (uint8_t)atoi(bufdetik_);
		Settime(bufjam, bufmenit, bufdetik);
		wakturtc(bufjam, bufjam_);
		wakturtc(bufmenit, bufmenit_);
		wakturtc(bufdetik, bufdetik_);
		sprintf(datatelemetri.echocmd, "ST%c%c:%c%c:%c%c", bufjam_[0], bufjam_[1], bufmenit_[0], bufmenit_[1], bufdetik_[0], bufdetik_[1]);
    }
}

void SIM()
{
    isidata(4, commandbuff);
    if (flagsim == 0 && commandbuff[0] == 'E' && commandbuff[1] == 'N')
    {
		flagsim = 1;
		strcpy(datatelemetri.echocmd, "SIMENABLE");
    }
    else if (flagsim == 1 && commandbuff[0] == 'A' && commandbuff[1] == 'C')
    {
		flagsim = 2;
		datatelemetri.fmode = 'S';
		strcpy(datatelemetri.echocmd, "SIMACTIVATE");
    }
    else if (commandbuff[0] == 'D' && commandbuff[1] == 'I')
    {
		flagrefalt = 0;
		flagsim = 0;
		datatelemetri.fmode = 'F';
		strcpy(datatelemetri.echocmd, "SIMDISABLE");
    }
}

void SIMP()
{
    if (flagsim == 2)
    {
		isidata(4, commandbuff);
		Spressure = atof(commandbuff);
		sprintf(datatelemetri.echocmd, "SIMP%.2f", Spressure);
		if (!flagrefalt)
		{
			refalt = pressuretoalt(Spressure / 100);
			flagrefalt = 1;
		}
    }
}

void GB()
{
	isidata(4, commandbuff);
	if ((commandbuff[0] == 'O') && (commandbuff[1] == 'N'))
	{
		osSemaphoreRelease(gimbalSemaphoreHandle);
	}
	else if ((commandbuff[0] == 'O') && (commandbuff[1] == 'F'))
	{
		osSemaphoreAcquire(gimbalSemaphoreHandle, osWaitForever);
	}
}

void HS()
{
	servogerak(&htim4, TIM_CHANNEL_1, 135);
}

void BCN()
{
    isidata(4, commandbuff);
    if ((commandbuff[0] == 'O')&&(commandbuff[1] == 'N'))
    {
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, SET);
		strcpy(datatelemetri.echocmd, "BCNON");
    }
    else if ((commandbuff[0] == 'O')&&(commandbuff[1] == 'F'))
    {
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, RESET);
		strcpy(datatelemetri.echocmd, "BCNOFF");
    }
}

void CAM()
{
	isidata(4, commandbuff);
	switch (commandbuff[0])
	{
		case '1':
			camera = MAIN_CAM;
			break;
		case '2':
			camera = BONUS_CAM;
			break;
		case 'O':
			camera = CAM_OFF;
			break;
	}
	osThreadFlagsSet(cameraTaskHandle, 1);
}

void CR()
{
    NVIC_SystemReset();
}
