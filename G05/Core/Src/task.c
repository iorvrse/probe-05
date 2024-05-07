#include "main.h"
#include "task.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "kom.h"
#include "BME280_STM32.h"
#include "bno055.h"
#include "GPS.h"
#include "servo.h"
#include "tm_stm32f4_bkpsram.h"
#include "as5600.h"

extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef hi2c2;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim3;

uint8_t flagkameraon;
uint8_t flagkameraoff;

uint8_t flagtel=0, flagsim = 0,flagstate = 0, valid = 0;
uint16_t counting = 1;
datatelemetri_t datatelemetri;
uint8_t check, csh;
uint8_t flagrefalt;
uint8_t flaginvalid;
uint8_t flaggimbal = 1;
uint8_t flagcal = 1;

bno055_calibration_state_t bno055_calStat;
bno055_calibration_data_t bno055_calData;
extern bno055_vector_t bno055_euler, bno055_gyro;

float Temperature, Pressure, Humidity, Spressure = 101325, refalt = 0, tempalt = 0;

lwgps_t gps;
char rxgps[128];

float gpslat,gpslong,gpsalt;
uint8_t gpssat;
char gpsdetik[4], gpsmenit[4], gpsjam[4];

char commandbuff[15];

#define FILTER_SIZE 5
float readings[FILTER_SIZE];
int ind_ = 0;

#define RHO 1.225
#define ZERO_SPAN 15
#define VELOCITY_OFFSET_SIZE 20
int velocity_offset = 0;
float adc_avg;
uint32_t dataadc[2];

void init()
{
    READRAM();
    strcpy(datatelemetri.state, "LAUNCH_WAIT");
    strcpy(datatelemetri.echocmd, "CXON");
    strcpy(gpsjam, "00");
    strcpy(gpsmenit, "00");
    strcpy(gpsdetik, "00");
    datatelemetri.fmode = 'F';
    gpslat = 0.0000;
    gpslong = 0.0000;
    gpsalt = 0.0;
    gpssat = 0;
	servogerak(&htim3, TIM_CHANNEL_3, 0);
}

void READRAM()
{
    counting = TM_BKPSRAM_Read16(PACKETCOUNT_ADR);
    refalt = TM_BKPSRAM_ReadFloat(REFALT_ADR);
    flagstate = TM_BKPSRAM_Read8(STATEIND_ADR);
    switch(flagstate)
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
    flagtel = TM_BKPSRAM_Read8(FLAGTEL_ADR);
    datatelemetri.hsdeploy = TM_BKPSRAM_Read8(HSDEPLOY_ADR);
    datatelemetri.pcdeploy = TM_BKPSRAM_Read8(PCDEPLOY_ADR);
    flaggimbal = TM_BKPSRAM_Read8(FLAGGIMBAL_ADR);
    flagcal = TM_BKPSRAM_Read8(FLAGCAL_ADR);
    bno055_calData = TM_BKPSRAM_ReadCalData(BNO055CAL_ADR);
}

void RESETSRAM()
{
    TM_BKPSRAM_Write16(PACKETCOUNT_ADR, counting);
    TM_BKPSRAM_WriteFloat(REFALT_ADR, refalt);
    TM_BKPSRAM_Write8(STATEIND_ADR, flagstate);
    TM_BKPSRAM_Write8(HSDEPLOY_ADR, datatelemetri.hsdeploy);
    TM_BKPSRAM_Write8(PCDEPLOY_ADR, datatelemetri.pcdeploy);
    TM_BKPSRAM_Write8(FLAGGIMBAL_ADR, flaggimbal);
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

    if (!flagcal)
    {
    	char calbuff[20];
		for (;;)
		{
			bno055_calStat = bno055_getCalibrationState();
			sprintf(calbuff, "%d,%d,%d,%d\r\n", bno055_calStat.accel, bno055_calStat.gyro, bno055_calStat.mag, bno055_calStat.sys);
			HAL_UART_Transmit(&huart3, (uint8_t *)calbuff, strlen(calbuff), HAL_MAX_DELAY);

			if (bno055_calStat.gyro == 3)
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
			else
				HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);

			if (bno055_calStat.accel == 3)
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
			else
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);

			if (bno055_calStat.mag == 3)
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, SET);
			else
				HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);

			if (bno055_calStat.sys == 3)
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, SET);
			else
				HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, RESET);

			if (bno055_calStat.gyro == 3 && bno055_calStat.mag == 3 && bno055_calStat.accel == 3 && bno055_calStat.sys)
			{
				bno055_calData = bno055_getCalibrationData();
				bno055_setCalibrationData(bno055_calData);
				TM_BKPSRAM_WriteCalData(BNO055CAL_ADR, bno055_calData);
				flagcal = 1;
				TM_BKPSRAM_Write8(FLAGCAL_ADR, flagcal);
				break;
			}
		}
    }
    else
    {
    	bno055_setCalibrationData(bno055_calData);
    }

    HAL_GPIO_WritePin(GPIOE, LED1_Pin | LED2_Pin | LED3_Pin | LED4_Pin, RESET);

    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
}

void ADC_measure()
{
	adc_avg = 0;

	/** Battery voltage measurement
	 * ADC_Ref / MAX_ADC * ADC_Value * (R1 + R2 / R2)
	 * 
	 * NOTE: R1 is 10k, R2 is 4k7 and the battery voltage is 8.4 V
	 * here we use 2.68 as ADC_Ref and 3333 as MAX_ADC since the highest possible output of
	 * voltage divider is ± 2.68 V while the actual ADC_Ref voltage is 3.3 V
	 */
	float sum = 0;
	readings[ind_] = ((2.68 / 3333) * dataadc[0]) * (14.7 / 4.7);
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
    for (int i = 0; i < 175; i++)
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
    	hasil = 44330.0 * (1.0 - pow((press/1013.25), (1/5.255)));
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

void get_time()
{
    RTC_TimeTypeDef gTime;
    RTC_DateTypeDef gDate;

    HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);

    wakturtc(gTime.Hours,datatelemetri.jam);
    wakturtc(gTime.Minutes,datatelemetri.menit);
    wakturtc(gTime.Seconds,datatelemetri.detik);
}

void Settime(uint8_t jam_, uint8_t menit_, uint8_t detik_)
{
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    sTime.Hours = jam_;
    sTime.Minutes = menit_;
    sTime.Seconds = detik_;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
    {
    	Error_Handler();
    }
    sDate.WeekDay = RTC_WEEKDAY_SATURDAY;
    sDate.Month = RTC_MONTH_JUNE;
    sDate.Date = 0x8;
    sDate.Year = 0x24;

    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
    {
    	Error_Handler();
    }

    HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2);
}

void rtcbackup()
{
    if (HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F2)
    {
    	Settime(0,0,0);
    }
}

void ambildata()
{
    get_time();
    datatelemetri.packetcount = counting;
    datatelemetri.temp = Temperature;
    tempalt = datatelemetri.alt;
    if (flagsim == 0 || flagsim == 1)
    {
		datatelemetri.alt = pressuretoalt(Pressure / 100);
		datatelemetri.barpress = Pressure / 1000;
		datatelemetri.alt -= refalt;
    }
    else if (flagsim == 2)
    {
		datatelemetri.alt = pressuretoalt(Spressure / 100);
		datatelemetri.barpress = Spressure / 1000;
		datatelemetri.alt -= refalt;
    }
    if (datatelemetri.alt < 0)
    {
    	datatelemetri.alt = 0;
    }
    state();
    datatelemetri.rot_z = bno055_gyro.z;
    datatelemetri.heading = bno055_euler.x;
    datatelemetri.tilt_x = bno055_euler.y;
    datatelemetri.tilt_y = bno055_euler.z;

    sprintf(datatelemetri.telemetribuff,"2032,%c%c:%c%c:%c%c,%d,%c,%s,%.1f,%.2f,%c,%c,%.1f,%.1f,%.1f,%c%c:%c%c:%c%c,%.1f,%.4f,%.4f,%d,%.2f,%.2f,%.1f,%s,,%.1f,",
        		datatelemetri.jam[0], datatelemetri.jam[1], datatelemetri.menit[0], datatelemetri.menit[1], datatelemetri.detik[0], datatelemetri.detik[1],
    			datatelemetri.packetcount, datatelemetri.fmode, datatelemetri.state, datatelemetri.alt, datatelemetri.airspeed, datatelemetri.hsdeploy, datatelemetri.pcdeploy,
    			datatelemetri.temp, datatelemetri.voltage, datatelemetri.barpress, gpsjam[0], gpsjam[1], gpsmenit[0], gpsmenit[1], gpsdetik[0], gpsdetik[1],
    			gpsalt, gpslat, gpslong, gpssat, datatelemetri.tilt_x, datatelemetri.tilt_y, datatelemetri.rot_z, datatelemetri.echocmd, datatelemetri.heading);
	csh = ~buatcs(datatelemetri.telemetribuff);
	sprintf(datatelemetri.telemetritotal,"%s%d\r\n", datatelemetri.telemetribuff, csh);
}

void kirimdata()
{
    HAL_UART_Transmit_DMA(&huart3, (uint8_t *)datatelemetri.telemetritotal, strnlen(datatelemetri.telemetritotal, sizeof(datatelemetri.telemetritotal)));
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, SET);
}

void state()
{
    if ((tempalt-datatelemetri.alt) > 49)
    	flaginvalid = 1;
    else
    	flaginvalid = 0;

    if (datatelemetri.alt > 100 && flagstate == 0)
    {
		strcpy(datatelemetri.state, "ASCENT");
		flagstate = 1;
		TM_BKPSRAM_Write8(STATEIND_ADR,0);
    }
    else if ((datatelemetri.alt - tempalt) < 0 && flagstate == 1)
    {
    	valid++;
		if (valid > 4)
		{
			valid = 0;

			strcpy(datatelemetri.state, "ROCKET_SEPARATION");
			datatelemetri.hsdeploy = 'P';
			datatelemetri.pcdeploy 	= 'N';
			TM_BKPSRAM_Write8(HSDEPLOY_ADR,datatelemetri.hsdeploy);
			TM_BKPSRAM_Write8(PCDEPLOY_ADR,datatelemetri.pcdeploy);

			flagstate = 2;
			TM_BKPSRAM_Write8(STATEIND_ADR, 1);
		}
    }
    else if ((datatelemetri.alt - tempalt) < 0 && flagstate == 2)
    {
		strcpy(datatelemetri.state, "DESCENT");
		flagstate = 3;
		TM_BKPSRAM_Write8(STATEIND_ADR,2);
    }
    else if (datatelemetri.alt <= 150 && flagstate == 3 && !flaginvalid)
    {
		servogerak(&htim3, TIM_CHANNEL_3, 135);
		if (datatelemetri.alt <= 100)
		{
			strcpy(datatelemetri.state, "HS_RELEASE");
			datatelemetri.pcdeploy = 'C';
			datatelemetri.hsdeploy = 'P';
			TM_BKPSRAM_Write8(HSDEPLOY_ADR,datatelemetri.hsdeploy);
			TM_BKPSRAM_Write8(PCDEPLOY_ADR,datatelemetri.pcdeploy);

			flagstate = 4;
			TM_BKPSRAM_Write8(STATEIND_ADR,3);
    	}
    }
    else if (datatelemetri.alt < 13 && flagstate == 4 && !flaginvalid)
    {
		strcpy(datatelemetri.state, "LANDED");
		datatelemetri.pcdeploy = 'C';
		datatelemetri.hsdeploy = 'P';
		TM_BKPSRAM_Write8(HSDEPLOY_ADR,datatelemetri.hsdeploy);
		TM_BKPSRAM_Write8(PCDEPLOY_ADR,datatelemetri.pcdeploy);

		valid++;
		if (valid > 5)
		{
			valid = 0;

			flagkameraoff = 1;
			flagtel = 0;
			flaggimbal = 0;
			TM_BKPSRAM_Write8(FLAGGIMBAL_ADR, flaggimbal);
			HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, SET);
			flagstate = 5;
			TM_BKPSRAM_Write8(STATEIND_ADR, 4);
		}
    }
    else if (datatelemetri.alt < 13 && flagstate == 5 && !flaginvalid)
	{
    	// do nothing
	}
}

void CX()
{
    isidata(4, commandbuff);
    if ((commandbuff[0] == 'O') && (commandbuff[1] == 'N'))
    {
    	flagtel = 1;
    }
    else if ((commandbuff[0] == 'O') && (commandbuff[1] == 'F'))
    {
		flagtel = 0;
    }
	TM_BKPSRAM_Write8(FLAGTEL_ADR, flagtel);
    strcpy(datatelemetri.echocmd, "CXON");
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

void ST()
{
    isidata(4, commandbuff);
    uint8_t bufjam, bufmenit, bufdetik;
    char bufjam_[3], bufmenit_[3], bufdetik_[3];
    if (commandbuff[0] == 'G')
    {
		sprintf(bufjam_, "%c%c", gpsjam[0], gpsjam[1]);
		sprintf(bufmenit_, "%c%c", gpsmenit[0], gpsmenit[1]);
		sprintf(bufdetik_, "%c%c", gpsdetik[0], gpsdetik[1]);
		bufjam = (uint8_t)atoi(bufjam_);
		bufmenit = (uint8_t)atoi(bufmenit_);
		bufdetik = (uint8_t)atoi(bufdetik_);
		Settime(bufjam, bufmenit, bufdetik);
		wakturtc(bufjam, bufjam_);
		wakturtc(bufmenit, bufmenit_);
		wakturtc(bufdetik, bufdetik_);
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

void CAL()
{
    if (flagsim == 0)
    {
    	refalt = pressuretoalt(Pressure / 100);
    }
    flagsim = 0;
    flagstate = 0;
    flagrefalt = 0;
    flaggimbal = 1;
    flaginvalid = 0;
    counting  = 0;
    datatelemetri.packetcount = counting;
    datatelemetri.hsdeploy = 'N';
    datatelemetri.pcdeploy = 'N';
    strcpy(datatelemetri.state, "LAUNCH_WAIT");
    strcpy(datatelemetri.echocmd, "CAL");

    RESETSRAM();

    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, RESET);
    servogerak(&htim3, TIM_CHANNEL_3, 0);

    resetPosition(0);
    resetCumulativePosition(0);
    TIM1->CCR2 = 0;
    TIM1->CCR3 = 0;
}

void GB()
{
	isidata(4, commandbuff);
	if ((commandbuff[0] == 'O') && (commandbuff[1] == 'N'))
	{
		flaggimbal = 1;
	}
	else if ((commandbuff[0] == 'O') && (commandbuff[1] == 'F'))
	{
		flaggimbal = 0;
	}
	TM_BKPSRAM_Write8(FLAGGIMBAL_ADR, flaggimbal);
}

void HS()
{
	servogerak(&htim3, TIM_CHANNEL_3, 135);
}

void CAM()
{
	isidata(4, commandbuff);
	switch (commandbuff[0])
	{
		case '1':
			flagkameraon = 1;
			break;
		case 'O':
			flagkameraoff = 1;
			break;
	}
}

void IMU()
{
	flagcal = 0;
	TM_BKPSRAM_Write8(FLAGCAL_ADR, flagcal);
	CR();
}

void CR()
{
    NVIC_SystemReset();
}
