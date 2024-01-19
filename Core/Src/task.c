	/*
 * task.c
 *
 *  Created on: Dec 26, 2022
 *      Author: user
 *
 */
#include "main.h"
#include "task.h"
#include "kom.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include "BME280_STM32.h"
#include "math.h"
#include "bno055.h"
#include "GPS.h"
#include "servo.h"
#include "tm_stm32f4_bkpsram.h"

#define TEAM_ID			2032
#define PACKETCOUNT_ADR 0x00
#define REFALT_ADR		0x32
#define STATEIND_ADR 	0x96
#define FLAGTEL_ADR		0xC8
#define HSDEPLOY_ADR 	0xFA
#define PCDEPLOY_ADR 	0x12C

extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef hi2c2;
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim4;

extern uint8_t flagkamera1on;
extern uint8_t flagkamera2on;
extern uint8_t flagkameraoff;

uint8_t flagtel=0, flagsim = 0,flagstate = 0, valid = 0;
uint16_t counting = 1;
datatelemetri_t datatelemetri;
uint8_t check, csh;
uint8_t flagrefalt;
uint8_t flaginvalid;

bno055_calibration_state_t calStat;
bno055_calibration_data_t calData;
extern bno055_vector_t bno055_euler, bno055_gyro;

float Temperature, Pressure, Humidity, Spressure = 101325, refalt = 0, tempalt = 0;

lwgps_t gps;
char rxgps[128];

float gpslat,gpslong,gpsalt;
uint8_t gpssat;
char gpsdetik[3], gpsmenit[3], gpsjam[3];

char commandbuff[15];

extern char buffer[256];
extern uint8_t flagsimpan;

#define FILTER_SIZE 5
float readings[FILTER_SIZE];
int ind_ = 0;

#define RHO 1.204
#define ZERO_SPAN 80
#define VELOCITY_OFFSET_SIZE 20
int velocity_offset = 0;
int adc_avg;
uint32_t dataadc[2];

void init()
{
    strcpy(datatelemetri.state, "LAUNCH_WAIT");
    READRAM();
    servogerak(&htim4, TIM_CHANNEL_1, 0);
    datatelemetri.fmode = 'F';
    strcpy(datatelemetri.echocmd, "CXON");
    gpslat = 0.0000;
    gpslong = 0.0000;
    gpsalt = 0.0;
    gpssat = 0;
    strcpy(gpsjam, "00");
    strcpy(gpsmenit, "00");
    strcpy(gpsdetik, "00");
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
			strcpy(datatelemetri.state, "HS_DEPLOYED");
			break;
		case 3:
			strcpy(datatelemetri.state, "DESCENT");
			break;
		case 4:
			strcpy(datatelemetri.state, "PC_DEPLOYED");
			break;
		case 5:
			strcpy(datatelemetri.state, "LANDED");
			break;
    }
    refalt = TM_BKPSRAM_ReadFloat(REFALT_ADR);
    flagstate = TM_BKPSRAM_Read8(STATEIND_ADR);
    flagtel = TM_BKPSRAM_Read8(FLAGTEL_ADR);
    datatelemetri.hsdeploy = TM_BKPSRAM_Read8(HSDEPLOY_ADR);
    datatelemetri.pcdeploy = TM_BKPSRAM_Read8(PCDEPLOY_ADR);
}

void RESETSRAM()
{
    TM_BKPSRAM_Write16(PACKETCOUNT_ADR, counting);
    TM_BKPSRAM_WriteFloat(REFALT_ADR, refalt);
    TM_BKPSRAM_Write8(STATEIND_ADR, flagstate);
    TM_BKPSRAM_Write8(HSDEPLOY_ADR, datatelemetri.hsdeploy);
    TM_BKPSRAM_Write8(PCDEPLOY_ADR, datatelemetri.pcdeploy);
}

void cal_airspeed()
{
    uint16_t adc_data = 0;
    HAL_ADC_Start(&hadc1);
    for (int i = 0; i < VELOCITY_OFFSET_SIZE; ++i)
    {
		HAL_ADC_PollForConversion(&hadc1, 100);
		adc_data = HAL_ADC_GetValue(&hadc1);
		velocity_offset += adc_data - 2048;
    }
    HAL_ADC_Stop(&hadc1);
    velocity_offset /= VELOCITY_OFFSET_SIZE;
}

void ADC_measure()
{
    // Battery voltage
    float sum = 0;
    readings[ind_] = ((3.3/4095) * dataadc[0]) * (14.7/4.7);
    ind_ = (ind_ + 1) % FILTER_SIZE;
    for (int i = 0; i < FILTER_SIZE; i++)
    {
		sum += readings[i];
		adc_avg += dataadc[1] - velocity_offset;
    }
    datatelemetri.voltage =  sum / FILTER_SIZE;

    // Air speed
    adc_avg /= FILTER_SIZE;

    if (adc_avg >= (2048 - ZERO_SPAN) && adc_avg <= (2048 + ZERO_SPAN))
    {
    	datatelemetri.airspeed = 0;
    }
    else
    {
		if (adc_avg < 2048 - ZERO_SPAN)
		{
			datatelemetri.airspeed = sqrtf((-10000.0 * ((adc_avg / 4095.0) - 0.5)) / RHO);
		}
		else
		{
			datatelemetri.airspeed = sqrtf((10000.0 * ((adc_avg / 4095.0) - 0.5)) / RHO);
		}
    }
}

uint8_t buatcs(char dat_[])
{
    uint8_t hasil = 0, temp = 0;
    uint16_t buffhasil = 0;
    for (int i = 0; i < 150; i++)
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
    	hasil = 44330 * (1 - pow((press/1013.25), (1/5.255)));
    }
    else
    {
    	hasil = 0;
    }
    return hasil;
}

void bno055_init()
{
    bno055_assignI2C(&hi2c2);
    bno055_setup();
    bno055_setOperationModeNDOF();

    for (;;)
    {
		calStat = bno055_getCalibrationState();

		if (calStat.gyro == 3)
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
		else
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);

		if (calStat.accel == 3)
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
		else
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);

		if (calStat.mag == 3)
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, SET);
		else
			HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);

		if (calStat.sys == 3)
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, SET);
		else
			HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, RESET);

		if (calStat.gyro == 3 && calStat.mag == 3 && calStat.accel == 3)
		{
			calData = bno055_getCalibrationData();
			bno055_setCalibrationData(calData);
			break;
		}
    }

    HAL_Delay(500);
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, RESET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, RESET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, RESET);
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, RESET);
    HAL_Delay(500);

    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);
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

    sprintf(datatelemetri.telemetribuff,"%d,%c%c:%c%c:%c%c,%d,%c,%s,%.1f,%.2f,%c,%c,%.1f,%.1f,%.1f,%c%c:%c%c:%c%c,%.1f,%.4f,%.4f,%d,%.2f,%.2f,%.1f,%s,,%.1f",
    		TEAM_ID, datatelemetri.jam[0], datatelemetri.jam[1], datatelemetri.menit[0], datatelemetri.menit[1], datatelemetri.detik[0], datatelemetri.detik[1],
			datatelemetri.packetcount, datatelemetri.fmode, datatelemetri.state, datatelemetri.alt, datatelemetri.airspeed, datatelemetri.hsdeploy, datatelemetri.pcdeploy,
			datatelemetri.temp, datatelemetri.barpress, datatelemetri.voltage, gpsjam[0], gpsjam[1], gpsmenit[0], gpsmenit[1], gpsdetik[0], gpsdetik[1],
			gpsalt, gpslat, gpslong, gpssat, datatelemetri.tilt_x, datatelemetri.tilt_y, datatelemetri.rot_z, datatelemetri.echocmd, datatelemetri.heading);
    csh = ~buatcs(datatelemetri.telemetribuff);
    sprintf(datatelemetri.telemetritotal,"%s,%d\r\n", datatelemetri.telemetribuff, csh);
}

void kirimdata()
{
    HAL_UART_Transmit_DMA(&huart3, (uint8_t *)datatelemetri.telemetritotal, strnlen(datatelemetri.telemetritotal, sizeof(datatelemetri.telemetritotal)));
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, SET);
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
    counting  = 0;
    datatelemetri.packetcount = counting;
    flagsim = 0;
    flagstate = 0;
    flagrefalt = 0;
    flagkamera1on = 1;
    datatelemetri.hsdeploy = 'N';
    datatelemetri.pcdeploy = 'N';
    flaginvalid = 0;
    servogerak(&htim4, TIM_CHANNEL_1, 0);
    strcpy(datatelemetri.state, "LAUNCH_WAIT");
    RESETSRAM();
    HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, RESET);
    strcpy(datatelemetri.echocmd, "CAL");
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
    else if (datatelemetri.alt >= 720 && datatelemetri.alt < 730 && flagstate == 1)
    {
		strcpy(datatelemetri.state, "ROCKET_SEPARATION");
		flagstate = 2;
		flagkamera2on = 1;
		TM_BKPSRAM_Write8(STATEIND_ADR,1);
    }
    else if (datatelemetri.alt > 710 && flagstate == 2)
    {
		strcpy(datatelemetri.state, "HS_DEPLOYED");
		datatelemetri.hsdeploy = 'P';
		datatelemetri.pcdeploy 	= 'N';
		TM_BKPSRAM_Write8(HSDEPLOY_ADR,datatelemetri.hsdeploy);
		TM_BKPSRAM_Write8(PCDEPLOY_ADR,datatelemetri.pcdeploy);
		flagstate = 3;
		TM_BKPSRAM_Write8(STATEIND_ADR,2);
    }
    else if ((datatelemetri.alt - tempalt) < 0 && flagstate == 3)
    {
		TM_BKPSRAM_Write8(STATEIND_ADR,3);
		valid++;
		if (valid > 4)
		{
			strcpy(datatelemetri.state, "DESCENT");
			flagstate = 4;
			valid = 0;
		}
    }
    else if (datatelemetri.alt <= 120 && flagstate == 4 && !flaginvalid)
    {
		TM_BKPSRAM_Write8(STATEIND_ADR,4);
		strcpy(datatelemetri.state, "PC_DEPLOYED");
		flagstate = 5;
		datatelemetri.pcdeploy = 'C';
		datatelemetri.hsdeploy = 'P';
		TM_BKPSRAM_Write8(HSDEPLOY_ADR,datatelemetri.hsdeploy);
		TM_BKPSRAM_Write8(PCDEPLOY_ADR,datatelemetri.pcdeploy);
		servogerak(&htim4, TIM_CHANNEL_1, 135);
    }
    else if (datatelemetri.alt < 13 && flagstate == 5 && !flaginvalid)
    {
		TM_BKPSRAM_Write8(STATEIND_ADR,5);
		strcpy(datatelemetri.state, "LANDED");
		flagstate = 6;
		datatelemetri.pcdeploy = 'C';
		datatelemetri.hsdeploy = 'P';
		TM_BKPSRAM_Write8(HSDEPLOY_ADR,datatelemetri.hsdeploy);
		TM_BKPSRAM_Write8(PCDEPLOY_ADR,datatelemetri.pcdeploy);
		flagkameraoff = 1;
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, SET);
    }
}

void CR()
{
    HAL_NVIC_SystemReset();
}

void adcinit()
{
    cal_airspeed();
    HAL_ADC_Start_DMA(&hadc1, dataadc, 2);
}
