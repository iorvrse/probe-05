/*
 * task.h
 *
 *  Created on: Dec 26, 2022
 *      Author: user
 */

#ifndef INC_TASK_H_
#define INC_TASK_H_

#include "stm32f407xx.h"

void rtcbackup();
void Settime(uint8_t jam_, uint8_t menit_, uint8_t detik_);
void CX();
void BCN();
void ST();
void SIM();
void SIMP();
void get_time();
void CAL();
void kirimdata();
void wakturtc(uint8_t timebuff, char datat[]);
void ambildata();
void init();
uint8_t buatcs(char dat_[]);
float pressuretoalt(float press);
void bno055_init();
void state();
void cal_air_speed();
void ADC_measure();
void adcinit();
void READRAM();
void RESETSRAM();
void CR();

typedef struct {
	char telemetribuff[170];
	char telemetritotal[175];

	char jam[2];
	char menit[2];
	char detik[2];

	uint16_t packetcount;

	char fmode;
	char state[20];

	float alt;
	float barpress;

	char hsdeploy;
	char pcdeploy;

	float temp;
	float voltage;
	float airspeed;

	float heading;
	float tilt_x;
	float tilt_y;
	float rot_z;

	char echocmd[15];
} datatelemetri_t;

#endif /* INC_TASK_H_ */
