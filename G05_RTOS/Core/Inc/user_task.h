#ifndef INC_TASK_H_
#define INC_TASK_H_

#include "stm32f407xx.h"
#include "stm32f4xx_hal.h"

void init();
void adcinit();
void bno055_init();
void READRAM();
void RESETSRAM();
void gpsinit();
void parsegpsdata();
void wakturtc(uint8_t timebuff, char datat[]);
void Settime(uint8_t jam_, uint8_t menit_, uint8_t detik_);
void get_time();
void rtcbackup();
void ADC_measure();
uint8_t buatcs(char dat_[]);
float pressuretoalt(float press);
void CX();
void BCN();
void ST();
void SIM();
void SIMP();
void CAL();
void GB();
void HS();
void CAM();
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
