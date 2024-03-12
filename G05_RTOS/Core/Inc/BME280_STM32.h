#ifndef INC_BME280_STM32_H_
#define INC_BME280_STM32_H_

#include "stm32f4xx_hal.h"
#ifdef FREERTOS_ENABLED
#include "cmsis_os.h"
#endif


/* Konfigurasi untuk BME280
 *
 * @osrs adalah oversampling untuk meningkatkan akurasi
 *       jika osrs diatur ke OSRS_OFF, pengukuran yang sesuai akan diabaikan
 *       Dapat diatur ke OSRS_1, OSRS_2, OSRS_4, dll. Periksa file header
 *
 * @mode dapat digunakan untuk mengatur mode untuk perangkat
 *       MODE_SLEEP akan memasukkan perangkat ke dalam mode tidur
 *       MODE_FORCED perangkat kembali ke mode tidur setelah satu pengukuran. Anda perlu menggunakan fungsi BME280_WakeUP() sebelum setiap pengukuran
 *       MODE_NORMAL perangkat melakukan pengukuran dalam mode normal. Periksa datasheet halaman no 16
 *
 * @t_sb adalah waktu standby. Waktu sensor menunggu sebelum melakukan pengukuran lain
 *       Digunakan bersama dengan mode normal. Periksa datasheet halaman no 16 dan halaman no 30
 *
 * @filter adalah koefisien filter IIR
 *         IIR digunakan untuk menghindari fluktuasi jangka pendek
 *         Periksa datasheet halaman no 18 dan halaman no 30
 */

void BME280_Delay(int time);

int BME280_Config (uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h, uint8_t mode, uint8_t t_sb, uint8_t filter);


// Baca parameter Trimming yang disimpan di NVM ROM perangkat
void TrimRead(void);

/* Mengukur suhu, tekanan, dan kelembaban
 * Nilai akan disimpan dalam parameter yang diteruskan ke fungsi
 */
void BME280_Measure (void);


// Oversampling definitions
#define OSRS_OFF    	0x00
#define OSRS_1      	0x01
#define OSRS_2      	0x02
#define OSRS_4      	0x03
#define OSRS_8      	0x04
#define OSRS_16     	0x05

// MODE Definitions
#define MODE_SLEEP      0x00
#define MODE_FORCED     0x01
#define MODE_NORMAL     0x03

// Standby Time
#define T_SB_0p5    	0x00
#define T_SB_62p5   	0x01
#define T_SB_125    	0x02
#define T_SB_250    	0x03
#define T_SB_500    	0x04
#define T_SB_1000   	0x05
#define T_SB_10     	0x06
#define T_SB_20     	0x07

// IIR Filter Coefficients
#define IIR_OFF     	0x00
#define IIR_2       	0x01
#define IIR_4       	0x02
#define IIR_8       	0x03
#define IIR_16      	0x04


// REGISTERS DEFINITIONS
#define ID_REG      	0xD0
#define RESET_REG  		0xE0
#define CTRL_HUM_REG    0xF2
#define STATUS_REG      0xF3
#define CTRL_MEAS_REG   0xF4
#define CONFIG_REG      0xF5
#define PRESS_MSB_REG   0xF7


#endif /* INC_BME280_STM32_H_ */
