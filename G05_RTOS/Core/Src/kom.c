#include "kom.h"
#include "user_task.h"
#include "cmsis_os.h"

uint8_t rxdata, datalkp[30], cmd = 0;
extern UART_HandleTypeDef huart3;
extern osThreadId_t calibrationTaskHandle;

void kominit(void)
{
	HAL_UART_Receive_DMA(&huart3, &rxdata, 1);
}

void isidata(uint8_t urutan,char dat_[])
{
	uint8_t n = 0, k = 0, m = 0;

	for (int c = 0; c < 28; c++)
	{
		dat_[c] = 0;
	}

	while (n < urutan)
	{
		if (datalkp[m] == '\r') n = urutan;
		if (datalkp[m] == ',') n++;
		if (n == urutan - 1 && datalkp[m] != ',')
		{
			dat_[k] = datalkp[m];
			k++;
		}
		m++;
	}
}

uint8_t cocokan(uint8_t urutan,char dat[])
{
	char buf[30];
	uint8_t hasil = 0;
	isidata(urutan, buf);
	uint8_t k = 0;

	while(buf[k] != '\0')
	{
		k++;
		if (buf[k] == dat[k]) hasil++;
	}
	return hasil;
}

void checkdata_(void)
{
	if (rxdata == 'C')
	{
		datalkp[cmd] = rxdata;
		cmd++;
	}
	else if (cmd > 0)
	{
		if (rxdata == ',')
		{
			datalkp[cmd] = ',';
			cmd++;
		}
		else if (rxdata == '\r')
		{
			datalkp[cmd] = '\r';
			cmd = 0;

			if ((cocokan(3, "CX") == 2) && (cocokan(2, "2032") == 4)) CX();
			else if ((cocokan(3, "SIMP") == 4) && (cocokan(2,"2032") == 4)) SIMP();
			else if ((cocokan(3, "SIM") == 3) && (cocokan(2,"2032") == 4)) SIM();
			else if ((cocokan(3, "IMU") == 3) && (cocokan(2,"2032") == 4)) osThreadFlagsSet(calibrationTaskHandle, 1);
			else if ((cocokan(3, "CAL") == 3) && (cocokan(2,"2032") == 4)) CAL();
			else if ((cocokan(3, "ST") == 2) && (cocokan(2,"2032") == 4)) ST();
			else if ((cocokan(3, "CR") == 2) && (cocokan(2,"2032") == 4)) CR();
			else if ((cocokan(3, "BCN") == 3) && (cocokan(2,"2032") == 4)) BCN();
			else if ((cocokan(3, "GB") == 2) && (cocokan(2,"2032") == 4)) GB();
			else if ((cocokan(3, "HS") == 2) && (cocokan(2,"2032") == 4)) HS();
			else if ((cocokan(3, "CAM") == 3) && (cocokan(2,"2032") == 4)) CAM();
		}
		else
		{
			datalkp[cmd] = rxdata;
			cmd++;
		}
	}
}
