/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include "BME280_STM32.h"
#include "bno055.h"
#include "GPS.h"
#include "kom.h"
#include "servo.h"
#include "user_task.h"
#include "tm_stm32f4_bkpsram.h"
#include "as5600.h"
#include "pid.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
state_t cansatState;
cam_t camera;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define SAMPLE_TIME_GIMBAL_MS 		1
#define SAMPLE_TIME_EULER_MS 		10
#define SAMPLE_TIME_GYRO_MS 		10
#define SAMPLE_TIME_BARO_MS 		40
#define SAMPLE_TIME_RTC_MS 			1000
#define SAMPLE_TIME_DATA_MS			1000
#define SAMPLE_TIME_TELEMETRY_MS	1000

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime7,
};
/* Definitions for gimbalTask */
osThreadId_t gimbalTaskHandle;
const osThreadAttr_t gimbalTask_attributes = {
  .name = "gimbalTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for gyroscopeTask */
osThreadId_t gyroscopeTaskHandle;
const osThreadAttr_t gyroscopeTask_attributes = {
  .name = "gyroscopeTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for eulerTask */
osThreadId_t eulerTaskHandle;
const osThreadAttr_t eulerTask_attributes = {
  .name = "eulerTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for rtcTask */
osThreadId_t rtcTaskHandle;
const osThreadAttr_t rtcTask_attributes = {
  .name = "rtcTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for barometerTask */
osThreadId_t barometerTaskHandle;
const osThreadAttr_t barometerTask_attributes = {
  .name = "barometerTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for telemetryTask */
osThreadId_t telemetryTaskHandle;
const osThreadAttr_t telemetryTask_attributes = {
  .name = "telemetryTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime5,
};
/* Definitions for cameraTask */
osThreadId_t cameraTaskHandle;
const osThreadAttr_t cameraTask_attributes = {
  .name = "cameraTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for calibrationTask */
osThreadId_t calibrationTaskHandle;
const osThreadAttr_t calibrationTask_attributes = {
  .name = "calibrationTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityRealtime6,
};
/* Definitions for adcTask */
osThreadId_t adcTaskHandle;
const osThreadAttr_t adcTask_attributes = {
  .name = "adcTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for gpsTask */
osThreadId_t gpsTaskHandle;
const osThreadAttr_t gpsTask_attributes = {
  .name = "gpsTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for getdataTask */
osThreadId_t getdataTaskHandle;
const osThreadAttr_t getdataTask_attributes = {
  .name = "getdataTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for telemetrySemaphore */
osSemaphoreId_t telemetrySemaphoreHandle;
const osSemaphoreAttr_t telemetrySemaphore_attributes = {
  .name = "telemetrySemaphore"
};
/* Definitions for gimbalSemaphore */
osSemaphoreId_t gimbalSemaphoreHandle;
const osSemaphoreAttr_t gimbalSemaphore_attributes = {
  .name = "gimbalSemaphore"
};
/* USER CODE BEGIN PV */

FATFS fs;
FIL fil;
FILINFO fno;
FRESULT fresult;
UINT br, bw;

extern float Temperature, Pressure, Humidity, Spressure, refalt, tempalt;

extern lwgps_t gps;
extern char rxgps[128];
extern float gpslat,gpslong,gpsalt;
extern uint8_t gpssat;
extern char gpsdetik[3], gpsmenit[3], gpsjam[3];

extern datatelemetri_t datatelemetri;
extern uint16_t counting;
extern uint8_t flagtel, flagsim;

bno055_calibration_data_t bno055_calData;
bno055_calibration_state_t bno055_calStat;
bno055_vector_t bno055_euler, bno055_gyro;

PID_TypeDef _PID;
volatile int CountENC = 0;
int Rev = 0;
int Current_Angle = 0;
int CW, CCW;
double output = 0, Setpoint = 0, jarak_min = 0, input = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_RTC_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM1_Init(void);
void StartDefaultTask(void *argument);
void StartGimbalTask(void *argument);
void StartGyroscopeTask(void *argument);
void StartEulerTask(void *argument);
void StartRTCTask(void *argument);
void StartBarometerTask(void *argument);
void StartTelemetryTask(void *argument);
void StartCameraTask(void *argument);
void StartCalibrationTask(void *argument);
void StartADCTask(void *argument);
void StartGPSTask(void *argument);
void StartGetDataTask(void *argument);

/* USER CODE BEGIN PFP */
int map(int value, int from_low, int from_high, int to_low, int to_high)
{
    return ((value - from_low) * (to_high - to_low)) / (from_high - from_low) + to_low;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_RTC_Init();
  MX_SDIO_SD_Init();
  MX_USART3_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM10_Init();
  MX_FATFS_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of telemetrySemaphore */
  telemetrySemaphoreHandle = osSemaphoreNew(1, 0, &telemetrySemaphore_attributes);

  /* creation of gimbalSemaphore */
  gimbalSemaphoreHandle = osSemaphoreNew(1, 0, &gimbalSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

//  /* creation of gimbalTask */
//  gimbalTaskHandle = osThreadNew(StartGimbalTask, NULL, &gimbalTask_attributes);
//
//  /* creation of gyroscopeTask */
//  gyroscopeTaskHandle = osThreadNew(StartGyroscopeTask, NULL, &gyroscopeTask_attributes);
//
//  /* creation of eulerTask */
//  eulerTaskHandle = osThreadNew(StartEulerTask, NULL, &eulerTask_attributes);
//
//  /* creation of rtcTask */
//  rtcTaskHandle = osThreadNew(StartRTCTask, NULL, &rtcTask_attributes);
//
//  /* creation of barometerTask */
//  barometerTaskHandle = osThreadNew(StartBarometerTask, NULL, &barometerTask_attributes);
//
//  /* creation of telemetryTask */
//  telemetryTaskHandle = osThreadNew(StartTelemetryTask, NULL, &telemetryTask_attributes);
//
//  /* creation of cameraTask */
//  cameraTaskHandle = osThreadNew(StartCameraTask, NULL, &cameraTask_attributes);
//
//  /* creation of calibrationTask */
//  calibrationTaskHandle = osThreadNew(StartCalibrationTask, NULL, &calibrationTask_attributes);
//
//  /* creation of adcTask */
//  adcTaskHandle = osThreadNew(StartADCTask, NULL, &adcTask_attributes);
//
//  /* creation of gpsTask */
//  gpsTaskHandle = osThreadNew(StartGPSTask, NULL, &gpsTask_attributes);
//
//  /* creation of getdataTask */
//  getdataTaskHandle = osThreadNew(StartGetDataTask, NULL, &getdataTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 12;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 6588-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 255;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 20000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 16800-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 10000-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 19200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED4_Pin|LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BUZZER_Pin|CAM1_Pin|CAM2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED4_Pin LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED4_Pin|LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZER_Pin CAM1_Pin CAM2_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|CAM1_Pin|CAM2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	osThreadFlagsSet(adcTaskHandle, 1);
//	ADC_measure();
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2)
	{
		osThreadFlagsSet(gpsTaskHandle, 1);
//		parsegpsdata();
	}

	if (huart == &huart3)
	{
		checkdata_();
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	TM_BKPSRAM_Init();
	init();
	rtcbackup();

	if (f_mount(&fs, (const TCHAR *)SDPath, 1) == FR_OK)
	{
		if (f_stat("2032.txt", &fno) != FR_OK)
		{
			f_open(&fil, "2032.txt", FA_CREATE_ALWAYS | FA_READ | FA_WRITE);
			f_close(&fil);
		}
	}
	else
	{
		Error_Handler();
	}

	bno055_init();
	bno055_setCalibrationData(bno055_calData);

	if (BME280_Config(OSRS_2, OSRS_16, OSRS_OFF, MODE_NORMAL, T_SB_0p5, IIR_16) == 0)
	{
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
	}
	else
	{
		Error_Handler();
	}

#ifdef USE_SERVO_GIMBAL
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
#else /* USE_SERVO_GIMBAL */
	PID(&_PID, &input, &output, &Setpoint, 11, 0, 0, _PID_P_ON_E, _PID_CD_DIRECT);
	PID_SetMode(&_PID, _PID_MODE_AUTOMATIC);
	PID_SetSampleTime(&_PID, 1);
	PID_SetOutputLimits(&_PID, -255, 255);
	resetPosition(0);
	resetCumulativePosition(0);
	TIM1->CCR2 = 0;
	TIM1->CCR3 = 0;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
#endif /* USE_SERVO_GIMBAL */

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

	/* creation of calibrationTask */
	calibrationTaskHandle = osThreadNew(StartCalibrationTask, NULL, &calibrationTask_attributes);

	/* creation of telemetryTask */
	telemetryTaskHandle = osThreadNew(StartTelemetryTask, NULL, &telemetryTask_attributes);

	/* creation of eulerTask */
	eulerTaskHandle = osThreadNew(StartEulerTask, NULL, &eulerTask_attributes);

	/* creation of gyroscopeTask */
	gyroscopeTaskHandle = osThreadNew(StartGyroscopeTask, NULL, &gyroscopeTask_attributes);

	/* creation of barometerTask */
	barometerTaskHandle = osThreadNew(StartBarometerTask, NULL, &barometerTask_attributes);

	/* creation of gimbalTask */
	gimbalTaskHandle = osThreadNew(StartGimbalTask, NULL, &gimbalTask_attributes);

	/* creation of getdataTask */
	getdataTaskHandle = osThreadNew(StartGetDataTask, NULL, &getdataTask_attributes);

	/* creation of rtcTask */
	rtcTaskHandle = osThreadNew(StartRTCTask, NULL, &rtcTask_attributes);

	/* creation of cameraTask */
	cameraTaskHandle = osThreadNew(StartCameraTask, NULL, &cameraTask_attributes);

	/* creation of adcTask */
	adcTaskHandle = osThreadNew(StartADCTask, NULL, &adcTask_attributes);

	/* creation of gpsTask */
	gpsTaskHandle = osThreadNew(StartGPSTask, NULL, &gpsTask_attributes);

	if (flagtel)
	{
		osSemaphoreRelease(telemetrySemaphoreHandle);
	}

	adcinit();
	gpsinit();
	kominit();

	osThreadTerminate(defaultTaskHandle);
  /* Infinite loop */
  for(;;)
  {
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartGimbalTask */
/**
* @brief Function implementing the gimbalTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGimbalTask */
void StartGimbalTask(void *argument)
{
  /* USER CODE BEGIN StartGimbalTask */
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreAcquire(gimbalSemaphoreHandle, osWaitForever);

#ifdef USE_SERVO_GIMBAL
	  if (bno055_euler.x >= 0 && bno055_euler.x < 180)
	  {
		  servogerak(&htim3, TIM_CHANNEL_1, bno055_euler.x);
		  servogerak(&htim3, TIM_CHANNEL_3, 0);
	  }
	  else if (bno055_euler.x >= 180 && bno055_euler.x <= 360)
	  {
		  servogerak(&htim3, TIM_CHANNEL_1, 180);
		  servogerak(&htim3, TIM_CHANNEL_3, bno055_euler.x - 180);
	  }
#else /* USE_SERVO_GIMBAL */
	  CountENC = getCumulativePosition();
	  Rev = CountENC % 4095;
	  Current_Angle = map(Rev, 0, 4095, 0, 359);
	  CW = ((int)bno055_euler.x - Current_Angle+360) % 360;
	  CCW = ((Current_Angle - (int)bno055_euler.x+360) % 360);

	  if (CW < CCW)
		  jarak_min = CW;
	  else
		  jarak_min = -CCW;

	  input = jarak_min;
	  PID_Compute(&_PID);

	  if (output > 0)
		  TIM1->CCR3 = (uint32_t)output;
	  else
		  TIM1->CCR2 = (uint32_t)abs(output);
#endif /* USE_SERVO_GIMBAL */

	  osSemaphoreRelease(gimbalSemaphoreHandle);

	  osDelay(SAMPLE_TIME_GIMBAL_MS);
  }
  /* USER CODE END StartGimbalTask */
}

/* USER CODE BEGIN Header_StartGyroscopeTask */
/**
* @brief Function implementing the gyroscopeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGyroscopeTask */
void StartGyroscopeTask(void *argument)
{
  /* USER CODE BEGIN StartGyroscopeTask */
  /* Infinite loop */
  for(;;)
  {
	  datatelemetri.rot_z = bno055_getRotationZ();

	  osDelay(SAMPLE_TIME_GYRO_MS);
  }
  /* USER CODE END StartGyroscopeTask */
}

/* USER CODE BEGIN Header_StartEulerTask */
/**
* @brief Function implementing the eulerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEulerTask */
void StartEulerTask(void *argument)
{
  /* USER CODE BEGIN StartEulerTask */
  /* Infinite loop */
  for(;;)
  {
	  bno055_euler = bno055_getEuler();
	  datatelemetri.heading = bno055_euler.x;
	  datatelemetri.tilt_x = bno055_euler.y;
	  datatelemetri.tilt_y = bno055_euler.z;

	  osDelay(SAMPLE_TIME_EULER_MS);
  }
  /* USER CODE END StartEulerTask */
}

/* USER CODE BEGIN Header_StartRTCTask */
/**
* @brief Function implementing the rtcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRTCTask */
void StartRTCTask(void *argument)
{
  /* USER CODE BEGIN StartRTCTask */
  /* Infinite loop */
  for(;;)
  {
	  RTC_TimeTypeDef gTime;
	  RTC_DateTypeDef gDate;

	  HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);

	  wakturtc(gTime.Hours,datatelemetri.jam);
	  wakturtc(gTime.Minutes,datatelemetri.menit);
	  wakturtc(gTime.Seconds,datatelemetri.detik);

	  osDelay(SAMPLE_TIME_RTC_MS);
  }
  /* USER CODE END StartRTCTask */
}

/* USER CODE BEGIN Header_StartBarometerTask */
/**
* @brief Function implementing the barometerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBarometerTask */
void StartBarometerTask(void *argument)
{
  /* USER CODE BEGIN StartBarometerTask */
  /* Infinite loop */
  for(;;)
  {
	  BME280_Measure();
	  datatelemetri.temp = Temperature;
	  switch (flagsim)
	  {
		  case 2:
			  datatelemetri.alt = pressuretoalt(Spressure / 100);
			  datatelemetri.barpress = Spressure / 1000;
			  datatelemetri.alt -= refalt;
			  break;
		  default:
			  datatelemetri.alt = pressuretoalt(Pressure / 100);
			  datatelemetri.barpress = Pressure / 1000;
			  datatelemetri.alt -= refalt;
			  break;
	  }
	  if (datatelemetri.alt < 0)
	  {
		  datatelemetri.alt = 0;
	  }
	  state();

	  osDelay(SAMPLE_TIME_BARO_MS);
  }
  /* USER CODE END StartBarometerTask */
}

/* USER CODE BEGIN Header_StartTelemetryTask */
/**
* @brief Function implementing the telemetryTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTelemetryTask */
void StartTelemetryTask(void *argument)
{
  /* USER CODE BEGIN StartTelemetryTask */
  /* Infinite loop */
  for(;;)
  {
	  osDelay(SAMPLE_TIME_TELEMETRY_MS);

	  if (f_mount(&fs, (const TCHAR *)SDPath, 1) == FR_OK)
	  {
		  f_open(&fil, "2032.txt", FA_OPEN_APPEND | FA_WRITE);
		  f_write(&fil, datatelemetri.telemetritotal, strlen(datatelemetri.telemetritotal), &bw);
		  f_close(&fil);
	  }

	  osSemaphoreAcquire(telemetrySemaphoreHandle, osWaitForever);

	  counting++;
	  TM_BKPSRAM_Write16(PACKETCOUNT_ADR, counting);
	  HAL_UART_Transmit_DMA(&huart3, (uint8_t *)datatelemetri.telemetritotal, strlen(datatelemetri.telemetritotal));
	  HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);

	  osSemaphoreRelease(telemetrySemaphoreHandle);
  }
  /* USER CODE END StartTelemetryTask */
}

/* USER CODE BEGIN Header_StartCameraTask */
/**
* @brief Function implementing the cameraTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCameraTask */
void StartCameraTask(void *argument)
{
  /* USER CODE BEGIN StartCameraTask */
  /* Infinite loop */
  for(;;)
  {
	  osThreadFlagsWait(1, osFlagsWaitAny, osWaitForever);

	  switch (camera)
	  {
	  	  case MAIN_CAM:
			  HAL_GPIO_WritePin(CAM1_GPIO_Port, CAM1_Pin, SET);
			  osDelay(4000);
			  HAL_GPIO_WritePin(CAM1_GPIO_Port, CAM1_Pin, RESET);
			  osDelay(2000);
			  HAL_GPIO_WritePin(CAM1_GPIO_Port, CAM1_Pin, SET);
			  osDelay(1000);
			  HAL_GPIO_WritePin(CAM1_GPIO_Port, CAM1_Pin, RESET);
			  break;

	  	  case BONUS_CAM:
			  HAL_GPIO_WritePin(CAM2_GPIO_Port, CAM2_Pin, SET);
			  osDelay(4000);
			  HAL_GPIO_WritePin(CAM2_GPIO_Port, CAM2_Pin, RESET);
			  osDelay(2000);
			  HAL_GPIO_WritePin(CAM2_GPIO_Port, CAM2_Pin, SET);
			  osDelay(1000);
			  HAL_GPIO_WritePin(CAM2_GPIO_Port, CAM2_Pin, RESET);
			  break;

	  	  case CAM_OFF:
			  HAL_GPIO_WritePin(CAM1_GPIO_Port, CAM1_Pin | CAM2_Pin, SET);
			  osDelay(1000);
			  HAL_GPIO_WritePin(CAM1_GPIO_Port, CAM1_Pin | CAM2_Pin, RESET);
			  osDelay(750);
			  HAL_GPIO_WritePin(CAM1_GPIO_Port, CAM1_Pin | CAM2_Pin, SET);
			  osDelay(5000);
			  HAL_GPIO_WritePin(CAM1_GPIO_Port, CAM1_Pin | CAM2_Pin, RESET);
			  break;
	  }
  }
  /* USER CODE END StartCameraTask */
}

/* USER CODE BEGIN Header_StartCalibrationTask */
/**
* @brief Function implementing the calibrationTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCalibrationTask */
void StartCalibrationTask(void *argument)
{
  /* USER CODE BEGIN StartCalibrationTask */
  /* Infinite loop */
  for(;;)
  {
	  osThreadFlagsWait(1, osFlagsWaitAny, osWaitForever);
	  osKernelLock();

	  for (;;)
	  {
		  bno055_calStat = bno055_getCalibrationState();

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

		  if (bno055_calStat.gyro == 3 && bno055_calStat.mag == 3 && bno055_calStat.accel == 3)
		  {
			  bno055_calData = bno055_getCalibrationData();
			  TM_BKPSRAM_WriteCalData(BNO055CAL_ADR, bno055_calData);
			  bno055_setCalibrationData(bno055_calData);
			  HAL_Delay(500);
			  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin | LED2_Pin | LED3_Pin | LED4_Pin, RESET);
			  HAL_Delay(500);
			  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, SET);

			  uint8_t chip_id;
			  HAL_I2C_Mem_Read(&hi2c3, BME280_ADDRESS, ID_REG, 1, &chip_id, 1, HAL_MAX_DELAY);
			  if (chip_id == 0x60)
			  {
				  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, SET);
			  }
			  break;
		  }
	  }

	  osKernelUnlock();
  }
  /* USER CODE END StartCalibrationTask */
}

/* USER CODE BEGIN Header_StartADCTask */
/**
* @brief Function implementing the adcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartADCTask */
void StartADCTask(void *argument)
{
  /* USER CODE BEGIN StartADCTask */
  /* Infinite loop */
  for(;;)
  {
	  osThreadFlagsWait(1, osFlagsWaitAny, osWaitForever);
	  ADC_measure();
	  osDelay(250);
  }
  /* USER CODE END StartADCTask */
}

/* USER CODE BEGIN Header_StartGPSTask */
/**
* @brief Function implementing the gpsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGPSTask */
void StartGPSTask(void *argument)
{
  /* USER CODE BEGIN StartGPSTask */
  /* Infinite loop */
  for(;;)
  {
	  osThreadFlagsWait(1, osFlagsWaitAny, osWaitForever);
	  parsegpsdata();
  }
  /* USER CODE END StartGPSTask */
}

/* USER CODE BEGIN Header_StartGetDataTask */
/**
* @brief Function implementing the getdataTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGetDataTask */
void StartGetDataTask(void *argument)
{
  /* USER CODE BEGIN StartGetDataTask */
  /* Infinite loop */
  for(;;)
  {
	  tempalt = datatelemetri.alt;
	  datatelemetri.packetcount = counting;
	  sprintf(datatelemetri.telemetribuff, "2032,%c%c:%c%c:%c%c,%d,%c,%s,%.1f,%.2f,%c,%c,%.1f,%.1f,%.1f,%c%c:%c%c:%c%c,%.1f,%.4f,%.4f,%d,%.2f,%.2f,%.1f,%s,,%.1f,",
			  datatelemetri.jam[0], datatelemetri.jam[1], datatelemetri.menit[0], datatelemetri.menit[1], datatelemetri.detik[0], datatelemetri.detik[1],
			  datatelemetri.packetcount, datatelemetri.fmode, datatelemetri.state, datatelemetri.alt, datatelemetri.airspeed, datatelemetri.hsdeploy, datatelemetri.pcdeploy,
			  datatelemetri.temp, datatelemetri.voltage, datatelemetri.barpress, gpsjam[0], gpsjam[1], gpsmenit[0], gpsmenit[1], gpsdetik[0], gpsdetik[1],
			  gpsalt, gpslat, gpslong, gpssat, datatelemetri.tilt_x, datatelemetri.tilt_y, datatelemetri.rot_z, datatelemetri.echocmd, datatelemetri.heading);
	  uint8_t csh = ~buatcs(datatelemetri.telemetribuff);
	  sprintf(datatelemetri.telemetritotal,"%s%d\r\n", datatelemetri.telemetribuff, csh);

	  osDelay(SAMPLE_TIME_DATA_MS);
  }
  /* USER CODE END StartGetDataTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
