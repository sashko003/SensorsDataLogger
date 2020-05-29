/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "data_logger.h"
#include "dht11_driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


typedef union data_log_s
{
	struct data_s
	{
		uint8_t sDataType[3]; // change on enum
		uint8_t bDataBuffer[sizeof(uint32_t)];
		uint8_t sTimestamp[9];
	} DATA_S;
	uint8_t bBuffer[16];
} DATA_LOG_S;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MIN_STACK_SIZE configMINIMAL_STACK_SIZE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* Definitions for DHT11TaskHandle */
osThreadId_t DHT11TaskHandle;
const osThreadAttr_t DHT11Task_attributes = {
  .name = "DHT11Task",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = MIN_STACK_SIZE * 4
};
/* Definitions for PotentiometerTaskHandle */
osThreadId_t PotentiometerTaskHandle;
const osThreadAttr_t PotentiometerTask_attributes = {
  .name = "PotentiometerTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = MIN_STACK_SIZE * 3
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_RTC_Init(void);
void readDHT11Data(void *argument);
void readPotentiometerData(void *argument);

/* USER CODE BEGIN PFP */
uint32_t used_mem(uint32_t address);

uint32_t flash_erase(uint32_t address);
uint32_t flash_copy_page(uint32_t src, uint32_t dst);
uint8_t define_page(uint32_t address);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


void set_time (void)
{
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;
  char chTime[8] = {0}, chDate[11] = {0},
	   chHours[2] = {0}, chMinutes[2] = {0}, chSeconds[2] = {0};
  memcpy(chTime, __TIME__, 8);
  memcpy(chDate, __DATE__, 11);
  memcpy(chHours, chTime, 2);
  memcpy(chMinutes, chTime+3, 2);
  memcpy(chSeconds, chTime+6, 2);

  sTime.Hours = (atoi(chHours) > 12) ? atoi(chHours) - 12 : atoi(chHours); // set hours
  sTime.Minutes = atoi(chMinutes); // set minutes
  sTime.Seconds = atoi(chSeconds); // set seconds
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    //_Error_Handler(__FILE__, __LINE__);
	  return;
  }
  sDate.WeekDay = RTC_WEEKDAY_TUESDAY; //  day
  sDate.Month = RTC_MONTH_MAY; // month
  sDate.Date = 0x5; // date
  sDate.Year = 0x16; // year
  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    //_Error_Handler(__FILE__, __LINE__);
	  return;
  }
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // backup register
}

void get_time(volatile char* time)
{
	//char time[10] = {0};
	//char date[10] = {0};
 RTC_DateTypeDef gDate;
 RTC_TimeTypeDef gTime;
/* Get the RTC current Time */
 HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
/* Get the RTC current Date */
 HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
/* Display time Format: hh:mm:ss */
// memcpy(time, &gTime.Hours, 2);
// memcpy(time + 2, &(":"), 1);
// memcpy(time, &gTime.Minutes, 2);
// memcpy(time + 2, &(":"), 1);
// memcpy(time, &gTime.Seconds, 2);
 sprintf((char*)time,"%02d:%02d:%02d",gTime.Hours, gTime.Minutes, gTime.Seconds);
 int len = strlen(time);
 len = sizeof(time);
/* Display date Format: dd-mm-yy */
 //sprintf((char*)date,"%02d-%02d-%2d",gDate.Date, gDate.Month, 2000 + gDate.Year);
}


uint32_t g_TicksToDelay = 0;
uint8_t u8Presence = 0;



uint8_t PendST = 0;
int32_t  OS_Tick_Enable (void) {
  if (PendST != 0U) {
    PendST = 0U;
    SCB->ICSR = SCB_ICSR_PENDSTSET_Msk;
  }
  SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;
  return (0);
}

int32_t  OS_Tick_Disable (void) {
  SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
  if ((SCB->ICSR & SCB_ICSR_PENDSTSET_Msk) != 0U) {
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
    PendST = 1U;
  }
  return (0);
}




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
  HAL_Delay(10000);
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  DHT11_SetTimer(&htim2);

  DATA_LOG_S TestTmp_S = {0}, TestHmd_S = {0};
  LOG_HEADER_S TestHeaders_S = {0};
  memcpy(TestTmp_S.bBuffer, LOGS_BEGIN, 16);
  memcpy(TestHmd_S.bBuffer, LOGS_BEGIN+16, 16);
  memcpy(TestHeaders_S.bBuffer, LOGGER_DATA, 16);
  FlashErase(FLASH_STORAGE);
  FlashErase(MIRROR_PAGE);
  LoggerInit();
  set_time();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, SET);
//  HAL_HAL_FLASH_Unlock(); HAL_FLASH_OB_Unlock();
//  uint32_t test = 0;
//  for(int i = 0; i < 10; ++i)
//	  test = (*((__IO uint8_t*)(LOGGER_DATA + i))) = i;
//  HAL_HAL_FLASH_Lock(); HAL_FLASH_OB_Lock();

  //NVIC_SetPriorityGrouping( NVIC_PriorityGroup_4 );
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* creation of DHT11TaskHandle */
  DHT11TaskHandle = osThreadNew(readDHT11Data, NULL, &DHT11Task_attributes);

  /* creation of PotentiometerTaskHandle */
  PotentiometerTaskHandle = osThreadNew(readPotentiometerData, NULL, &PotentiometerTask_attributes);

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//	  HAL_FLASH_Unlock(); HAL_FLASH_OB_Unlock();
//	  flash_write(FLASH_STORAGE, 0x11111111);
//	  flash_write(FLASH_STORAGE+4, 0x11111111);
//	  flash_write(FLASH_STORAGE+8, 0x11111111);
//	  flash_write(FLASH_STORAGE+12, 0x11111111);
//	  HAL_FLASH_Lock(); HAL_FLASH_OB_Lock();
	  readPotentiometerData(0);
	  readPotentiometerData(0);
	  readPotentiometerData(0);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  //readDHT11Data(0);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_RTC
                              |RCC_PERIPHCLK_TIM2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode 
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

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

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x28;
  sDate.Year = 0x20;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the TimeStamp 
  */
  if (HAL_RTCEx_SetTimeStamp(&hrtc, RTC_TIMESTAMPEDGE_RISING, RTC_TIMESTAMPPIN_DEFAULT) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart2.Init.BaudRate = 460800;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_readDHT11Data */
/**
  * @brief  Function implementing the readDHT11Data thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_readDHT11Data */
void readDHT11Data(void *argument)
{
  /* USER CODE BEGIN 5 */
	//static uint32_t u32Offset = 0;
#ifdef DEBUG
	char chMsg[30] = {0};
    char chData[5] = {0};
#endif
    const DHT11_DATA_S *pDHT11Data = 0;
	DATA_LOG_S TemperatureData_S = {0},
			   HumidityData_S = {0};
#ifdef DEBUG
	DATA_LOG_S Test_S = {0};
#endif
	memcpy(TemperatureData_S.DATA_S.sDataType,
		   "TMP",
		   3);
	memcpy(HumidityData_S.DATA_S.sDataType,
		   "HMD",
		   3);
  TickType_t xLastWakeTime;
  /* Infinite loop */
  for(;;)
  {
#ifdef RELEASE
	  uint8_t i = 0;
#endif
#ifdef DEBUG
	  HAL_UART_Transmit(&huart2, "NEW_LOOP\n\r", 11, HAL_MAX_DELAY);
	  memset(chMsg, 0, 30);
	  memset(chData, 0, 5);
	  /**/
	  sprintf(chMsg, "Response: %d\n\r", u8Presence);
	  HAL_UART_Transmit(&huart2, chMsg, strlen(chMsg), HAL_MAX_DELAY);
#endif
	  OS_Tick_Disable();
	  DHT11_Start();
	  u8Presence = DHT11_CheckResponse();
	  if(1 != u8Presence)
	  {
		  continue;
	  }
	  /**/

	  pDHT11Data = DHT11_ReadData();
	  if(0 == DHT11_IsDataValid())
	  {
		  continue;
	  }
//	  for(int i = 0; i<5; ++i)
//	  {
//	   	chData[i] = DHT11_Read();
//	  }
	  OS_Tick_Enable();

	  get_time(HumidityData_S.DATA_S.sTimestamp);
	  get_time(TemperatureData_S.DATA_S.sTimestamp);

	  memcpy(HumidityData_S.DATA_S.bDataBuffer,
	  	     (void*)pDHT11Data->u8Humidity,
	   	  	 sizeof(uint8_t));
	  memcpy(TemperatureData_S.DATA_S.bDataBuffer,
	  	     (void*)pDHT11Data->u8Temperature,
	  	  	 sizeof(uint8_t));

	  OS_Tick_Disable();
	  LoggerSaveData(TemperatureData_S.bBuffer, sizeof(DATA_LOG_S));
	  LoggerSaveData(HumidityData_S.bBuffer, sizeof(DATA_LOG_S));
	  // Initialise the xLastWakeTime variable with the current time.
	  xLastWakeTime = xTaskGetTickCount();


	  LoggerSaveState(UPDATED);
	  LOG_HEADER_S TestHeaders_S = {0};
	  memcpy(TestHeaders_S.bBuffer, LOGGER_DATA, 16);

      OS_Tick_Enable();
	  memset(HumidityData_S.DATA_S.bDataBuffer, 0, sizeof(uint32_t)+8);
	  memset(TemperatureData_S.DATA_S.bDataBuffer, 0, sizeof(uint32_t)+8);
//	  u8Presence = 0;
//	  memset(chMsg, 0, 20);
//	  uint64_t f = 0.0;
#ifdef DEBUG
	  uint16_t u16CheckSum = 0;
	  memcpy(chData, pDHT11Data, sizeof(DHT11_DATA_S));
	  for(int i = 0; i<4; ++i)
	  {
//	  	uint8_t value = chData[i];
//	  	uint8_t f_part = chData[i+1];
	  	u16CheckSum += chData[i];
//	  	u16CheckSum += chData[i+1];
//	  	sprintf(chMsg, "Data%d: %d,%d\n\r", i, value, f_part);
//	  	HAL_UART_Transmit(&huart2, chMsg, strlen(chMsg), HAL_MAX_DELAY);
	  }
//	  sprintf(chMsg, "Humidity: %d,%d\n\r", pDHT11Data->u8Humidity, pDHT11Data->u8HmdFloatPart);
//	  HAL_UART_Transmit(&huart2, chMsg, strlen(chMsg), HAL_MAX_DELAY);
//	  sprintf(chMsg, "Temperature: %d,%d\n\r", pDHT11Data->u8Temperature, pDHT11Data->u8TmpFloatPart);
//	  HAL_UART_Transmit(&huart2, chMsg, strlen(chMsg), HAL_MAX_DELAY);
	  //HAL_UART_Transmit(&huart2, TemperatureData_S.DATA_S.sTimestamp, 8, HAL_MAX_DELAY);
	  sprintf(chMsg, "CHECK_SUM: %d\n\r", ((uint8_t*)&u16CheckSum)[0]);
	  HAL_UART_Transmit(&huart2, chMsg, strlen(chMsg), HAL_MAX_DELAY);
#endif
	  vTaskDelayUntil( &xLastWakeTime, 8500);
	  //vTaskDelay(g_TicksToDelay*1000/portTICK_PERIOD_MS);
	  //HAL_Delay(500);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_readPotentiometerData */
/**
* @brief Function implementing the PotentiometerTaskHandle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_readPotentiometerData */
void readPotentiometerData(void *argument)
{
  /* USER CODE BEGIN readPotentiometerData */
#ifdef DEBUG
	char chMsg[10] = {0};
#endif
	//uint8_t u8Buffer[0x800] = {0};
	uint32_t uiValue = 0;
	DATA_LOG_S PotentiometerData_S = {0};
	TickType_t xLastWakeTime;
	memcpy(PotentiometerData_S.DATA_S.sDataType,
				 "PTM",
				 3);
  /* Infinite loop */
  for(;;)
  {
#ifdef DEBUG
	  memset(chMsg, 0, 10);
#endif
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  uiValue = HAL_ADC_GetValue(&hadc1);
	  g_TicksToDelay = uiValue/1000+1;
	  memcpy(PotentiometerData_S.DATA_S.sTimestamp,
			 __TIME__,
			 8);
	  //get_time(PotentiometerData_S.DATA_S.sTimestamp);
	  memcpy(PotentiometerData_S.DATA_S.bDataBuffer,
	  		(char*)(&uiValue),
	  		sizeof(uint32_t));

	  OS_Tick_Disable();
	  LoggerSaveData(PotentiometerData_S.bBuffer, sizeof(DATA_LOG_S));
	  //save_to_flash((uint8_t*)&PotentiometerData_S, sizeof(DATA_LOG_S));
	  OS_Tick_Enable();
	  //read_flash(u8Buffer);
#ifdef DEBUG
	  sprintf(chMsg, "PTM: %u\n\r", uiValue);
	  HAL_UART_Transmit(&huart2, chMsg, strlen(chMsg), HAL_MAX_DELAY);
#endif
	  //vTaskDelay(g_TicksToDelay);
	  HAL_Delay(g_TicksToDelay*1000);
    //osDelay(500);
  }
  /* USER CODE END readPotentiometerData */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
