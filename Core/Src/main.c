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

typedef union log_header_s
{
	struct header_s
	{
		uint32_t u32RowsNumber;
		uint8_t u8PageNumber;
		uint32_t u32PlaceAvailable;
		uint32_t u32LogSize;
	} HEADER_S;

	uint8_t bBuffer[16];

	uint8_t *pFlashPointer;
} LOG_HEADER_S;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
__attribute__((__section__(".log_data"))) const uint8_t LOG_DATA[1];
#define FLASH_STORAGE LOG_DATA
#define PAGE_SIZE 0x800
#define LOGS_BEGIN FLASH_STORAGE + 0x10
#define LOGGER_DATA FLASH_STORAGE
#define MAX_PAGES  (3U)
#define MIRROR_PAGE (FLASH_STORAGE + MAX_PAGES*PAGE_SIZE)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* Definitions for readDHT11Data */
osThreadId_t readDHT11DataHandle;
const osThreadAttr_t readDHT11Data_attributes = {
  .name = "readDHT11Data",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for PotentiometerDa */
osThreadId_t PotentiometerDaHandle;
const osThreadAttr_t PotentiometerDa_attributes = {
  .name = "PotentiometerDa",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */
static LOG_HEADER_S LogHeaderS = {0};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_RTC_Init(void);
void StartDefaultTask(void *argument);
void readPotentiometerData(void *argument);

/* USER CODE BEGIN PFP */
uint32_t used_mem(uint32_t address);

uint32_t flash_erase(uint32_t address);
uint32_t flash_copy_page(uint32_t src, uint32_t dst);
uint8_t define_page(uint32_t address);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim2, 0); // set counter value to 0
	uint32_t i = __HAL_TIM_GET_COUNTER(&htim2);
	while(__HAL_TIM_GET_COUNTER(&htim2) < us){
		;//i = __HAL_TIM_GET_COUNTER(&htim2);
	}
	// wait for the counter to reach the us input in the parameter
}

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

  sTime.Hours = atoi(chHours); // set hours
  sTime.Minutes = atoi(chMinutes); // set minutes
  sTime.Seconds = atoi(chSeconds); // set seconds
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    //_Error_Handler(__FILE__, __LINE__);
	  return;
  }
//  sDate.WeekDay = RTC_WEEKDAY_THURSDAY; //  day
//  sDate.Month = RTC_MONTH_AUGUST; // month
//  sDate.Date = 0x9; // date
//  sDate.Year = 0x18; // year
//  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
//  {
//    _Error_Handler(__FILE__, __LINE__);
//  }
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // backup register
}

void get_time(char* time)
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
 sprintf((char*)time,"%02d:%02d:%02d",gTime.Hours, gTime.Minutes, gTime.Seconds);
 int len = strlen(time);
 len = sizeof(time);
/* Display date Format: dd-mm-yy */
 //sprintf((char*)date,"%02d-%02d-%2d",gDate.Date, gDate.Month, 2000 + gDate.Year);
}
/*****************************___DHT11___*****************************/
float fTemperature = 0.0;
float fHumidity = 0.0;
uint8_t u8RhByte1 = 0, u8RhByte2 = 0, u8Temp1 = 0, u8Temp2 = 0;
uint16_t u16CheckSum = 0, u16RH = 0, u16Temp = 0;
uint8_t u8Presence = 0;

void set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_Init_S = {0};
	GPIO_Init_S.Pin = GPIO_Pin;
	GPIO_Init_S.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_Init_S.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_Init_S);
}

void set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_Init_S = {0};
	GPIO_Init_S.Pin = GPIO_Pin;
	GPIO_Init_S.Mode = GPIO_MODE_INPUT;
	GPIO_Init_S.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOx, &GPIO_Init_S);
}

#define DHT11_GPIO GPIOA
#define DHT11_PIN  GPIO_PIN_1

void DHT11_Start(void)
{
	set_Pin_Output(DHT11_GPIO, DHT11_PIN);
	HAL_GPIO_WritePin(DHT11_GPIO, DHT11_PIN, RESET);
	HAL_Delay(18);
	HAL_GPIO_WritePin(DHT11_GPIO, DHT11_PIN, SET);
	delay_us(30);
	set_Pin_Input(DHT11_GPIO, DHT11_PIN);
}

uint8_t DHT11_Check_Response(void)
{
	uint8_t u8Response = 0;
	delay_us(40);
	if(!HAL_GPIO_ReadPin(DHT11_GPIO, DHT11_PIN))
	{
		delay_us(80);
		if(HAL_GPIO_ReadPin(DHT11_GPIO, DHT11_PIN))
		{
			u8Response = 1;
		}
		else
		{
			u8Response = -1;
		}
	}

	int k = 0;
	uint32_t start = __HAL_TIM_GET_COUNTER(&htim2);
	while(HAL_GPIO_ReadPin(DHT11_GPIO, DHT11_PIN))
	{;
	    if(__HAL_TIM_GET_COUNTER(&htim2)-start > 50000)
	    {
	    	break;
	    }
	}

	return u8Response;
}

uint8_t DHT11_Read(void)
{
	uint8_t i = 0, j;
	for(j = 0; j<8; ++j)
	{
		int k = 0;
		while(!HAL_GPIO_ReadPin(DHT11_GPIO, DHT11_PIN))
		{;
			k++;
		}
		delay_us(30);
		if(!HAL_GPIO_ReadPin(DHT11_GPIO, DHT11_PIN))
		{
			i &= ~(1<<(7-j));
		}
		else
		{
		    i |= (1<<(7-j));
		}
		uint32_t start = __HAL_TIM_GET_COUNTER(&htim2);
		while(HAL_GPIO_ReadPin(DHT11_GPIO, DHT11_PIN))
		{;
		    if(__HAL_TIM_GET_COUNTER(&htim2)-start > 50000)
			{
			  	break;
			}
		}
	}
	return i;
}
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

void save_to_flash(uint8_t *data)//, uint32_t data_length)
{
	static uint32_t u32Offset = 0;
	uint32_t curr_size = used_mem(FLASH_STORAGE);
	uint32_t *data_to_FLASH = (uint32_t *)malloc(curr_size+16);
	memset((uint8_t*)data_to_FLASH, 0, curr_size+16);
	memcpy((uint8_t*)data_to_FLASH, (uint8_t*)FLASH_STORAGE, curr_size);
	memcpy((uint8_t*)data_to_FLASH+curr_size, (uint8_t*)(FLASH_STORAGE+curr_size), 16);

	volatile uint32_t data_length = ((16) / 4)
									+ (int)((16 % 4) != 0);
	volatile uint16_t pages = ((16)/PAGE_SIZE)
									+ (int)((16%PAGE_SIZE) != 0);
	  /* Unlock the Flash to enable the flash control register access *************/
	  HAL_HAL_FLASH_Unlock(); HAL_FLASH_OB_Unlock();

	  /* Allow Access to option bytes sector */
	  //HAL_FLASH_OB_Unlock();

	  /* Fill EraseInit structure*/
	  FLASH_EraseInitTypeDef EraseInitStruct;
	  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	  EraseInitStruct.PageAddress = FLASH_STORAGE + u32Offset;
	  EraseInitStruct.NbPages = pages;
	  uint32_t PageError;

	  volatile uint32_t write_cnt=0, index=0;

	  volatile HAL_StatusTypeDef status = HAL_OK;
	  status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
	  CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
	  while(index < data_length)
	  {
		  if (status == HAL_OK)
		  {
			  status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE+u32Offset+write_cnt, data_to_FLASH[index]);
			  if(status == HAL_OK)
			  {
				  write_cnt += 4;
				  index++;
			  }
//			  status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE+u32Offset, *(uint64_t*)(&data[index]));
//			  if(status == HAL_OK)
//			  {
//			  	  u32Offset += 4;
//			  	  index += 4;
//			  }
		  }
	  }
	  u32Offset += 16;
	  HAL_HAL_FLASH_Lock(); HAL_FLASH_OB_Lock();
}

void read_flash(uint8_t* data)
{
	volatile uint32_t read_data;
	volatile uint32_t read_cnt=0;
	do
	{
		read_data = *(uint32_t*)(FLASH_STORAGE + read_cnt);
		if(read_data != 0xFFFFFFFF)
		{
			data[read_cnt] = (uint8_t)read_data;
			data[read_cnt + 1] = (uint8_t)(read_data >> 8);
			data[read_cnt + 2] = (uint8_t)(read_data >> 16);
			data[read_cnt + 3] = (uint8_t)(read_data >> 24);
			read_cnt += 4;
		}
	}while(read_data != 0xFFFFFFFF);
}

uint32_t used_mem(uint32_t address)
{
	uint32_t* read_data = NULL, size = 0;
	read_data = address;
	while(0xFFFFFFFF != read_data[size])
	{
		++size;
	}
	return (4*size);
}

#define FLASH_KEY1               ((uint32_t)0x45670123)
#define FLASH_KEY2               ((uint32_t)0xCDEF89AB)
///////////////////////////////////////////////////////
void LoggerInit(void)
{
	memcpy(LogHeaderS.bBuffer, (uint8_t*)LOGGER_DATA, sizeof(LOG_HEADER_S));
	uint64_t test1 = *(uint64_t*)LogHeaderS.bBuffer;
	uint64_t test2 = *(uint64_t*)(LogHeaderS.bBuffer+8);
//	if(0xFFFFFFFFFFFFFFFF == *(uint64_t*)LogHeaderS.bBuffer &&
//	   0xFFFFFFFFFFFFFFFF == *(uint64_t*)(LogHeaderS.bBuffer+8))
	{
	    memset(LogHeaderS.bBuffer, 0, sizeof(LOG_HEADER_S));
	    HAL_FLASH_Unlock(); HAL_FLASH_OB_Unlock();
	    for(int i = 0; i<sizeof(LOG_HEADER_S); i+=4)
	    {
	    	flash_write(LOGGER_DATA+i, 0);
	    }
	    HAL_FLASH_Lock(); HAL_FLASH_OB_Lock();
	}
}

void LoggerUpdate(uint8_t *pLogger)
{
	uint8_t test[16] = {0};
	uint32_t size = sizeof(LOG_HEADER_S);
	HAL_FLASH_Unlock(); HAL_FLASH_OB_Unlock();
    for(int i = 0; i<sizeof(LOG_HEADER_S); i+=4)
    {
    	flash_write(LOGGER_DATA+i, 0xFFFF);
    }
	for(int i = 0; i<sizeof(LOG_HEADER_S); i+=4)
    {
	    flash_write(LOGGER_DATA+i, *(uint32_t*)(pLogger+i));
	    //pLogger += 4;
	}
	memcpy(&test, (void*)(LOGGER_DATA), 16);
	HAL_FLASH_Lock(); HAL_FLASH_OB_Lock();
}

void LogData(uint8_t* pData, uint32_t size)
{
	if(LogHeaderS.HEADER_S.u32LogSize+size >= PAGE_SIZE)
	{
		LogHeaderS.HEADER_S.u8PageNumber += 1;

		if(MAX_PAGES == LogHeaderS.HEADER_S.u8PageNumber)
		{
			LogHeaderS.HEADER_S.u8PageNumber = 0;
		}
		uint32_t page_adr = FLASH_STORAGE + PAGE_SIZE*LogHeaderS.HEADER_S.u8PageNumber;
		if(0xFF == define_page(page_adr))
		{
			for(int i = 0; i<size; i+=4)
			{
				flash_write(page_adr + i, 0);
			}
		}
		else
		{
			flash_erase(MIRROR_PAGE);
			flash_copy_page(page_adr, MIRROR_PAGE);
			flash_erase(page_adr);
		}
		LogHeaderS.HEADER_S.u32LogSize = 0;
	}
	HAL_FLASH_Unlock(); HAL_FLASH_OB_Unlock();

	if(0 == size%4)
	{
		uint32_t curr_adr = LOGS_BEGIN+(LogHeaderS.HEADER_S.u8PageNumber*PAGE_SIZE)+LogHeaderS.HEADER_S.u32LogSize;
		for(int i = 0; i<size; i+=4)
		{
			flash_write(curr_adr+i, *(uint32_t*)(pData+i));
		}
	}
//	else
//	{
//		for(int i = 0; i<size-1; i+=2)
//		{
//			flash_write(FLASH_STORAGE+LogHeaderS.HEADER_S.u32LogSize+i, *(uint16_t*)pData);
//		}
//		flash_write(FLASH_STORAGE+LogHeaderS.HEADER_S.u32LogSize+i, *(uint16_t*)pData);
//	}
	LogHeaderS.HEADER_S.u32LogSize += size;
	HAL_FLASH_Lock(); HAL_FLASH_OB_Lock();
}

uint8_t flash_ready(void) {
	return !(FLASH->SR & FLASH_SR_BSY);
}

void flash_erase_all_pages(void) {
    FLASH->CR |= FLASH_CR_MER;
    FLASH->CR |= FLASH_CR_STRT;
    while(!flash_ready())
    	;
    FLASH->CR &= FLASH_CR_MER;
}

void flash_erase_page(uint32_t address) {
    FLASH->CR|= FLASH_CR_PER;
    FLASH->AR = address;
    FLASH->CR|= FLASH_CR_STRT;
    while(!flash_ready())
    	;
    FLASH->CR&= ~FLASH_CR_PER;
}


void flash_unlock(void) {
	  FLASH->KEYR = FLASH_KEY1;
	  FLASH->KEYR = FLASH_KEY2;
}

void flash_lock() {
	FLASH->CR |= FLASH_CR_LOCK;
}


void flash_write(uint32_t address,uint32_t data)
{
    char chTest[4] = {0};
    uint16_t test = 0;
	FLASH->CR |= FLASH_CR_PG;
//	while(!flash_ready())
//		;
//	memset((void*)address, 0, sizeof(uint32_t));
	while(!flash_ready())
		;
	test = (data>>0) & 0xFFFF;
	//HAL_FLASH_Unlock(); HAL_FLASH_OB_Unlock();
    *(__IO uint16_t*)address = test;
    test = *(__IO uint16_t*)address;
    *(uint16_t*)chTest = *(__IO uint16_t*)address;
	memcpy((__IO uint16_t*)address, (uint16_t*)&data, 2);
	while(!flash_ready())
		;
	address+=2;
	test = (data>>16) & 0xFFFF;
    *(__IO uint16_t*)address = (data>>16) & 0xFFFF;
    test = *(__IO uint16_t*)address;
    *(uint16_t*)(chTest+2) = (uint16_t)data;
	//memcpy((__IO uint16_t*)address, ((uint16_t*)&data)+2, 2);
	while(!flash_ready())
		;
    FLASH->CR &= ~(FLASH_CR_PG);

}

uint32_t flash_read(uint32_t address) {
	return (*(__IO uint32_t*) address);
}

uint32_t flash_erase(uint32_t address)
{
	/* Allow Access to option bytes sector */
	//HAL_FLASH_OB_Unlock();

	 /* Fill EraseInit structure*/
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = address;
	EraseInitStruct.NbPages = 1;
	uint32_t PageError;
	volatile HAL_StatusTypeDef status = HAL_OK;
	//OS_Tick_Disable();
	while(!flash_ready())
			;

	HAL_FLASH_Unlock(); HAL_FLASH_OB_Unlock();
	status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
	HAL_FLASH_Lock(); HAL_FLASH_OB_Lock();

	while(!flash_ready())
			;
	//OS_Tick_Enable();

	return status;
}

uint32_t flash_copy_page(uint32_t src, uint32_t dst)
{
	HAL_FLASH_Unlock(); HAL_FLASH_OB_Unlock();
    for(uint32_t i = 0; i<PAGE_SIZE; i += 4)
    {
    	flash_write(dst + i, *((uint32_t*)(src + i)));
    }
    HAL_FLASH_Lock(); HAL_FLASH_OB_Lock();
    return HAL_OK;
}

uint8_t define_page(uint32_t address)
{
	uint8_t pageHeader[16] = {0};
	uint8_t pageState = 1;
	HAL_FLASH_Lock(); HAL_FLASH_OB_Lock();
	memcpy(pageHeader, (uint8_t*)address, 16);
	if(0 == (uint64_t*)pageHeader[0] &&
	   0 == (uint64_t*)pageHeader[1])
	{
		pageState = 0;
	}
	else if(0xFFFFFFFFFFFFFFFF == (uint64_t*)pageHeader[0] &&
			0xFFFFFFFFFFFFFFFF == (uint64_t*)pageHeader[1])
	{
		pageState = 0xFF;
	}
	else
	{
		pageState = 1;
	}
	HAL_FLASH_Unlock(); HAL_FLASH_OB_Unlock();
	return pageState;
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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim2);
  DATA_LOG_S TestTmp_S = {0}, TestHmd_S = {0};
  LOG_HEADER_S TestHeaders_S = {0};
  memcpy(TestTmp_S.bBuffer, LOGS_BEGIN, 16);
  memcpy(TestHmd_S.bBuffer, LOGS_BEGIN+16, 16);
  memcpy(TestHeaders_S.bBuffer, LOGGER_DATA, 16);
  flash_erase(FLASH_STORAGE);
  flash_erase(MIRROR_PAGE);
  LoggerInit();
  set_time();

//  HAL_HAL_FLASH_Unlock(); HAL_FLASH_OB_Unlock();
//  uint32_t test = 0;
//  for(int i = 0; i < 10; ++i)
//	  test = (*((__IO uint8_t*)(LOGGER_DATA + i))) = i;
//  HAL_HAL_FLASH_Lock(); HAL_FLASH_OB_Lock();

  //NVIC_SetPriorityGrouping( NVIC_PriorityGroup_4 );
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

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
  /* creation of readDHT11Data */
  readDHT11DataHandle = osThreadNew(StartDefaultTask, NULL, &readDHT11Data_attributes);

  /* creation of PotentiometerDa */
  PotentiometerDaHandle = osThreadNew(readPotentiometerData, NULL, &PotentiometerDa_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

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
	  StartDefaultTask(0);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_TIM2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_SYSCLK;
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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  htim2.Init.Prescaler = 72-1;
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the readDHT11Data thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	static uint32_t u32Offset = 0;
	char chMsg[30] = {0};
    char chData[5] = {0};
	static DATA_LOG_S TemperatureData_S = {0},
			   HumidityData_S = {0},
			   Test_S = {0};
	memcpy(TemperatureData_S.DATA_S.sDataType,
		   "TMP",
		   3);
	memcpy(HumidityData_S.DATA_S.sDataType,
		   "HMD",
		   3);

  /* Infinite loop */
  for(;;)
  {
	  HAL_UART_Transmit(&huart2, "NEW_LOOP\n\r", 11, HAL_MAX_DELAY);
	  memset(chMsg, 0, 30);
	  memset(chData, 0, 5);
	  {
	  sprintf(chMsg, "Response: %d\n\r", u8Presence);
	  HAL_UART_Transmit(&huart2, chMsg, strlen(chMsg), HAL_MAX_DELAY);
	  OS_Tick_Disable();
	  DHT11_Start();
	  u8Presence = DHT11_Check_Response();
	  }

	  for(int i = 0; i<5; ++i)
	  {
	   	chData[i] = DHT11_Read();
	  }
	  OS_Tick_Enable();

//	  memcpy(HumidityData_S.DATA_S.sTimestamp,
//	  	  	 __TIME__,
//	  	  	 8);
//	  memcpy(TemperatureData_S.DATA_S.sTimestamp,
//	  	     __TIME__,
//	  	  	 8);
	  get_time(HumidityData_S.DATA_S.sTimestamp);
	  get_time(TemperatureData_S.DATA_S.sTimestamp);

	  memcpy(HumidityData_S.DATA_S.bDataBuffer,
	  	     (void*)&chData[0],
	   	  	 sizeof(uint8_t));
	  memcpy(TemperatureData_S.DATA_S.bDataBuffer,
	  	     (void*)&chData[2],
	  	  	 sizeof(uint8_t));

	  OS_Tick_Disable();
	  LogData(TemperatureData_S.bBuffer, sizeof(DATA_LOG_S));
	  LogData(HumidityData_S.bBuffer, sizeof(DATA_LOG_S));
	  //LogData(LogHeaderS.bBuffer, sizeof(LOG_HEADER_S));
//	  HAL_FLASH_Unlock(); HAL_FLASH_OB_Unlock();
//	  int i = 0;
//	  uint32_t test = 0;
//	  char chTest[4] = {0};
//	  for(; i<4; ++i)
//	  {
//		  test = ((uint32_t*)TemperatureData_S.bBuffer)[i];
//		  *(uint32_t*)chTest = ((uint32_t*)TemperatureData_S.bBuffer)[i];
//		  flash_write(LOGS_BEGIN+LogHeaderS.HEADER_S.u32LogSize+i*4, ((uint32_t*)TemperatureData_S.bBuffer)[i]);
//	  }
//	  LogHeaderS.HEADER_S.u32LogSize += 16;
//	  for(i = 0; i<4; ++i)
//	  {
//		  test = ((uint32_t*)HumidityData_S.bBuffer)[i];
//		  *(uint32_t*)chTest = ((uint32_t*)HumidityData_S.bBuffer)[i];
//	  	  flash_write(LOGS_BEGIN+LogHeaderS.HEADER_S.u32LogSize+i*4, ((uint32_t*)HumidityData_S.bBuffer)[i]);
//	  }
//	  LogHeaderS.HEADER_S.u32LogSize += 16;
//	  HAL_FLASH_Lock(); HAL_FLASH_OB_Lock();

	  LoggerUpdate((uint8_t*)&LogHeaderS);
	  LOG_HEADER_S TestHeaders_S = {0};
	  memcpy(TestHeaders_S.bBuffer, LOGGER_DATA, 16);
//	  save_to_flash((uint8_t*)&HumidityData_S);//, sizeof(DATA_LOG_S));
//	  save_to_flash((uint8_t*)&TemperatureData_S);//, sizeof(DATA_LOG_S));
      OS_Tick_Enable();
	  memset(HumidityData_S.DATA_S.bDataBuffer, 0, sizeof(uint32_t)+8);
	  memset(TemperatureData_S.DATA_S.bDataBuffer, 0, sizeof(uint32_t)+8);
	  //memcpy(Test_S.bBuffer, LOGS_BEGIN+LogHeaderS.HEADER_S.u32LogSize+u32Offset-32, 16);
      memcpy(Test_S.bBuffer, LOGS_BEGIN+LogHeaderS.HEADER_S.u32LogSize-16, 16);
	  u8Presence = 0;
	  memset(chMsg, 0, 20);
	  uint64_t f = 0.0;
	  uint16_t u16CheckSum = 0;
	  for(int i = 0; i<5; ++i)
	  {
	  	f = chData[i];
	  	if(4 != i){
	  	    u16CheckSum += chData[i];
	  	}
	  	sprintf(chMsg, "Data%d: %d\n\r", i, f);
	  	HAL_UART_Transmit(&huart2, chMsg, strlen(chMsg), HAL_MAX_DELAY);
	  }
	  HAL_UART_Transmit(&huart2, TemperatureData_S.DATA_S.sTimestamp, 8, HAL_MAX_DELAY);
	  sprintf(chMsg, "CHECK_SUM: %d\n\r", ((uint8_t*)&u16CheckSum)[0]);
	  HAL_UART_Transmit(&huart2, chMsg, strlen(chMsg), HAL_MAX_DELAY);
	  //HAL_Delay(500);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_readPotentiometerData */
/**
* @brief Function implementing the PotentiometerDa thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_readPotentiometerData */
void readPotentiometerData(void *argument)
{
  /* USER CODE BEGIN readPotentiometerData */
	char chMsg[10] = {0};
	//uint8_t u8Buffer[0x800] = {0};
	uint32_t uiValue = 0;
	DATA_LOG_S PotentiometerData_S = {0};
  /* Infinite loop */
  for(;;)
  {
	  memset(chMsg, 0, 10);
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  uiValue = HAL_ADC_GetValue(&hadc1);
//	  memcpy(PotentiometerData_S.sTimestamp,
//			 __TIME__,
//			 8);
//	  memcpy(PotentiometerData_S.sDataType,
//			 "PTM",
//			 3);
//	  memcpy(PotentiometerData_S.bDataBuffer,
//			 (char*)(&uiValue),
//			 sizeof(uint32_t));
//	  OS_Tick_Disable();
//	  save_to_flash((uint8_t*)&PotentiometerData_S, sizeof(DATA_LOG_S));
//	  OS_Tick_Enable();
	  //read_flash(u8Buffer);
	  sprintf(chMsg, "%u\n\r", uiValue);
	  HAL_UART_Transmit(&huart2, chMsg, strlen(chMsg), HAL_MAX_DELAY);
	  //HAL_Delay(500);
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
