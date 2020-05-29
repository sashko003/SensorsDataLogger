/*
 * dht11_driver.c
 *
 *  Created on: April 23, 2020
 *      Author: oleksandr.kovbasiuk
 */
#include "dht11_driver.h"

#define DHT11_GPIO GPIOA
#define DHT11_PIN  GPIO_PIN_1

typedef enum response_e
{
	UNDEFINED_R = 0,
	SUCCESS_R = 1,
	FAIL_R = -1,
} RESPONSE_E;


static TIM_HandleTypeDef htim;


/*****************************___DHT11___*****************************/
//float fTemperature = 0.0;
//float fHumidity = 0.0;
//uint8_t u8RhByte1 = 0, u8RhByte2 = 0, u8Temp1 = 0, u8Temp2 = 0;
//uint16_t u16CheckSum = 0, u16RH = 0, u16Temp = 0;
static DHT11_DATA_S DHT11Data_s = {0};

void DHT11_SetTimer(TIM_HandleTypeDef* pTimer)
{
	memcpy(&htim, pTimer, sizeof(TIM_HandleTypeDef));
}


TIM_HandleTypeDef* DHT11_GetTimer(void)
{
	return &htim;
}


void delay_us(uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim, 0); // set counter value to 0
	uint32_t i = __HAL_TIM_GET_COUNTER(&htim);
	while(__HAL_TIM_GET_COUNTER(&htim) < us){
		;//i = __HAL_TIM_GET_COUNTER(&htim2);
	}
	// wait for the counter to reach the us input in the parameter
}

/**
  * @brief Initialize pin on OUTPUT
  * @param pGPIOx -
  *        GPIO_Pin -
  * @retval None
  */
void set_Pin_Output(GPIO_TypeDef *pGPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_Init_S = {0};
	GPIO_Init_S.Pin = GPIO_Pin;
	GPIO_Init_S.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_Init_S.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(pGPIOx, &GPIO_Init_S);
}

/**
  * @brief Initialize pin on INPUT
  * @param pGPIOx -
  *        GPIO_Pin -
  * @retval None
  */
void set_Pin_Input(GPIO_TypeDef *pGPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_Init_S = {0};
	GPIO_Init_S.Pin = GPIO_Pin;
	GPIO_Init_S.Mode = GPIO_MODE_INPUT;
	GPIO_Init_S.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(pGPIOx, &GPIO_Init_S);
}

void DHT11_Start(void)
{
	set_Pin_Output(DHT11_GPIO, DHT11_PIN);
	HAL_GPIO_WritePin(DHT11_GPIO, DHT11_PIN, RESET);
	HAL_Delay(18);
	HAL_GPIO_WritePin(DHT11_GPIO, DHT11_PIN, SET);
	delay_us(30);
	set_Pin_Input(DHT11_GPIO, DHT11_PIN);
}

uint8_t DHT11_CheckResponse(void)
{
	uint8_t u8Response = UNDEFINED_R;
	delay_us(40);
	if(!HAL_GPIO_ReadPin(DHT11_GPIO, DHT11_PIN))
	{
		delay_us(80);
		if(HAL_GPIO_ReadPin(DHT11_GPIO, DHT11_PIN))
		{
			u8Response = SUCCESS_R;
		}
		else
		{
			u8Response = FAIL_R;
		}
	}

	int k = 0;
	uint32_t start = __HAL_TIM_GET_COUNTER(&htim);
	while(HAL_GPIO_ReadPin(DHT11_GPIO, DHT11_PIN))
	{;
	    if(__HAL_TIM_GET_COUNTER(&htim)-start > 50000)
	    {
	    	break;
	    }
	}

	return u8Response;
}

uint8_t DHT11_ReadByte(void)
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
		uint32_t start = __HAL_TIM_GET_COUNTER(&htim);
		while(HAL_GPIO_ReadPin(DHT11_GPIO, DHT11_PIN))
		{;
		    if(__HAL_TIM_GET_COUNTER(&htim)-start > 50000)
			{
			  	break;
			}
		}
	}
	return i;
}

const DHT11_DATA_S* DHT11_ReadData(void)
{
	DHT11Data_s.u8Humidity = DHT11_ReadByte();
	DHT11Data_s.u8HmdFloatPart = DHT11_ReadByte();
	DHT11Data_s.u8Temperature = DHT11_ReadByte();
	DHT11Data_s.u8TmpFloatPart = DHT11_ReadByte();
	DHT11Data_s.u8CheckSum = DHT11_ReadByte();
	return &DHT11Data_s;
}

uint8_t DHT11_IsDataValid(void)
{
	uint8_t u8Sum = (DHT11Data_s.u8Humidity + DHT11Data_s.u8HmdFloatPart +
		    DHT11Data_s.u8Temperature + DHT11Data_s.u8TmpFloatPart);
	return u8Sum == DHT11Data_s.u8CheckSum;
}
