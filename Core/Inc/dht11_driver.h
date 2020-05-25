/*
 * dht11_driver.h
 *
 *  Created on: April 23, 2020
 *      Author: oleksandr.kovbasiuk
 */

#ifndef INC_DHT11_DRIVER_H_
#define INC_DHT11_DRIVER_H_

#include "cmsis_os.h"
#include "main.h"

typedef struct dht11_data_s
{
	uint8_t u8Humidity;
	uint8_t u8HmdFloatPart;
	uint8_t u8Temperature;
	uint8_t u8TmpFloatPart;
	uint8_t u8CheckSum;
} DHT11_DATA_S;

void set_timer(TIM_HandleTypeDef* pTimer);
TIM_HandleTypeDef* get_timer(void);
void DHT11_Start(void);
uint8_t DHT11_CheckResponse(void);
uint8_t DHT11_ReadByte(void);
const DHT11_DATA_S* DHT11_ReadData(void);
uint8_t DHT11_IsDataValid(void);


#endif /* INC_DHT11_DRIVER_H_ */
