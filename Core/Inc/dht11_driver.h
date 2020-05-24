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

void set_timer(TIM_HandleTypeDef* pTimer);
TIM_HandleTypeDef* get_timer(void);
void DHT11_Start(void);
uint8_t DHT11_Check_Response(void);
uint8_t DHT11_Read(void);


#endif /* INC_DHT11_DRIVER_H_ */
