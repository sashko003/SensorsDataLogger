/*
 * flash_driver.h
 *
 *  Created on: April 23, 2020
 *      Author: oleksandr.kovbasiuk
 */

#ifndef INC_FLASH_DRIVER_H_
#define INC_FLASH_DRIVER_H_

#include "main.h"

void flash_write(uint32_t address,uint32_t data);
uint32_t flash_erase(uint32_t address);


#endif /* INC_FLASH_DRIVER_H_ */
