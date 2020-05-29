/*
 * flash_driver.h
 *
 *  Created on: April 23, 2020
 *      Author: oleksandr.kovbasiuk
 */

#ifndef INC_FLASH_DRIVER_H_
#define INC_FLASH_DRIVER_H_

#include "main.h"

void FlashWrite(uint32_t address,uint32_t data);
uint32_t FlashErase(uint32_t address);
void FlashEraseAllPages(void);
void FlashErasePage(uint32_t address);
void FlashUnlock(void);
void FlashLock();
uint32_t FlashRead(uint32_t address);
uint32_t FlashCopyPage(uint32_t src, uint32_t dst);



#endif /* INC_FLASH_DRIVER_H_ */
