/*
 * common.h
 *
 *  Created on: May 1, 2020
 *      Author: sashko003
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

#include "stm32f3xx_hal.h"

//__attribute__((__section__(".log_data"))) const uint8_t LOG_DATA[1];
#define FLASH_STORAGE (0x08070000)
#define PAGE_SIZE 0x800
#define LOGS_BEGIN FLASH_STORAGE + 0x10
#define LOGGER_DATA FLASH_STORAGE
#define MAX_PAGES  (4U)
#define RESERVED_PAGES (2U)
#define MIRROR_PAGE (FLASH_STORAGE + (MAX_PAGES-2)*PAGE_SIZE)
#define SYSTEM_PAGE (FLASH_STORAGE + (MAX_PAGES-1)*PAGE_SIZE)

#define ADDRESS(address) ((uint32_t*)address)

#endif /* INC_COMMON_H_ */
