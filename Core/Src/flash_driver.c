/*
 * flash_driver.c
 *
 *  Created on: April 23, 2020
 *      Author: oleksandr.kovbasiuk
 */
#include "flash_driver.h"
#include "common.h"


#define FLASH_KEY1               ((uint32_t)0x45670123)
#define FLASH_KEY2               ((uint32_t)0xCDEF89AB)

static uint8_t flash_ready(void);

///////////////////////////////////////////////////////




static uint8_t flash_ready(void)
{
	return !(FLASH->SR & FLASH_SR_BSY);
}

void FlashEraseAllPages(void)
{
    FLASH->CR |= FLASH_CR_MER;
    FLASH->CR |= FLASH_CR_STRT;
    while(!flash_ready())
    	;
    FLASH->CR &= FLASH_CR_MER;
}

void FlashErasePage(uint32_t address)
{
    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR = address;
    FLASH->CR |= FLASH_CR_STRT;
    while(!flash_ready())
    	;
    FLASH->CR &= ~FLASH_CR_PER;
}


void FlashUnlock(void)
{
	  FLASH->KEYR = FLASH_KEY1;
	  FLASH->KEYR = FLASH_KEY2;
}

void FlashLock()
{
	FLASH->CR |= FLASH_CR_LOCK;
}


void FlashWrite(uint32_t address,uint32_t data)
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
	//memcpy((__IO uint16_t*)address, (uint16_t*)&data, 2);
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

uint32_t FlashRead(uint32_t address)
{
	return (*(__IO uint32_t*) address);
}

uint32_t FlashErase(uint32_t address)
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

uint32_t FlashCopyPage(uint32_t src, uint32_t dst)
{
	HAL_FLASH_Unlock(); HAL_FLASH_OB_Unlock();
    for(uint32_t i = 0; i<PAGE_SIZE; i += 4)
    {
    	FlashWrite(dst + i, *((uint32_t*)(src + i)));
    }
    HAL_FLASH_Lock(); HAL_FLASH_OB_Lock();
    return HAL_OK;
}
