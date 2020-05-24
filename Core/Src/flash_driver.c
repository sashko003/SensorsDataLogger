/*
 * flash_driver.c
 *
 *  Created on: April 23, 2020
 *      Author: oleksandr.kovbasiuk
 */
#include "flash_driver.h"
#include "common.h"

//void save_to_flash(uint8_t *data)//, uint32_t data_length)
//{
//	static uint32_t u32Offset = 0;
//	uint32_t curr_size = used_mem(FLASH_STORAGE);
//	uint32_t *data_to_FLASH = (uint32_t *)malloc(curr_size+16);
//	memset((uint8_t*)data_to_FLASH, 0, curr_size+16);
//	memcpy((uint8_t*)data_to_FLASH, (uint8_t*)FLASH_STORAGE, curr_size);
//	memcpy((uint8_t*)data_to_FLASH+curr_size, (uint8_t*)(FLASH_STORAGE+curr_size), 16);
//
//	volatile uint32_t data_length = ((16) / 4)
//									+ (int)((16 % 4) != 0);
//	volatile uint16_t pages = ((16)/PAGE_SIZE)
//									+ (int)((16%PAGE_SIZE) != 0);
//	  /* Unlock the Flash to enable the flash control register access *************/
//	  HAL_FLASH_Unlock(); HAL_FLASH_OB_Unlock();
//
//	  /* Allow Access to option bytes sector */
//	  //HAL_FLASH_OB_Unlock();
//
//	  /* Fill EraseInit structure*/
//	  FLASH_EraseInitTypeDef EraseInitStruct;
//	  EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
//	  EraseInitStruct.PageAddress = FLASH_STORAGE + u32Offset;
//	  EraseInitStruct.NbPages = pages;
//	  uint32_t PageError;
//
//	  volatile uint32_t write_cnt=0, index=0;
//
//	  volatile HAL_StatusTypeDef status = HAL_OK;
//	  status = HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
//	  CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
//	  while(index < data_length)
//	  {
//		  if (status == HAL_OK)
//		  {
//			  status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE+u32Offset+write_cnt, data_to_FLASH[index]);
//			  if(status == HAL_OK)
//			  {
//				  write_cnt += 4;
//				  index++;
//			  }
////			  status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_STORAGE+u32Offset, *(uint64_t*)(&data[index]));
////			  if(status == HAL_OK)
////			  {
////			  	  u32Offset += 4;
////			  	  index += 4;
////			  }
//		  }
//	  }
//	  u32Offset += 16;
//	  HAL_HAL_FLASH_Lock(); HAL_FLASH_OB_Lock();
//}
//
//void read_flash(uint8_t* data)
//{
//	volatile uint32_t read_data;
//	volatile uint32_t read_cnt=0;
//	do
//	{
//		read_data = *(uint32_t*)(FLASH_STORAGE + read_cnt);
//		if(read_data != 0xFFFFFFFF)
//		{
//			data[read_cnt] = (uint8_t)read_data;
//			data[read_cnt + 1] = (uint8_t)(read_data >> 8);
//			data[read_cnt + 2] = (uint8_t)(read_data >> 16);
//			data[read_cnt + 3] = (uint8_t)(read_data >> 24);
//			read_cnt += 4;
//		}
//	}while(read_data != 0xFFFFFFFF);
//}
//
//uint32_t used_mem(uint32_t address)
//{
//	uint32_t* read_data = NULL, size = 0;
//	read_data = address;
//	while(0xFFFFFFFF != read_data[size])
//	{
//		++size;
//	}
//	return (4*size);
//}

#define FLASH_KEY1               ((uint32_t)0x45670123)
#define FLASH_KEY2               ((uint32_t)0xCDEF89AB)
///////////////////////////////////////////////////////




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
