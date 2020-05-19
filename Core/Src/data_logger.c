#include "data_logger.h"

static LOG_HEADER_S LogHeaderS = {0};
static LOGGER_DATA_S LoggerS = {0};

static uint32_t find_last_record(void);
static uint8_t define_page(uint32_t address);


void LoggerInit(void)
{
	uint32_t u32Addr = find_last_record();
	memcpy(&LoggerS, ADDRESS(u32Addr), sizeof(LOGGER_STORE_DATA_SIZE));
	switch(LoggerS._State)
	{
		case(NOT_INIT):
		{

		}
		case(INITIALIZED):
		{

		}
		case(IN_WORK):
		{

		}
		case(UPDATED):
		{
			memset(&LoggerS, 0, sizeof(LOGGER_DATA_S));
			break;
		}
		case(UNDEFINED):
		{
			memset(&LoggerS, 0, sizeof(LOGGER_DATA_S));
			break;
		}
		default:
		{
			memset(&LoggerS, 0, sizeof(LOGGER_DATA_S));
		}
	}
//	{
//	    memset(LogHeaderS.bBuffer, 0, sizeof(LOG_HEADER_S));
//	    HAL_FLASH_Unlock(); HAL_FLASH_OB_Unlock();
//	    for(int i = 0; i<sizeof(LOG_HEADER_S); i+=4)
//	    {
//	    	flash_write(LOGGER_DATA+i, 0);
//	    }
//	    HAL_FLASH_Lock(); HAL_FLASH_OB_Lock();
//	}
}

void LoggerUpdate(LOGGER_STATE_E state)
{
	static uint16_t u16Offset = 0;
	uint8_t u8Buffer[LOGGER_STORE_DATA_SIZE] = {0};
	uint8_t test[LOGGER_STORE_DATA_SIZE] = {0};
	LoggerS._State = state;
	memcpy(u8Buffer, &LoggerS, LOGGER_STORE_DATA_SIZE);
	u8Buffer[LOGGER_STORE_DATA_SIZE-1] = 0;

	HAL_FLASH_Unlock(); HAL_FLASH_OB_Unlock();
	if(u16Offset+LOGGER_STORE_DATA_SIZE >= PAGE_SIZE ||
	   0 == u16Offset)
	{
		flash_erase(SYSTEM_PAGE);
		u16Offset = 0;
	}
    for(int i = 0; i<LOGGER_STORE_DATA_SIZE; i+=4)
    {
    	flash_write(SYSTEM_PAGE+u16Offset, *((uint16_t*)(&u8Buffer + i)));
    	u16Offset += 4;
    }
	memcpy(&test, (void*)(SYSTEM_PAGE+u16Offset), LOGGER_STORE_DATA_SIZE);
	HAL_FLASH_Lock(); HAL_FLASH_OB_Lock();
}

void LogData(uint8_t* pData, uint32_t size)
{
	LoggerUpdate(IN_WORK);
	if(LoggerS._LogSize+size >= PAGE_SIZE)
	{
		LoggerS._CurrentPage += 1;
		LogHeaderS.HEADER_S.u8PageNumber += 1;

		if(MAX_PAGES-RESERVED_PAGES == LoggerS._CurrentPage)
		{
			LogHeaderS.HEADER_S.u8PageNumber = 0;
			LoggerS._CurrentPage = 0;
			LoggerS._FilledPages -= 1;
			LoggerS._LoopNumber += 1;
		}
		else
		{
			LoggerS._FilledPages += 1;
		}
		uint32_t page_adr = FLASH_STORAGE + PAGE_SIZE*LoggerS._CurrentPage;
		if(0xFF == define_page(page_adr))
		{
			for(int i = 0; i<size; i+=4)
			{
				flash_write(page_adr + i, 0);
			}
		}
		else
		{
			//flash_erase(MIRROR_PAGE);
			//flash_copy_page(page_adr, MIRROR_PAGE);
			flash_erase(page_adr);
		}
		LogHeaderS.HEADER_S.u32LogSize = 0;
		LoggerS._LogSize = 0;
	}
	HAL_FLASH_Unlock(); HAL_FLASH_OB_Unlock();

	if(0 == size%4)
	{
		uint32_t curr_adr = LOGS_BEGIN+(LoggerS._CurrentPage*PAGE_SIZE)+LoggerS._LogSize;
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
	LoggerS._LogSize += size;
	HAL_FLASH_Lock(); HAL_FLASH_OB_Lock();
}

static uint32_t find_last_record(void)
{
	uint32_t u32Addr = SYSTEM_PAGE;
	if(UNDEFINE == *ADDRESS(u32Addr))
	{
		return u32Addr;
	}
	while(UNDEFINE != *ADDRESS(u32Addr))
	{
		u32Addr += 4;
	}
	return u32Addr-4;
}

static uint8_t define_page(uint32_t address)
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
