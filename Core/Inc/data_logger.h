#include "common.h"

#define UNDEFINE 0xFFFFFFFF
#define LOGGER_STORE_DATA_SIZE  8

typedef enum logger_state_e
{
	NOT_INIT = 0,
	INITIALIZED = 1,
	IN_WORK = 2,
	UPDATED = 3,
	UNDEFINED = 0xFFFF
} LOGGER_STATE_E;

typedef enum page_state_e
{
	FULL = 0,
	WRITING = 1,
	ERASED = 2,
	BUSY = 3
} PAGE_STATE_E;

typedef union log_header_s
{
	struct header_s
	{
		//uint16_t u16RowsNumber;
		uint8_t u8PageNumber;
		uint8_t u8CurrentPage;
		PAGE_STATE_E PageStateE;
		uint32_t u32LogSize;
	} HEADER_S;

	uint8_t bBuffer[16];

	uint8_t *pFlashPointer;
} LOG_HEADER_S;

/* new version LOG_HEADER */
typedef struct logger_data_s
{
	uint8_t _FilledPages;
	uint8_t _CurrentPage;
	uint32_t _LoopNumber;
	LOGGER_STATE_E _State;
	uint32_t _LogSize;
} LOGGER_DATA_S;

void LoggerInit(void);
void LoggerUpdate(LOGGER_STATE_E state);
void LogData(uint8_t* pData, uint32_t size);


