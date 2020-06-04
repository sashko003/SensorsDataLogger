#include "common.h"

#define UNDEFINE 0xFFFFFFFF
#define LOGGER_STORE_DATA_SIZE  sizeof(LOGGER_DATA_S)

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

typedef enum data_type_e
{
	HMD = 0,
	TMP = 1,
	PTM = 2,
} DATA_TYPE_E;

typedef union data_log_s
{
	struct data_s
	{
		DATA_TYPE_E DataTypeE;
		uint8_t bDataBuffer[2];
		uint8_t sTimestamp[9];
	} DATA_S;
	uint8_t bBuffer[12];
} DATA_LOG_S;

/* new version LOG_HEADER */
typedef struct logger_data_s
{
	uint8_t _FilledPages;
	uint8_t _CurrentPage;
	LOGGER_STATE_E _State;
	uint32_t _LoopNumber;
	uint32_t _LogSize;
} LOGGER_DATA_S;

void LoggerInit(void);
void LoggerSaveState(LOGGER_STATE_E state);
void LoggerSaveData(uint8_t* pData, uint32_t size);


