//=============================================================================
//								modbus.h
//=============================================================================

#ifndef _MODBUS_H_
#define _MODBUS_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
//#include "project.h"

//#include <utils.h>

#define MAX_REPEATE_TRANSMIT_COUNT 			5
#define MODBUS_REGS_IOCOUNT_MAX				125
#define MOBUS_MAX_PACKET_SIZE				(1 /* addr */ + 				\
											 1 /* func */ + 				\
											 2 /* startAddr */ + 			\
											 2 /* count */ + 				\
											 (MODBUS_REGS_IOCOUNT_MAX * 2))
											 
#define MODBUS_DEVICE_ADDRESS_MAX			247


typedef enum
{
	  MODBUS_CMD_READ_COILS 				= 0x01 // Чтение статуса выходов
	, MODBUS_CMD_READ_DINPUTS 				= 0x02 // Чтение состояния дискретных входов
	, MODBUS_CMD_READ_HOLDING_REGS 			= 0x03 // Чтение содержания регистров
	, MODBUS_CMD_READ_INPUT_REGS 			= 0x04 // Чтение содержания входных регистров
	, MODBUS_CMD_WRITE_SINGLE_COIL 			= 0x05 // Уствновка единичного выхода в ON или OFF
	, MODBUS_CMD_WRITE_SINGLE_REG 			= 0x06 // Запись в единичный регистр
	, MODBUS_CMD_READ_DEVICE_STATUS			= 0x07 // Чтение статуса устройства
	, MODBUS_CMD_WRITE_MULTI_COILS 			= 0x0F // Установка множества выходов в ON или OFF
	, MODBUS_CMD_WRITE_MULTI_REGS 			= 0x10 // Запись в несколько регистров
	, MODBUS_CMD_READ_DEVICE_INFO			= 0x11 // Чтение информации об устройстве
} ModbusCmd_t;

typedef enum
{
	  NoError								= 0
	, ErrInvalidFunctionCode 				= 1
	, ErrInvalidDataAddress					= 2
	, ErrInvalidDataValue					= 3
	, ErrInvalidFunctionExecute				= 4
		
	, ErrMasterInvalidDataStructPtr 		= 40
	, ErrSlaveNotResponse					= 41
	, ErrBadCRC								= 42
	, ErrResponseNotValid					= 43
} MbErrorCode_t;

#endif // _MODBUS_H_
