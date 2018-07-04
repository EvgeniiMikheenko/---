//=============================================================================
//							modbus_slave.h
//=============================================================================

#ifndef _MODBUS_SLAVE_H_
#define _MODBUS_SLAVE_H_

#include "modbus.h"
#include "crc16.h"


#pragma anon_unions
//#include "lpc17xx_uart.h"
//#include "lpc17xx_timer.h"

#define READ_HOLDING_REGISTERS_DATA_INDEX	

typedef void (*UartInit)(uint32_t baudrate, uint8_t stopBits);
typedef void (*UartSend)(uint8_t data);
typedef void (*SetBit)(bool value);
typedef void (*GetDeviceInfo)(uint8_t* lpDstBuf, uint32_t bufSize, uint32_t *lpWrLen);
typedef bool (*GetRegWriteAccess)(uint32_t num);

#pragma pack(push, 1)
//typedef union {
//	uint8_t buf[MOBUS_MAX_PACKET_SIZE];
//	
//	struct {
//		uint8_t 	addr;
//		uint8_t 	func;
//	} Device;
//	
//	struct {
//		uint8_t 	addr;
//		uint8_t 	func;
//		uint16_t 	startAddr;
//		uint16_t 	count;
//	} RdHoldingRegs;
//	
//	struct {
//		uint8_t 	addr;
//		uint8_t 	func;
//		uint16_t 	regAddr;
//		uint16_t	data;
//	} WrSingleReg;
//	
//	struct {
//		uint8_t 	addr;
//		uint8_t 	func;
//		uint16_t 	startAddr;
//		uint16_t 	count;
//		uint8_t		byteCount;
//	} WrMultRegs;
//	
//	struct {
//		uint8_t 	addr;
//		uint8_t 	func;
//		uint8_t		errorCode;
//	} Exception;
//	
//} ModbusPacket_t, *lpModbusPacket_t;

typedef struct {
	int	regAddrDeviceAddress;			/* ������ �������� ������ ���������� */
	int	regAddrBaudrate;				/* ������ �������� �������� ����� */
	int regAddrHostTimeout;				/* ������ �������� �������� ����� � ������ */
} ConfigRegsIndexes_t, *lpConfigRegsIndexes_t;

typedef struct {
	uint8_t 	address;						/* ����� ���������� */
	uint8_t 	stopBits;						/* ���������� ���� ��� */
	uint32_t 	serialBaudrate;					/* �������� ����� */
	//
	uint8_t 	*rxBuf;							/* ��������� �� ����� ������ */
	uint32_t	rxBufSize;						/* ������ ������ ������ */
	uint32_t	rxCount;						/* ���������� �������� ������ */
	
	uint8_t 	*txBuf;							/* ��������� �� ����� �������� */
	uint32_t	txBufSize;						/* ������ ������ �������� */
	uint32_t	txCount;						/* ���������� ���� ��� �������� */
	uint32_t	txIndex;
	//
	uint32_t	timeLeft;						/* ������ ������� � ������� ������ ���������� ����� */
	uint32_t	timeout;						/* �������� �������� ����� ��������� ���������  */
	//		
	uint32_t	hostTimeOut;					/* �������� ��������� �������� ����� � ������ */
	uint32_t	hostTimeLeft;					/* ������ ������� � ������� ��������� ���������� */	
	
	UartInit	lpUartInit;						/* ��������� �� ������� ������������� ����� */
	UartSend	lpUartSend;						/* ��������� �� ������� �������� ������ ����� */
	GetDeviceInfo lpGetDeviceInfo;				/* ��������� �� ������� ��������� ���������� �� ���������� */
	
	SetBit		lpSetLed;						/*  */
	SetBit		lpSetTxEn;						/* ��������� �� ������� ���/���� ������ TXEN RS485 */
	
	GetRegWriteAccess lpGetRegWriteAccess;
	
	int			passwordRegNum;					/* ����� ���������� �������� ��� ������ */
	int			deviceInfoSizeRegNum;
	int			deviceInfoRegNum;				/* ����� ���������� �������� ��� ���������� �� ���������� */
	
	const char* password;
	int			passwordLen;
	
	Crc16Info_t crcInfo;						/* ������ ��� �������� CRC16 */
	
	uint32_t	errorCount;						/* ���������� ������ ���������*/
	
	uint16_t	*lpRegisters;					/* ��������� �� ����� ��������� */
	uint16_t	registersCount;					/* ���������� ��������� */
	
	ConfigRegsIndexes_t regIndexes;				/* ������� ��������� ������������ */
	 
	union {
		uint32_t value;
		struct {
			unsigned enableTimer 			: 1;	/* ���� ���������� ������� */
			unsigned enableParseRxData 		: 1;	/* ���� ���������� ��������� ��������� ������ */
			unsigned isParsedStart 			: 1;	/* ����, �����������, ��� ��������� �������� ������ ������ */
			unsigned isRxDataMain 			: 1;	/* ���� ���������� �������� ������ */
			unsigned isHostTimeout 			: 1;	/* ���� �������� ����� � ������ */
			unsigned isConfigChangeLock 	: 1;	/* ���� ���������� ��������� ������������ */
			unsigned eepromNeedUpdate 		: 1;	/* ���� ������������� ���������� ����������������� ������ */
			unsigned isPasswordValid		: 1;   	/* ���� ����������� ������ */
		};
	} Flags;
	
} MbSlaveParam_t, *lpMbSlaveParam_t;

#pragma pack(pop)

typedef enum
{
	bps9600 	= 0,
	bps14400 	= 1,
	bps19200 	= 2,
	bps38400 	= 3,
	bps57600 	= 4,
	bps115200 	= 5
} usart_baudrates;

void mb_slave_init(lpMbSlaveParam_t lpParam);

void mb_slave_rx_func(lpMbSlaveParam_t lpParam, uint8_t data, bool frameError, bool overflowError);

void mb_slave_stop_tx(lpMbSlaveParam_t lpParam);

void mb_slave_send_error(lpMbSlaveParam_t lpParam, uint8_t func, uint8_t errorCode);

void mb_slave_start_tx(lpMbSlaveParam_t lpParam, uint32_t size);

void mb_slave_parce_packet(lpMbSlaveParam_t lpParam);

uint32_t mb_slave_convert_baudrate(uint16_t regValue);

uint16_t mb_baudrate_to_reg_value(uint32_t baudrate);

void mb_slave_timer_tick(lpMbSlaveParam_t lpParam, uint32_t us);


#define STOP_TIMER_ENABLE										1


#ifdef STOP_TIMER_ENABLE

void Timer_stop( void );
void Timer_start( void );

#endif















#endif // _MODBUS_SLAVE_H_
