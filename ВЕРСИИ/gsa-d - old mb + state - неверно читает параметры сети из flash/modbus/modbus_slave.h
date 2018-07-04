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
	int	regAddrDeviceAddress;			/* Индекс регистра адреса устройства */
	int	regAddrBaudrate;				/* Индекс регистра скорости порта */
	int regAddrHostTimeout;				/* Индекс регистра таймаута связи с хостом */
} ConfigRegsIndexes_t, *lpConfigRegsIndexes_t;

typedef struct {
	uint8_t 	address;						/* Адрес устройства */
	uint8_t 	stopBits;						/* Количество стоп бит */
	uint32_t 	serialBaudrate;					/* Скорость порта */
	//
	uint8_t 	*rxBuf;							/* Указатель на буфер приема */
	uint32_t	rxBufSize;						/* Размер буфера приема */
	uint32_t	rxCount;						/* Количество принятых данных */
	
	uint8_t 	*txBuf;							/* Указатель на буфер передачи */
	uint32_t	txBufSize;						/* Размер буфера передачи */
	uint32_t	txCount;						/* Количество байт для передачи */
	uint32_t	txIndex;
	//
	uint32_t	timeLeft;						/* Прошло времени с момента приема последнего байта */
	uint32_t	timeout;						/* Значение таймаута после окончания тразакции  */
	//		
	uint32_t	hostTimeOut;					/* Значение защитного таймаута связи с хостом */
	uint32_t	hostTimeLeft;					/* Прошло времени с момента последней транзакции */	
	
	UartInit	lpUartInit;						/* Указатель на функцию инициализации порта */
	UartSend	lpUartSend;						/* Указатель на функцию отправки одного байта */
	GetDeviceInfo lpGetDeviceInfo;				/* Указатель на функцию получения информации об устройстве */
	
	SetBit		lpSetLed;						/*  */
	SetBit		lpSetTxEn;						/* Указатель на функцию вкл/выкл вывода TXEN RS485 */
	
	GetRegWriteAccess lpGetRegWriteAccess;
	
	int			passwordRegNum;					/* номер начального регистра для пароля */
	int			deviceInfoSizeRegNum;
	int			deviceInfoRegNum;				/* номер начального регистра для информации об устройстве */
	
	const char* password;
	int			passwordLen;
	
	Crc16Info_t crcInfo;						/* Данные для рассчета CRC16 */
	
	uint32_t	errorCount;						/* количество ошибок протокола*/
	
	uint16_t	*lpRegisters;					/* Указатель на буфер регистров */
	uint16_t	registersCount;					/* количество регистров */
	
	ConfigRegsIndexes_t regIndexes;				/* Индексы регистров конфигурации */
	 
	union {
		uint32_t value;
		struct {
			unsigned enableTimer 			: 1;	/* Флаг активности таймера */
			unsigned enableParseRxData 		: 1;	/* Флаг разрешения обработки принятого пакета */
			unsigned isParsedStart 			: 1;	/* Флаг, указывающий, что обработка текущего пакета начата */
			unsigned isRxDataMain 			: 1;	/* Флаг готовности принятых данных */
			unsigned isHostTimeout 			: 1;	/* Флаг таймаута связи с хостом */
			unsigned isConfigChangeLock 	: 1;	/* Флаг блокировки изменения конфигурации */
			unsigned eepromNeedUpdate 		: 1;	/* Флаг необходимости обновления энергонезависимых данных */
			unsigned isPasswordValid		: 1;   	/* Флаг правильного пароля */
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
