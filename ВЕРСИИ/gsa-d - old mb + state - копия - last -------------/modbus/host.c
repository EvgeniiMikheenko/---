//=============================================================================
//
//=============================================================================

#include "host.h"
#include <modbus_slave.h>
//#include "lpc17xx_uart.h"
#include <string.h>
//#include <soft_pwm.h>
#include <app_types.h>
#include <stm32f10x_tim.h>


#define MB_ADDR 						1
#define RXNE								0x20
#define TXE									0x80
#define Tim_Period					24000*2
#define	CPUFreq							24000000


extern Params_Struct Params;
extern int mb_read_flag;

uint8_t m_mbRxBuf[MOBUS_MAX_PACKET_SIZE];
uint8_t m_mbTxBuf[MOBUS_MAX_PACKET_SIZE];

void Timer3_init(void);

void usart_init(uint32_t baudrate, uint8_t stopBits);
void usart_send(uint8_t data);
void set_rs485_led(bool value);
void set_rs485_txen(bool value);
void get_device_info(uint8_t* lpDstBuf, uint32_t bufSize, uint32_t *lpWrLen);

void create_rw_regs_info(uint32_t regsCount);
bool test_reg_write_access(uint32_t num);
void init_regs_write_access(void);
void set_regs_write_accsess_enable(uint32_t num, bool enable);








uint32_t *m_lpRwRegsMode = NULL;
int m_RwRegsModeLen = 0;

MbSlaveParam_t m_mbParam =  {
	MB_ADDR,				// uint8_t 	address;						/* Адрес устройства */
	1, 						// uint8_t 	stopBits;						/* Количество стоп бит */
	5, 						// uint32_t 	serialBaudrate;				/* Скорость порта */
	m_mbRxBuf, 				// uint8_t 	*rxBuf;							/* Указатель на буфер приема */
	MOBUS_MAX_PACKET_SIZE, 	// uint32_t	rxBufSize;						/* Размер буфера приема */
	0, 						// uint32_t	rxCount;						/* Количество принятых данных */
	m_mbTxBuf, 				// uint8_t 	*txBuf;							/* Указатель на буфер передачи */
	MOBUS_MAX_PACKET_SIZE, 	// uint32_t	txBufSize;						/* Размер буфера передачи */
	0, 						// uint32_t	txCount;						/* Количество байт для передачи */
	0, 						// uint32_t	txIndex;
	0, 						// uint32_t	timeLeft;						/* Прошло времени с момента приема последнего байта */
	0, 						// uint32_t	timeout;						/* Значение таймаута после окончания тразакции  */		
	0, 						// uint32_t	hostTimeOut;					/* Значение защитного таймаута связи с хостом */
	0,  					// uint32_t	hostTimeLeft;					/* Прошло времени с момента последней транзакции */	
	usart_init, 			// UartInit	lpUartInit;						/* Указатель на функцию инициализации порта */
	usart_send, 			// UartSend	lpUartSend;						/* Указатель на функцию отправки одного байта */
	get_device_info, 		// GetDeviceInfo lpGetDeviceInfo;			/* Указатель на функцию получения информации об устройстве */
	0, //Board_SetLedRS485,		// SetBit		lpSetLed;					/*  */
	0, //Board_SetRS485_TxEn, 	// SetBit		lpSetTxEn;					/* Указатель на функцию вкл/выкл вывода TXEN RS485 */
	test_reg_write_access,
	
	PasswordRegNum, 		// int			passwordRegNum;					/* номер начального регистра для пароля */
	InfoSizeRegNum,			// int			deviceInfoSizeRegNum;
	InfoRegNum,				// int			deviceInfoRegNum;				/* номер начального регистра для информации об устройстве */
	
	MODBUS_PASSWORD,		// const char* password;
	MODBUS_PASSWORD_LEN,	// int			passwordLen;
//	m_mbCrcInfo, 			// Crc16Info_t crcInfo;						/* Данные для рассчета CRC16 */
//	
//	0, 						// uint32_t	errorCount;						/* количество ошибок протокола*/
//	
//	(uint16_t*)&Params, 		// uint16_t	*lpRegisters;				/* Указатель на буфер регистров */
//	0, 						// ???????? uint16_t	registersCount;		/* количество регистров */
//	
//	m_mbConfIndexes, 		// ConfigRegsIndexes_t regIndexes;			/* Индексы регистров конфигурации */
//	 
//	union {
//		uint32_t value;
//		struct {
//			unsigned enableTimer 			: 1;	/* Флаг активности таймера */
//			unsigned enableParseRxData 		: 1;	/* Флаг разрешения обработки принятого пакета */
//			unsigned isParsedStart 			: 1;	/* Флаг, указывающий, что обработка текущего пакета начата */
//			unsigned isRxDataMain 			: 1;	/* Флаг готовности принятых данных */
//			unsigned isHostTimeout 			: 1;	/* Флаг таймаута связи с хостом */
//			unsigned isConfigChangeLock 	: 1;	/* Флаг блокировки изменения конфигурации */
//			unsigned eepromNeedUpdate 		: 1;	/* Флаг необходимости обновления энергонезависимых данных */
//		};
//	} Flags;
	
};


void host_init(void) {
	m_mbParam.crcInfo.crc = 0;
	m_mbParam.crcInfo.counter = 0;
	
	m_mbParam.errorCount = 0;
	
	m_mbParam.lpRegisters = (uint16_t*)&Params;
	m_mbParam.registersCount = sizeof(Params) / 2;
	
	create_rw_regs_info(m_mbParam.registersCount);
	init_regs_write_access();
	
	m_mbParam.regIndexes.regAddrBaudrate = -1;
	m_mbParam.regIndexes.regAddrDeviceAddress = -1;
	m_mbParam.regIndexes.regAddrHostTimeout = -1;
	
	m_mbParam.Flags.value = 0;
	m_mbParam.Flags.isConfigChangeLock = 1;
	
	mb_slave_init(&m_mbParam);
	Timer3_init();
}

void host_reset(void) {
	mb_slave_stop_tx(&m_mbParam);
}

void Timer3_init(void) {
	
	
	TIM_TimeBaseInitTypeDef  TIM_TimeBase;
	TIM_OCInitTypeDef  TIM_OCInit;
	
	TIM_TimeBase.TIM_Period = Tim_Period;
	TIM_TimeBase.TIM_Prescaler = 0;
	TIM_TimeBase.TIM_ClockDivision = 0x0;
	TIM_TimeBase.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBase.TIM_RepetitionCounter = 0x0;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBase);
	
	TIM_OCInit.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInit.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInit.TIM_Pulse = 12000;
	TIM_OCInit.TIM_OCPolarity = TIM_OCPolarity_High;
	
	
//	TIM_OC2Init(TIM4, &TIM_OCInit);

//	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM4->DIER=1;				//Update interrupt enable
//	TIM_Cmd(TIM4, ENABLE);
	
	
}







void host_timer_tick(uint32_t us) {
	
	
	mb_slave_timer_tick(&m_mbParam, us);
}

void usart_init(uint32_t baudrate, uint8_t stopBits) {
	
	USART_InitTypeDef USART_InitStructure;
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART1, ENABLE);
	
	
}

void get_device_info(uint8_t* lpDstBuf, uint32_t bufSize, uint32_t *lpWrLen) {
	
	if(lpWrLen != NULL)
		*lpWrLen = 0;
	
	if((lpDstBuf == NULL) || (bufSize == 0))
		return;
	
	int len = strlen(DEVICE_INFO);
	*lpWrLen = len * 2;
	
	int index = 0;
	char * str = DEVICE_INFO;
	for(int i = 0; i < len; i++) {
		index = i * 2;
		
		lpDstBuf[index] = 0x00;
		lpDstBuf[index + 1] = str[i];
	}
}

void usart_send(uint8_t data) {
	
	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	
	USART_SendData(USART1, data);
}

void create_rw_regs_info(uint32_t regsCount) {
	m_RwRegsModeLen = 0;
	int len = regsCount / 32;
	if((regsCount % 32) != 0)
		len++;
	
	if(m_lpRwRegsMode != NULL) {
		free(m_lpRwRegsMode);
		m_lpRwRegsMode = NULL;
	}
	
	m_lpRwRegsMode = (uint32_t*)malloc(len * sizeof(uint32_t));
	if(m_lpRwRegsMode != NULL) {
		memset(m_lpRwRegsMode, 0, len * sizeof(uint32_t));
		m_RwRegsModeLen = len;
	}
}

bool test_reg_write_access(uint32_t num) {
	
	if(m_lpRwRegsMode == NULL)
		return true;
	
	int index = num / 32;
	int bit = num % 32;
	uint32_t result;
	
	if(index >= m_RwRegsModeLen)
		return false;
	
	result = (m_lpRwRegsMode[index] & (1 << bit));
	return (result != 0);
}

void set_regs_write_accsess_enable(uint32_t num, bool enable) {
	if(m_lpRwRegsMode == NULL)
		return;
	
	int index = num / 32;
	int bit = num % 32;
	
	if(index >= m_RwRegsModeLen)
		return;
	
	if(enable)
		m_lpRwRegsMode[index] |= (1 << bit);
	else
		m_lpRwRegsMode[index] &= ~(1 << bit);
}

void init_regs_write_access(void) {
	
	// Разрешаем запись в регистры
	// Params.FPGA_Out_Struct
	// Params.CPU_In_Struct
	// 0 .. 126
	for(int i = 0; i < 127; i++) {
		set_regs_write_accsess_enable(i, true);
	}
	
}

void UART_IRQ (void) {
	uint32_t intsrc, tmp, tmp1;
	bool isOverflow = false;
	bool isFrameError = false;
	uint8_t data;
	uint32_t rLen;
 
    /* Determine the interrupt source */
	
		intsrc = USART1->SR;
    //intsrc = UART_GetIntId((LPC_UART_TypeDef*)LPC_UART1);
 //   tmp = intsrc & UART_IIR_INTID_MASK;
 
    // Receive Line Status
  
 
    // Receive Data Available or Character time-out
    if (tmp == RXNE){
		rLen = 	USART_ReceiveData(USART1);									//UART_Receive((LPC_UART_TypeDef*)LPC_UART1, &data, 1, NONE_BLOCKING);
		if(rLen != 0) {
			// Забираем данные
			mb_slave_rx_func(&m_mbParam, data, isFrameError, isOverflow);
			mb_read_flag = 1;
		}
    }
 
    // Transmit Holding Empty
    if (tmp == TXE){
		//UART_IntTransmit();
		if(m_mbParam.txBuf != NULL) {
			if(m_mbParam.txIndex < m_mbParam.txCount) {
				
				USART_SendData(USART1, m_mbParam.txBuf[m_mbParam.txIndex++]);
				//UART_SendByte((LPC_UART_TypeDef*)LPC_UART1, m_mbParam.txBuf[m_mbParam.txIndex++]);
			}
			else {
				mb_slave_stop_tx(&m_mbParam);
			}
		}
		else {
			mb_slave_stop_tx(&m_mbParam);
		}
    }
}

void TIM4_IRQHandler (void) {

	/*  Clear Interrupt */
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	/* Code...*/
	mb_slave_timer_tick(&m_mbParam, Tim_Period / CPUFreq);

}
