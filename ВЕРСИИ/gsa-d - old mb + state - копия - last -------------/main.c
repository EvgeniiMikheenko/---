#include "main.h"

char init=1;
int mb_addr, mb_sl_bytecount = 0, mb_sl_timer = 0, mb_sl_datasize = 0;
signed char mb_sl_mode = MB_RECEIVE_MODE, uart_free = 1, flash_write = 0, check4flash_write = 0, work_enable = 0, triac_on = 0, triac_on_prev = 0, Tset_in_flag = 0, Tallow_Tset_update = 1, Tdiff_zone = 0, Tdiff_zone_prev = 0;
unsigned mb_sl_timeout = 0;
int mb_read_flag = 0;
unsigned char mb_slave_buf[MB_SLAVE_BUFSIZE];

unsigned short mb_input_params[MB_MAX_INPUT_PARAMS + 1];

#define mb_addr mb_slave_buf[0]
#define mb_func mb_slave_buf[1]



#define STATE			1
#define SWITCH_STATE	4

#ifdef STATE

#pragma pack (push, 1)
typedef struct
{
	unsigned short HV_Value;
	unsigned short Freq;
	unsigned short HV_Offset;
	unsigned short Pos_Tresh;
	unsigned short Neg_Tresh;
	unsigned short Set_Null;
	unsigned short Pos_Tresh_Danger;
	unsigned short Neg_Tresh_Danger;
	unsigned short Filter_Level;
	unsigned short Zone_timer;
	unsigned short Dummy[9];	
	
//	unsigned short Pos_Value;
//	unsigned short Neg_Value;
	unsigned short Gummy[2];
}	Settings;



typedef struct
{
	unsigned short Flags_Write;
	Settings Set[3];
//	unsigned short Filter_Level;	
	unsigned short crc;	
	unsigned short src_num[2];	
	unsigned short Date[3];
//unsigned short Dummy[10];	
	unsigned short Flags_Read;	
	unsigned short Pos_Value;
	unsigned short Neg_Value;
	unsigned short HV;
	unsigned short Ass;	
	unsigned short Temp_Int;			// 0x4B
	unsigned short ID1;
	unsigned short ID2;
	unsigned short ID3;
	unsigned short ID4;
	unsigned short ID5;
	unsigned short ID6;
	unsigned short hash[8];
	unsigned short ver;					//0x5A	
	//unsigned short Dummy1[7];
	unsigned short Zone_ch_off;	//0x5B				Zone_ch_of == 1 -> не переключаем зоны
	unsigned short Zone_num;		
	unsigned short Dummy1[5];							
	unsigned short State;
	unsigned short Device;			//0x63
	unsigned short baudrate;			//0x64
	unsigned short databits;
	unsigned short stopbits;
	unsigned short parity;
	unsigned short dev_addr;
	unsigned short crc2;
	
		
} Params_Struct;

#pragma pack(pop)
#else




typedef struct
{
	unsigned short Flags_Write;
	unsigned short HV_Value;
	unsigned short Freq;
	unsigned short HV_Offset;
	unsigned short Pos_Tresh;
	unsigned short Neg_Tresh;
	unsigned short Set_Null;
	unsigned short Pos_Tresh_Danger;
	unsigned short Neg_Tresh_Danger;
	unsigned short Filter_Level;
	unsigned short crc;
	unsigned short src_num[2];
	unsigned short Date[3];
	unsigned short Dummy[10];
	unsigned short Pos2_Tresh;
	unsigned short Neg2_Tresh;
	unsigned short Pos2_Tresh_Danger;
	unsigned short Neg2_Tresh_Danger;

	unsigned short Flags_Read;
	unsigned short Pos_Value;			//		31
	unsigned short Neg_Value;			//
	unsigned short HV;						//
	unsigned short Ass;						//
	unsigned short Temp_Int;			//
	unsigned short ID1;
	unsigned short ID2;
	unsigned short ID3;
	unsigned short ID4;
	unsigned short ID5;
	unsigned short ID6;
	unsigned short hash[8];
	unsigned short ver;
	unsigned short Pos2_Value;
	unsigned short Neg2_Value;
	unsigned short Dummy1[7];
	
	
	unsigned short	HV2_Value;
	unsigned short	Freq2;
	unsigned short	HV2_Offset;
	unsigned short	Set2_Null;
	
	
	unsigned short	Neg3_Tresh;
	unsigned short	Pos3_Tresh;
	unsigned short 	Pos3_Tresh_Danger;
	unsigned short	Neg3_Tresh_Danger;
	unsigned short	Pos3_Value;
	unsigned short	Neg3_Value;
	unsigned short	HV3_Offset;
	unsigned short	Set3_Null;
	
	unsigned short	HV3_Value;
	unsigned short	Freq3;
	
	
} Params_Struct;
#endif							//STATE



Params_Struct Params;



typedef struct
{

	unsigned short baudrate;			//64
	unsigned short databits;
	unsigned short stopbits;
	unsigned short parity;
	unsigned short dev_addr;
	unsigned short crc2;

} Net_Struct;

Net_Struct Net_Params;








unsigned *UNIQUE_ID1 =(unsigned*)0x1FFFF7E8;
unsigned *UNIQUE_ID2 =(unsigned*)0x1FFFF7EC;


unsigned short * PARAMS_BUF = (unsigned short *) & Params;
unsigned short * FLASH_PARAMS_BUF = (unsigned short *) FLASH_LOCATION;

unsigned short * FLASH_PARAMS_BUF_2 = (unsigned short *) FLASH_LOCATION_2;


Params_Struct * Flash_Params = (Params_Struct *) FLASH_LOCATION;
Net_Struct * Flash_Params2 = (Net_Struct *) FLASH_LOCATION_2;
//Params_Struct * Flash_Params2 = (Params_Struct *) FLASH_LOCATION_2;

typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
u32 EraseCounter = 0x00, Address = 0x00;
u32 Data;
vu32 NbrOfPage = 0x00;
volatile FLASH_Status FLASHStatus;
volatile TestStatus MemoryProgramStatus;
ErrorStatus HSEStartUpStatus;


unsigned * BUF1 = (unsigned *) &Params;
unsigned * BUF1_2 = ( unsigned *) &Net_Params;
//unsigned * BUF1_2 = ( unsigned *) &Params.baudrate;
unsigned * BUF2 = (unsigned *) FLASH_LOCATION;
unsigned * BUF2_2 = (unsigned *) FLASH_LOCATION_2;



void FlashStore( void );
int FlashRestore( void );
void RCC_Configuration( void );
void NVIC_Configuration( void );
void USART_ReInit(Params_Struct *Params );		// перезапускк UART с новыми параметрами сети 
void Net_SettingsRestore( Params_Struct *Param, Net_Struct *Net );
void	Net_SettingsUpdate( Params_Struct *Param, Net_Struct *Net );

int adc_channel=0;
int Pos_Value_int=0;
int Neg_Value_int=0;
int Ehv_Value_int=0;
int Eass_Value_int=0;
int Temp_Value_int=0;
int Dev_ID=0;
int ready_timer=0;
	
	#ifdef		STATE
	#define NUM_OF_STATE	1
	uint16_t state_counter = 0;
	uint8_t state1 = 0;
	#endif			// STATE


USART_InitTypeDef USART_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;
ADC_InitTypeDef ADC_InitStructure;

TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
u16 CCR1_Val = 32768;
u16 CCR2_Val = 375;
u16 CCR3_Val = 250;
u16 CCR4_Val = 125;
int i;
int main(void)
{
#ifdef DEBUG
	debug();
#endif

	/* <hash>  */
	unsigned char *buffer, digest[16];
	int trbr=0;
	MD5_CTX context;
	buffer = (unsigned char*)0x08000000;

	MD5Init (&context);
	MD5Update (&context,buffer, (0x0801FFFF-0x08000000));
	MD5Final (digest, &context);
	while(trbr<8)
	{
		Params.hash[trbr] = (unsigned short)digest[trbr*2]<<8|(unsigned short)digest[trbr*2+1];	//2 char in 1 ushort
		trbr++;
	}
	/* </hash> */


	
	
	
	
	
	Params.ver = 0x0102;
	Params.Device = 0x01;

	
	
	RCC_Configuration();

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16|RCC_APB2Periph_TIM17| RCC_APB2Periph_USART1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 |RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA| RCC_APB2Periph_GPIOB| RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE);

	AFIO->MAPR|=AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1|AFIO_MAPR_TIM3_REMAP_PARTIALREMAP|AFIO_MAPR_SWJ_CFG_JTAGDISABLE;


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12;			//P_I		P_F
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed =  GPIO_Speed_2MHz;

	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6;		//EASS		EHV		EMU1		EMU2
	GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Speed =  GPIO_Speed_2MHz;

	GPIO_Init(GPIOA, &GPIO_InitStructure);


	/* Configure USART1 Tx (PA.09) as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;			//Tx
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART1 Rx (PA.10) as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;			//Rx
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_9;					//--		CTRAS		CTR2U		0_CR
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;		//#RE		DE		KMAX		--
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_13;				//--
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_15;			//--
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* ADC1 and ADC2 operate independantly */
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	/* Disable the scan conversion so we do one at a time */
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	/* Don't do contimuous conversions - do them on demand */
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	/* Start conversin by software, not an external trigger */
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	/* Conversions are 12 bit - put them in the lower 12 bits of the result */
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
	/* Say how many channels would be used by the sequencer */
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC1->CR2|=(1<<23);

	/* Now do the setup */
	ADC_Init(ADC1, &ADC_InitStructure);
	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/* Enable ADC1 reset calibaration register */
	ADC_ResetCalibration(ADC1);
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));
	/* Start ADC1 calibaration */
	ADC_StartCalibration(ADC1);
	/* Check the end of ADC1 calibration */


	/* TIM2 configuration */
	TIM_TimeBaseStructure.TIM_Period = 24000*2;
	TIM_TimeBaseStructure.TIM_Prescaler = 0;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 12000;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC2Init(TIM2, &TIM_OCInitStructure);

	TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM2->DIER=1;
	TIM_Cmd(TIM2, ENABLE);

	TIM_TimeBaseStructure.TIM_Period = 24000;
	TIM_TimeBaseStructure.TIM_Prescaler = 3;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;












	DE_OFF;
	RE_ON;
	READY_OFF;
		
		
	
		
		
		
		
		
		
		
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



	

	
	
	


	
	
	NVIC_Configuration();
	



	


	
	while (1)
	{
	}
}














	






void RCC_Configuration(void)
{
	/* RCC system reset(for debug purpose) */
	RCC_DeInit();

	/* Enable Prefetch Buffer */
	FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);

	/* Flash 2 wait state */
	FLASH_SetLatency(FLASH_Latency_2);

	/* HCLK = SYSCLK */
	RCC_HCLKConfig(RCC_SYSCLK_Div1);

	/* PCLK2 = HCLK */
	RCC_PCLK2Config(RCC_HCLK_Div1);

	/* PCLK1 = HCLK/2 */
	RCC_PCLK1Config(RCC_HCLK_Div2);

	/* ADCCLK = PCLK2/4 */
	RCC_ADCCLKConfig(RCC_PCLK2_Div4);

	/* PLLCLK = 8MHz * 7 = 56 MHz */
	RCC_PLLConfig(RCC_PLLSource_HSI_Div2,RCC_PLLMul_6);

	/* Enable PLL */
	RCC_PLLCmd(ENABLE);

	/* Wait till PLL is ready */
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET);

	/* Select PLL as system clock source */
	RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

	/* Wait till PLL is used as system clock source */
	while(RCC_GetSYSCLKSource() != 0x08);

}


void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;

#ifdef  VECT_TAB_RAM
	/* Set the Vector Table base location at 0x20000000 */
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
	/* Set the Vector Table base location at 0x08000000 */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif


	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the TIM2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

char in_byte;
void USART1_IRQHandler(void)
{
	unsigned int	stat, crc, i;
	unsigned int	startaddr, count;
	char			stat_rs485;
	char			stat_rs232, mb_lsr;

	stat_rs485 = (RS485_IIR);

	if ((stat_rs485&0x80))
		stat=0x2;

	if (stat_rs485&0x8) i=MB_RBR;
	if (stat_rs485&0x20) stat=0x4;
	stat_rs485 = (USART1->CR1);

	switch (stat)
	{
		case 0x2:
			if ((MB_LSR & 0x80)  && (mb_sl_bytecount < mb_sl_datasize))
			{
				RS485_THR = mb_slave_buf[mb_sl_bytecount++];
			}
			if (mb_sl_bytecount == mb_sl_datasize)
			{
				mb_sl_timer = MB_RCVPAUSE; // ??
				mb_sl_mode = MB_RECEIVE_MODE;
				mb_sl_bytecount = 0;
				mb_sl_datasize = 0;
				USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
			}
			break;
		case 0xC:
		case 0x4:
			in_byte = MB_RBR;
			switch (mb_sl_bytecount)
			{
				case 0:
					if (in_byte == Params.dev_addr || in_byte == MB_ADDR_REQUEST)				//if (in_byte == MB_ADDR || in_byte == MB_ADDR_REQUEST)
						mb_addr = in_byte;
					else
						mb_sl_bytecount = - 1;
					break;
				case 1:
					mb_func = in_byte;
					if ((mb_func != 4) && (mb_func != 3) && (mb_func != 15) && (mb_func != 16) && (mb_func != 6) && (mb_func != 1) && (mb_func != 2) && (mb_func != 5))
						mb_sl_bytecount = - 1;
					else
						mb_sl_datasize = 8;
					break;
				default:
					mb_slave_buf[mb_sl_bytecount] = in_byte;
					if ((mb_sl_bytecount == 6) && ((mb_func == 16) || (mb_func == 15)))
						mb_sl_datasize = 9 + mb_slave_buf[6];
					if (mb_sl_datasize - 2 < mb_sl_bytecount)
					{
						mb_sl_bytecount = - 1;
						if (CRC16((unsigned char *) & mb_slave_buf, mb_sl_datasize))
						{
							mb_sl_datasize = 0;
							break;
						}
						RE_OFF;
						USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
						startaddr = mb_slave_buf[3] + (mb_slave_buf[2] << 8);
						count = mb_slave_buf[5] + (mb_slave_buf[4] << 8);
						switch (mb_func)
						{
							case 5:
								if ((startaddr > 7777) || mb_sl_timer)
								{
									mb_func = 0x85;
									mb_slave_buf[2] = 2;
									mb_sl_datasize = 3;
								}
								else
								{
									mb_sl_datasize = 6;
									i = (mb_slave_buf[5] + (mb_slave_buf[4] << 8));
								}
								break;
							case 6:
								if (((startaddr) > MB_MAXPARAMS - MB_MAX_INPUT_PARAMS) || mb_sl_timer)
								{
									mb_func = 0x86;
									mb_slave_buf[2] = 2;
									mb_sl_datasize = 3;
								}
								else
								{
									mb_sl_datasize = 6;
									PARAMS_BUF[startaddr] = mb_slave_buf[5] + (mb_slave_buf[4] << 8);
								}
								break;
							case 3:
								if (((startaddr + count) > MB_MAXPARAMS) || ((startaddr + count) < 1) || mb_sl_timer)
								{
									mb_func = 0x80 + 3;
									mb_slave_buf[2] = 2;
									mb_sl_datasize = 3;
								}
								else
								{
									mb_slave_buf[2] = count << 1;
									mb_sl_datasize = 3 + mb_slave_buf[2];
									for (i = 0; i <= count; i++)
									{
										if ((startaddr + i) == 0)
										mb_read_flag = 1;
										mb_slave_buf[3 + (i << 1)] = (int) PARAMS_BUF[startaddr + i] >> 8;
										mb_slave_buf[4 + (i << 1)] = PARAMS_BUF[startaddr + i];
									}
								}
								break;
							case 4:
								if (((startaddr + count) > MB_MAX_INPUT_PARAMS) || ((startaddr + count) < 1) || mb_sl_timer)
								{
									mb_func = 0x80 + 4;
									mb_slave_buf[2] = 2;
									mb_sl_datasize = 3;
								}
								else
								{
									mb_slave_buf[2] = count << 1;
									mb_sl_datasize = 3 + mb_slave_buf[2];
									for (i = 0; i <= count; i++)
									{
										mb_slave_buf[3 + (i << 1)] = (int) mb_input_params[startaddr + i] >> 8;
										mb_slave_buf[4 + (i << 1)] = mb_input_params[startaddr + i];
									}
								}
								break;
							case 16:
								if (((startaddr + count) > MB_MAXPARAMS) || ((startaddr + count) < 1) || mb_sl_timer)
								{
									mb_func = 0x80 + 16;
									mb_slave_buf[2] = 2;
									mb_sl_datasize = 3;
								}
								else
								{
									count = (mb_slave_buf[4] << 8) + mb_slave_buf[5];
									for (i = 0; i < count; i++) {
										PARAMS_BUF[i + startaddr] = ((mb_slave_buf[7 + (i << 1)] << 8) + mb_slave_buf[8 + (i << 1)]);
									}
									if (0)
									{
										mb_func = 0x80 + 16;
										mb_slave_buf[2] = 4;
										mb_sl_datasize = 3;
									}
									else
										mb_sl_datasize = 6;
								}
								break;
							default:
								mb_func = 0x80 + mb_func;
								mb_slave_buf[2] = 1;
								mb_sl_datasize = 3;
						}
						crc = CRC16(mb_slave_buf, mb_sl_datasize);
						mb_slave_buf[mb_sl_datasize++] = crc;
						mb_slave_buf[mb_sl_datasize++] = crc >> 8;
						mb_sl_timer = MB_SENDPAUSE;
						mb_sl_mode = MB_SEND_MODE;
						mb_sl_timeout = 0;
					}
					}
					mb_sl_bytecount++;
					break;
		default:
			break;
	}
}






void TIM2_IRQHandler(void)
{
	
	

	if (mb_sl_timer)
	{
		if (--mb_sl_timer == 0)
		{
			if (mb_sl_mode == MB_SEND_MODE)
			{
				DE_ON;
				USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
				if ((MB_LSR & 0x80) && (mb_sl_bytecount < mb_sl_datasize))
				{
					RS485_THR = mb_slave_buf[mb_sl_bytecount++];
				}
			}
			else
			{
				DE_OFF;
				RE_ON;
				USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
			}
		}
	}

	
	
	
	
	
	

	
	
	
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}















#ifdef  DEBUG
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(u8* file, u32 line)
{
	/* User can add his own implementation to report the file name and line number,
	ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

	/* Infinite loop */
	while (1)
	{
	}
}

#endif



/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/





