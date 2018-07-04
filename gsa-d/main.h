
#include <stm32f10x.h>
#include <stm32f10x_flash.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_adc.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_dac.h>
#include <misc.h>
#include "md5.h"


#define READY_TIMER_VALUE		30*1000 //30 sec

#define DEV_ADC_EDELWEIS		0
#define DEV_ADC_ASTRA			0
#define DEV_ADC_LIMB				((330<<16)/3300)
#define DELTA_ADC_DEV			((50<<16)/3300)
#define DEV_EDELWEIS				1
#define DEV_ASTRA					2
#define DEV_LIMB					3


#define PWM_PULSE_LIMB			(24000*2)

#define LIMB_TEST_VAL1			(500*PWM_PULSE_LIMB/3300)
#define LIMB_TEST_VAL2			(3000*PWM_PULSE_LIMB/3300)
#define LIMB_TEST_VAL3			(2500*PWM_PULSE_LIMB/3300)
#define LIMB_TEST_VAL4			(2000*PWM_PULSE_LIMB/3300)
#define LIMB_TEST_VAL5			(1500*PWM_PULSE_LIMB/3300)
#define LIMB_TEST_VAL6			(1000*PWM_PULSE_LIMB/3300)

#define LIMB_NONE_VAL			(1000*PWM_PULSE_LIMB/3300)
#define LIMB_PI_VAL				(1500*PWM_PULSE_LIMB/3300)
#define LIMB_PF_VAL				(2000*PWM_PULSE_LIMB/3300)
#define LIMB_PI_PF_VAL			(2500*PWM_PULSE_LIMB/3300)



#define FLAG_FILTER				(Params_reg.Flags_Write&4)

#define FLASH_PAGE_SIZE			((u16)0x400)
#define FLASH_LOCATION			((u32)0x08008000-FLASH_PAGE_SIZE)
#define EndAddr					((u32)0x08008000)
#define FLASH_SIZE				(((unsigned short *)&Params_reg.crc - (unsigned short *) & Params_reg)/2+1)


#define DE_ON						GPIO_SetBits(GPIOB,GPIO_Pin_12)
#define DE_OFF						GPIO_ResetBits(GPIOB,GPIO_Pin_12)

#define RE_ON						GPIO_ResetBits(GPIOB,GPIO_Pin_13)
#define RE_OFF						GPIO_SetBits(GPIOB,GPIO_Pin_13)

#define READY_ON					Params_reg.Flags_Read|=32;if(Dev_ID==DEV_EDELWEIS)GPIO_SetBits(GPIOA,GPIO_Pin_15)
#define READY_OFF					Params_reg.Flags_Read&=~32;if(Dev_ID==DEV_EDELWEIS)GPIO_ResetBits(GPIOA,GPIO_Pin_15)
#define FAILURE_ON				Params_reg.Flags_Read|=16
#define FAILURE_OFF				Params_reg.Flags_Read&=~16

#define PI_DANGER_OFF			(GPIO_ResetBits(GPIOA,GPIO_Pin_12),Params_reg.Flags_Read&=~2)
#define PF_DANGER_OFF			(GPIO_ResetBits(GPIOA,GPIO_Pin_11),Params_reg.Flags_Read&=~1)

#define PI_DANGER_ON				(GPIO_SetBits(GPIOA,GPIO_Pin_12),GPIO_ResetBits(GPIOA,GPIO_Pin_11),Params_reg.Flags_Read|=2)
#define PF_DANGER_ON				(GPIO_SetBits(GPIOA,GPIO_Pin_11),GPIO_ResetBits(GPIOA,GPIO_Pin_12),Params_reg.Flags_Read|=1)

#define PI_TRESH_OFF				Params_reg.Flags_Read&=~8
#define PF_TRESH_OFF				Params_reg.Flags_Read&=~4

#define PI_TRESH_ON				Params_reg.Flags_Read|=8
#define PF_TRESH_ON				Params_reg.Flags_Read|=4

#define Kmax_OFF					(GPIO_SetBits(GPIOB,GPIO_Pin_14))
#define Kmax_ON					(GPIO_ResetBits(GPIOB,GPIO_Pin_14))

#define MAX_PARAMS				10000
#define MB_MAXPARAMS				MAX_PARAMS
#define MB_MAX_INPUT_PARAMS	64


#define MB_SENDPAUSE				20
#define MB_RCVPAUSE				20
#define MB_SL_TIMEOUT			3000
#define MB_SLAVE_BUFSIZE		1024
#define MB_SEND_MODE				1
#define MB_RECEIVE_MODE			0
#define UART_RDA					0x4
#define UART_CTI					0xC
#define UART_RLS					0x6
#define UART_THRE					0x2

#define MB_SOURCE					1
#define MB_ADDR					1

#define MB_RBR						((MB_SOURCE)?RS485_RBR:RS232_RBR)

#define RS485_IIR					USART1->SR
#define RS485_LSR					USART1->SR

#define RS232_IIR					USART1->SR
#define RS232_LSR					USART1->SR

#define RS485_THR					USART1->DR
#define RS232_THR					USART1->DR

#define RS485_RBR					USART1->DR
#define RS232_RBR					USART1->DR

#define MB_LSR						((MB_SOURCE)?RS485_LSR:RS232_LSR)


