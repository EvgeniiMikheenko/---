

#define _FLASH_PROG					   	
//#define _RCC 1
#include <stm32f10x.h>
//#include "stm32f10x_lib.h"
#include <stm32f10x_adc.h>
#include <stm32f10x_flash.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_usart.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_adc.h>
#include <stm32f10x_tim.h>
#include <stm32f10x_dac.h>
//#include <misc.h>
#include <stm32f10x_nvic.h>



//#define UNIQUE_ID1    ((u16)0x1FFFF7E8)

#define FLAG_FILTER      (Params.Flags_Write&4)

#define FLASH_PAGE_SIZE    ((unsigned short)0x400)
#define FLASH_LOCATION  ((unsigned)0x08004000-FLASH_PAGE_SIZE)
#define EndAddr    ((unsigned)0x08004000)
#define FLASH_SIZE (((unsigned short *)&Params.crc - (unsigned short *) & Params)/2+1)


#define DE_ON GPIO_SetBits(GPIOA,GPIO_Pin_8)
#define DE_OFF GPIO_ResetBits(GPIOA,GPIO_Pin_8)

#define RE_ON GPIO_ResetBits(GPIOB,GPIO_Pin_15)
#define RE_OFF GPIO_SetBits(GPIOB,GPIO_Pin_15)

//#define PI_PF_OFF GPIO_ResetBits(GPIOA,GPIO_Pin_11|GPIO_Pin_12)
#define PI_DANGER_OFF (GPIO_ResetBits(GPIOB,GPIO_Pin_3),Params.Flags_Read&=~1)
#define PF_DANGER_OFF (GPIO_ResetBits(GPIOA,GPIO_Pin_15),Params.Flags_Read&=~2)

#define PI_DANGER_ON (GPIO_SetBits(GPIOB,GPIO_Pin_3),GPIO_ResetBits(GPIOA,GPIO_Pin_15),Params.Flags_Read|=1)
#define PF_DANGER_ON (GPIO_SetBits(GPIOA,GPIO_Pin_15),GPIO_ResetBits(GPIOB,GPIO_Pin_3),Params.Flags_Read|=2)

#define PI_TRESH_OFF Params.Flags_Read&=~4
#define PF_TRESH_OFF Params.Flags_Read&=~8

#define PI_TRESH_ON Params.Flags_Read|=4
#define PF_TRESH_ON Params.Flags_Read|=8


#define Kmax_OFF (GPIO_SetBits(GPIOB,GPIO_Pin_14))
#define Kmax_ON (GPIO_ResetBits(GPIOB,GPIO_Pin_14))




#define MAX_PARAMS 10000

#define MB_MAXPARAMS  MAX_PARAMS

#define MB_MAX_INPUT_PARAMS 64



//#define MB_SENDPAUSE  20
#define MB_SENDPAUSE  40
#define MB_RCVPAUSE   5
#define MB_SL_TIMEOUT 3000
#define MB_SLAVE_BUFSIZE 1024
#define MB_SEND_MODE  1
#define MB_RECEIVE_MODE 0
#define UART_RDA  0x4
#define UART_CTI  0xC
#define UART_RLS  0x6
#define UART_THRE  0x2



#define MB_SOURCE 1
#define MB_ADDR 1

#define MB_RBR ((MB_SOURCE)?RS485_RBR:RS232_RBR)

#define RS485_IIR USART1->SR
#define RS485_LSR USART1->SR

#define RS232_IIR USART1->SR
#define RS232_LSR USART1->SR

#define RS485_THR USART1->DR
#define RS232_THR USART1->DR

#define RS485_RBR USART1->DR
#define RS232_RBR USART1->DR



#define MB_LSR ((MB_SOURCE)?RS485_LSR:RS232_LSR)