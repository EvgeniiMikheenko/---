/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : main.c
* Author             : MCD Application Team
* Version            : V2.0.1
* Date               : 06/13/2008
* Description        : Main program body
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/


#include "main.h"
//#define AFIO_MAPR_TIM3_REMAP_NOREMAP         ((uint32_t)0x00000000)        /*!< No remap (CH1/PA6, CH2/PA7, CH3/PB0, CH4/PB1) */
//#define AFIO_MAPR_TIM3_REMAP_PARTIALREMAP    ((uint32_t)0x00000800)        /*!< Partial remap (CH1/PB4, CH2/PB5, CH3/PB0, CH4/PB1) */
//#define AFIO_MAPR_TIM3_REMAP_FULLREMAP       ((uint32_t)0x00000C00)        /*!< Full remap (CH1/PC6, CH2/PC7, CH3/PC8, CH4/PC9) */
//#define AFIO_MAPR_SWJ_CFG_JTAGDISABLE        ((uint32_t)0x02000000)        /*!< JTAG-DP Disabled and SW-DP Enabled */
//#define RCC_APB2Periph_TIM16             ((uint32_t)0x00020000)
//#define RCC_APB2Periph_TIM17             ((uint32_t)0x00040000)

//#include "stm32f10x.h"

//#define _TIM



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Output         : None
* Return         : None
*******************************************************************************/

 








int mb_addr, mb_sl_bytecount = 0, mb_sl_timer = 0, mb_sl_datasize = 0;
signed char mb_sl_mode = MB_RECEIVE_MODE, uart_free = 1, flash_write = 0, check4flash_write = 0, work_enable = 0, triac_on = 0, triac_on_prev = 0, Tset_in_flag = 0, Tallow_Tset_update = 1, Tdiff_zone = 0, Tdiff_zone_prev = 0;
unsigned mb_sl_timeout = 0;
int mb_read_flag = 0;
unsigned char mb_slave_buf[MB_SLAVE_BUFSIZE];

unsigned short mb_input_params[MB_MAX_INPUT_PARAMS + 1];

#define mb_addr mb_slave_buf[0]
#define mb_func mb_slave_buf[1]

typedef struct
    {

        //char Dummy[5-4];
//        FPGA_Out_Struct FPGA_OUT;
//        CPU_In_Struct   CPU_IN;
		unsigned short Flags_Write;
		unsigned short HV_Value;
		unsigned short Freq;
		unsigned short HV_Offset;
		unsigned short Pos_Tresh;
		unsigned short Neg_Tresh;
		//unsigned short Dummy2[4];
		unsigned short Set_Null;
		unsigned short Pos_Tresh_Danger;
		unsigned short Neg_Tresh_Danger;
		unsigned short Filter_Level;
		unsigned short  crc;
        unsigned short  Dummy[19];
		unsigned short Flags_Read;
		unsigned short  Pos_Value;
		unsigned short  Neg_Value;
		unsigned short  HV;
		unsigned short  Ass;
		unsigned short  Temp_Int;
		unsigned short  ID1;
		unsigned short  ID2;
		unsigned short  ID3;
		unsigned short  ID4;
		unsigned short  ID5;
		unsigned short  ID6;
		unsigned short  Dummy1[19];


//        CPU_Out_Struct  CPU_OUT;
//        FPGA_In_Struct  FPGA_IN;
        //  FPGA_Data_Struct FPGA_Data;
    } Params_Struct;

//#pragma PACK(4)
//char  tmp_align[2];

Params_Struct Params;


//#define FLASH_SIZE 24 

unsigned *UNIQUE_ID1 =(unsigned*)0x1FFFF7E8;
unsigned *UNIQUE_ID2 =(unsigned*)0x1FFFF7EC;


unsigned short * PARAMS_BUF = (unsigned short *) & Params;
unsigned short * FLASH_PARAMS_BUF = (unsigned short *) FLASH_LOCATION;

Params_Struct * Flash_Params = (Params_Struct *) FLASH_LOCATION;

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/  
typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
unsigned EraseCounter = 0x00, Address = 0x00;
unsigned Data;
volatile unsigned NbrOfPage = 0x00;
volatile FLASH_Status FLASHStatus;
volatile TestStatus MemoryProgramStatus;
ErrorStatus HSEStartUpStatus;


        unsigned * BUF1 = (unsigned *) &Params;
        unsigned * BUF2 = (unsigned *) FLASH_LOCATION;

void FlashStore()
{
  int i,j;
  FLASHStatus = FLASH_COMPLETE;
  MemoryProgramStatus = PASSED;
  //Data = 0x15041979;

	   j=0;
   for (i=0;i<FLASH_SIZE;i++)
 if (BUF1[i]!=BUF2[i]) j=1;

 if (!j) return;

  
  FLASH_Unlock();

  
  NbrOfPage = (EndAddr - FLASH_LOCATION) / FLASH_PAGE_SIZE;

  
  FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);	

  
  for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
  {
    FLASHStatus = FLASH_ErasePage(FLASH_LOCATION + (FLASH_PAGE_SIZE * EraseCounter));
  }
  
  
  Address = FLASH_LOCATION;
  i=FLASH_SIZE;

   Params.crc = CRC16((unsigned char *) & Params, FLASH_SIZE*4-4);
 for (i=0;i<FLASH_SIZE;i++)
 if (BUF1[i]!=BUF2[i])
 FLASH_ProgramWord(FLASH_LOCATION+(i<<2), BUF1[i]);

//  while((Address < EndAddr) && (FLASHStatus == FLASH_COMPLETE))
//  {
//    FLASHStatus = FLASH_ProgramWord(Address, Data);
//    Address = Address + 4;
//  }
  
  

}

int FlashRestore()
    {
        int        i;

        if (CRC16((unsigned char *) Flash_Params, FLASH_SIZE*4-4) != Flash_Params->crc)
            {
                //     reset_params();
                //Params.CRC=0;
                for (i = 0; i < FLASH_SIZE ; i++)
                    PARAMS_BUF[i] = 0;
                return 0;
            }
        else
            {
                for (i = 0; i < FLASH_SIZE  ; i++)
                    BUF1[i] = BUF2[i];
                return 1;
            }
								   
    }

  



void RCC_Configuration(void)
{
  /* RCC system reset(for debug purpose) */
  RCC_DeInit();
  

  /* Enable HSE */
//  ..RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  //HSEStartUpStatus = RCC_WaitForHSEStartUp();

  //if(HSEStartUpStatus == SUCCESS)
  {
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
    RCC_PLLConfig(RCC_CFGR_PLLSRC_HSI_Div2,RCC_CFGR_PLLMULL6);

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {														   
    }
  }
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
        unsigned int stat=0, crc, i;
        unsigned int startaddr, count;
        char         stat_rs485; //,in_byte232;
        char         stat_rs232, mb_lsr; //,in_byte485,mb_lsr;    

  
//return;
//        mb_lsr = MB_LSR;

		
        stat_rs485 = (RS485_IIR);
		//i=USART1->CR1;

        //stat = ((MB_SOURCE) ? stat_rs485: stat_rs232) & 0xf;
		if (mb_sl_mode == MB_SEND_MODE)
		{		
		if ((stat_rs485&0x80)) 
		stat=0x2; 

		}
		else  
		{
		//USART_ITConfig(USART1, USART_IT_TXE, DISABLE);	
		if (stat_rs485&0x8) 
		i=MB_RBR; 
		if (stat_rs485&0x20) 
		stat=0x4; 
		}


		//stat_rs485 = (USART1->CR1);

        switch (stat)
            {
                case 0x2:
	//USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);					
if ((mb_sl_bytecount+1 == mb_sl_datasize))
                        {
//						i=USART1->SR;
//						i=USART1->CR1;
                            mb_sl_timer = MB_RCVPAUSE; // ??
//                                          mb_sl_timer   =0;
                            mb_sl_mode = MB_RECEIVE_MODE;
							//RS485_THR = 0x66;							
						 	USART_ITConfig(USART1, USART_IT_TXE, DISABLE);	
							USART1->CR1&=~((1<<7));	
							USART1->SR&=~((1<<6));	
							}
							
				
                  if ((MB_LSR & 0x80)  && (mb_sl_bytecount < mb_sl_datasize))
                        {
                            //              for(i=1000;i;i--);
							
				  RS485_THR = mb_slave_buf[mb_sl_bytecount++];
				  }

                    
if ((mb_sl_bytecount == mb_sl_datasize))							
						{
						 //USART1->CR1&=(~(1<<3));
							
                            mb_sl_bytecount = 0;
                            mb_sl_datasize = 0;

                            //                            LED_MB_ACCESS_OFF;
                        }



                                  
	


                    //                if ((U1LSR&0x20)&&(mb_sl_bytecount<mb_sl_datasize)) U1THR=mb_slave_buf[mb_sl_bytecount++];

                    break;
                case 0xC:

                    /*if (mb_sl_bytecount < 8)
                    {
                    while (MB_LSR & 1)
                    in_byte = MB_RBR;
                    mb_sl_bytecount = 0;
                    //                            reset_uart();
                    break;
                    }*/
					break;

                case 0x4:
                    //              stat=CRC16(&inMBbuf,8);
                    //                MBRTU_Receive();
                    //                    while (MB_LSR & 1)
                    //                        {
                    //MB_ADR = U1RBR;
                    in_byte = MB_RBR;
					//in_byte=(USART_ReceiveData(USART1) & 0x7F);
/*                    if (mb_lsr & 0xE)
                        {
                            mb_lsr = 0;
                            break;
                        }*/
                    switch (mb_sl_bytecount)
                        {
                            case 0:
                                //  mb_addr=MB_ADDR;
                                if (in_byte == MB_ADDR)
                                    {
                                        mb_addr = in_byte;
                                    }
                                else
                                    {
                                        mb_sl_bytecount = - 1;
                                        //                                                reset_uart();
                                    }
                                break;
                            case 1:
                                mb_func = in_byte;
                                //                                if (mb_func==3) mb_sl_datasize=8; else mb_sl_datasize=MB_SLAVE_BUFSIZE;
                                if ((mb_func != 4) && (mb_func != 3) && (mb_func != 15) && (mb_func != 16) && (mb_func != 6) && (mb_func != 1) && (mb_func != 2) && (mb_func != 5))
                                    mb_sl_bytecount = - 1;
                                else
                                    mb_sl_datasize = 8;
                                break;

                            default:
                                mb_slave_buf[mb_sl_bytecount] = in_byte;

                                if ((mb_sl_bytecount == 6) && ((mb_func == 16) || (mb_func == 15)))
                                    {
                                        //                                                mb_sl_datasize = 9 + ((mb_slave_buf[5] + (mb_slave_buf[4] << 8)) << 1);
                                        mb_sl_datasize = 9 + mb_slave_buf[6];
                                    }

                                if (mb_sl_datasize - 2 < mb_sl_bytecount)
                                    {
                                        mb_sl_bytecount = - 1;
                                        if (CRC16((unsigned char *) & mb_slave_buf, mb_sl_datasize))
                                            {
                                                mb_sl_datasize = 0;
                                                break;
                                            }
										if (MB_SOURCE)
	                                            RE_OFF;
												
											USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);
											USART1->CR1&=~((1<<5)+(1<<2));	
											

                                            //                                                LED_MB_ACCESS_ON;
                                        startaddr = mb_slave_buf[3] + (mb_slave_buf[2] << 8);

                                            {
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
                                                                    //                                                             params2mb();
                                                                    mb_sl_datasize = 6;
                                                                    i = (mb_slave_buf[5] + (mb_slave_buf[4] << 8));
                                                                    /*                                     if (i==0xFF00) di_output|=(1<<startaddr);
                                                                    else
                                                                    {
                                                                    if (i==0x0000) di_output&=~(1<<startaddr);
                                                                    else
                                                                    {
                                                                    mb_slave_buf[1] = 0x86;
                                                                    mb_slave_buf[2] = 3;
                                                                    mb_sl_datasize = 3;
                                                                    }
                                                                    }*/
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
                                                                    //                                                             params2mb();
                                                                    mb_sl_datasize = 6;
                                                                    PARAMS_BUF[startaddr] = mb_slave_buf[5] + (mb_slave_buf[4] << 8);

                                                                }
                                                            break;

                                                            /*                              case 1:
                                                            if (((startaddr + count) > DO_COUNT) || ((startaddr + count) < 1) || mb_sl_timer)
                                                            {
                                                            mb_func = 0x80 + 1;
                                                            mb_slave_buf[2] = 2;
                                                            mb_sl_datasize = 3;
                                                            }
                                                            else
                                                            {

                                                            //                                                                            params2mb();
                                                            mb_slave_buf[2] = count >>3;
                                                            if (count&0x7) mb_slave_buf[2]++;
                                                            mb_sl_datasize = 3 + mb_slave_buf[2];
                                                            i=di_output&(((1<<count)-1)<<startaddr);
                                                            mb_slave_buf[3] = i&0xff;;
                                                            mb_slave_buf[4] =(i>>8)&0xff;
                                                            }
                                                            break;

                                                            case 2:
                                                            if (((startaddr + count) > DI_COUNT) || ((startaddr + count) < 1) || mb_sl_timer)
                                                            {
                                                            mb_func = 0x80 + 2;
                                                            mb_slave_buf[2] = 2;
                                                            mb_sl_datasize = 3;
                                                            }
                                                            else
                                                            {

                                                            //                                                                            params2mb();
                                                            mb_slave_buf[2] = count >>3;
                                                            if (count&0x7) mb_slave_buf[2]++;
                                                            mb_sl_datasize = 3 + mb_slave_buf[2];
                                                            i=DI_input&(((1<<count)-1)<<startaddr);
                                                            mb_slave_buf[3] = i&0xff;;
                                                            mb_slave_buf[4] =(i>>8)&0xff;
                                                            mb_slave_buf[5] =(i>>16)&0xff;
                                                            }
                                                            break;

                                                            */
                                                        case 3:
                                                            if (((startaddr + count) > MB_MAXPARAMS) || ((startaddr + count) < 1) || mb_sl_timer)
                                                                {
                                                                    mb_func = 0x80 + 3;
                                                                    mb_slave_buf[2] = 2;
                                                                    mb_sl_datasize = 3;
                                                                }
                                                            else
                                                                {

                                                                    //                                                                            params2mb();
                                                                    mb_slave_buf[2] = count << 1;
                                                                    mb_sl_datasize = 3 + mb_slave_buf[2];
                                                                    for (i = 0; i <= count; i++)
                                                                        {
                                                                            //mb_slave_buf[3 + (i << 1)] = (int) mb_params[startaddr + i] >> 8;
                                                                            //mb_slave_buf[4 + (i << 1)] = mb_params[startaddr + i];
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
                                                            /*                                                                case 15:
                                                            if (((startaddr + count) > DO_COUNT) || ((startaddr + count) < 1) || mb_sl_timer)
                                                            {
                                                            mb_func = 0x80 + 15;
                                                            mb_slave_buf[2] = 2;
                                                            mb_sl_datasize = 3;
                                                            }
                                                            else
                                                            {
                                                            //                                      params2mb();
                                                            //                                                                            count = (mb_slave_buf[4] << 8) + mb_slave_buf[5];
                                                            i=(mb_slave_buf[8] << 8) + mb_slave_buf[7];
                                                            di_output|=i&(((1<<count)-1)<<startaddr);
                                                            di_output&=(i|(~(((1<<count)-1)<<startaddr)));

                                                            if (0)
                                                            {
                                                            mb_slave_buf[1] = 0x80 + 16;
                                                            mb_slave_buf[2] = 4;
                                                            mb_sl_datasize = 3;

                                                            }
                                                            else
                                                            {
                                                            mb_sl_datasize = 6;
                                                            }
                                                            }
                                                            break; */

                                                        case 16:
                                                            if (((startaddr + count) > MB_MAXPARAMS) || ((startaddr + count) < 1) || mb_sl_timer)
                                                                {
                                                                    mb_func = 0x80 + 16;
                                                                    mb_slave_buf[2] = 2;
                                                                    mb_sl_datasize = 3;
                                                                }
                                                            else
                                                                {
                                                                    //                                      params2mb();
                                                                    count = (mb_slave_buf[4] << 8) + mb_slave_buf[5];
                                                                    for (i = 0; i < count; i++)
                                                                    //mb_params[startaddr + i] = ((mb_slave_buf[7 + (i << 1)] << 8) + mb_slave_buf[8 + (i << 1)]);
                                                                        PARAMS_BUF[i] = ((mb_slave_buf[7 + (i << 1)] << 8) + mb_slave_buf[8 + (i << 1)]);

                                                                    if (0)
                                                                        {
                                                                            mb_func = 0x80 + 16;
                                                                            mb_slave_buf[2] = 4;
                                                                            mb_sl_datasize = 3;

                                                                        }
                                                                    else
                                                                        {
                                                                            mb_sl_datasize = 6;
                                                                        }
                                                                }
                                                            break;
                                                        default:
                                                                {
                                                                    mb_func = 0x80 + mb_func;
                                                                    mb_slave_buf[2] = 1;
                                                                    mb_sl_datasize = 3;
                                                                }

                                                    }

                                            }
                                        crc = CRC16(mb_slave_buf, mb_sl_datasize);
                                        mb_slave_buf[mb_sl_datasize++] = crc;
                                        mb_slave_buf[mb_sl_datasize++] = crc >> 8;
                                        mb_sl_timer = MB_SENDPAUSE;
                                        mb_sl_mode = MB_SEND_MODE;
                                        mb_sl_timeout = 0;

//                                        rx485_led_timer = RX_TX_LED_TIMER_VALUE;
                                        //                      mb_sl_timer=0;

                                        //                                                mb_led_timer = MB_BLINK_TIME;

                                    }
                                ;
                        }
                    mb_sl_bytecount++;
                    //                        }
                    break;

                default:
                    //                    stat = MB_LSR;
                    //                    reset_uart();
                    break;

            }

        ;




}




int adc_channel=0;
int Pos_Value_int=0;
int Neg_Value_int=0;
int Ehv_Value_int=0;
int Eass_Value_int=0;
int Temp_Value_int=0;
void TIM2_IRQHandler(void)
{
static ch=0;
unsigned int ADC_Value,i,ADC_tmp;
float tmp_x;
// GPIO_SetBits(GPIOA, GPIO_Pin_11|GPIO_Pin_12);

 //ch=USART1->CR2;

//adc_channel=(adc_channel+1)&0x3;

PI_DANGER_ON;

adc_channel=((adc_channel==4)?0:adc_channel+1);
ADC_Value=ADC_GetConversionValue(ADC1);
switch (adc_channel)
	{
	case 0:
//		Params.Neg_Value=ADC_Value;	
		Temp_Value_int+=(((ADC_Value<<16)-Temp_Value_int)>>8);	
		Params.Temp_Int=(28000-(Temp_Value_int>>16))/8.539539393939394+250;
		ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 1, ADC_SampleTime_239Cycles5);
		break;
	case 1:
		
		//Params.Ass=32768+(ADC_Value-11400)*47.57/2340-240;					//Eass
		Eass_Value_int+=((((ADC_Value-32768)<<16)-Eass_Value_int)>>8);
		Params.Ass=32768-90*((Eass_Value_int>>16))/32768;					//Eass

		ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_239Cycles5);
		break;
	case 2:
		//Params.HV=(ADC_Value)*4000*3.3/65536; //Ehv		 
		//Ehv_Value_int+=(((ADC_Value<<16)-Ehv_Value_int)>>8);
		//tmp_x=3.3*10*285.3/1.308*(Ehv_Value_int>>16)/65536;
		//Params.HV=285.3/1.308*(Ehv_Value_int>>16)/65536*10*3.3; //Ehv		 
		Params.HV=10*3.3*659/3*(ADC_Value)/65536/2; //Ehv		 
		ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_239Cycles5); //5
		break;
	case 3:
		//if (ADC_Value>32768) 
		
		Pos_Value_int+=(((ADC_Value<<16)-Pos_Value_int)>>Params.Filter_Level);
		ADC_tmp=FLAG_FILTER?(Pos_Value_int>>16):ADC_Value;
		Params.Pos_Value=(ADC_tmp>32768)?ADC_tmp-32768:0;
		Params.Neg_Value=(ADC_tmp<32768)?32768-ADC_tmp:0;

		//Params.Pos_Value=ADC_Value;
		//Params.Pos_Value=30000;
		ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_239Cycles5);		   //6
		break;
	case 4:
//		Neg_Value_int+=(((ADC_Value<<16)-Neg_Value_int)>>Params.Filter_Level);
//		Params.Neg_Value=FLAG_FILTER?(Neg_Value_int>>16):ADC_Value;		
		ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_239Cycles5);
		break;
		}


ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  

        if (mb_sl_timer)
            {
                if (--mb_sl_timer == 0)
                    {
                        if (mb_sl_mode == MB_SEND_MODE)
                            {
							//mb_sl_timer = 1000;
                                if (MB_SOURCE)
                                    DE_ON;
									
									
											

								//	(MB_LSR & 0x80) &&
								   //mb_sl_datasize=7;
                                if ( (mb_sl_bytecount < mb_sl_datasize))
                                    {
                                             
							//	i=USART1->SR;																				        
                                        if (MB_SOURCE)
                                            {
											    i=USART1->CR1;
                                             //   RS485_THR = mb_slave_buf[mb_sl_bytecount++];
											 //mb_sl_bytecount++;
											    //RS485_THR = 0xAA;
                                                ///                                     tx485_led_timer=RX_TX_LED_TIMER_VALUE;
                                            }
								//RS485_THR = 0xAA;
									USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
									USART1->CR1|=((1<<7));	

                                      
                                        ;

                                    }
                            }
                        else
                            {
                                if (MB_SOURCE)
                                    {
                                        DE_OFF;
                                        RE_ON;
										USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
										USART1->CR1|=((1<<5)+(1<<2));	
                                    }
                            }

                    }
            }
PI_DANGER_OFF;
   TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
}



/* Private define ------------------------------------------------------------*/

void delay(time)    //us
{
time<<=3;  //HZ
while(time--);
}





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



DAC_InitTypeDef            DAC_InitStructure;

int x;

int main(void)
{
int y;
//while(1);

#ifdef DEBUG
  debug();
#endif

//while(1);

RCC_Configuration();

 





  /* TIM3 clock enable */


RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16|RCC_APB2Periph_TIM17| RCC_APB2Periph_USART1, ENABLE);
RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC|RCC_APB1Periph_TIM2 |RCC_APB1Periph_TIM3, ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1 |RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA| RCC_APB2Periph_GPIOB|RCC_APB2Periph_AFIO, ENABLE);

//AFIO->MAPR|=AFIO_MAPR_TIM3_REMAP_PARTIALREMAP|AFIO_MAPR_SWJ_CFG_JTAGDISABLE;
AFIO->MAPR|=AFIO_MAPR_SWJ_CFG_JTAGDISABLE;


  /*GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_10|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed =  GPIO_Speed_2MHz;
  
  GPIO_Init(GPIOB, &GPIO_InitStructure);

GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_11|GPIO_Pin_12;

  GPIO_Init(GPIOA, &GPIO_InitStructure);




  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed =  GPIO_Speed_2MHz;  
  GPIO_Init(GPIOB, &GPIO_InitStructure);	  */

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AIN;
  GPIO_InitStructure.GPIO_Speed =  GPIO_Speed_2MHz;
  
  GPIO_Init(GPIOA, &GPIO_InitStructure);	  




	
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	   //PI
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	   //PF
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_13|GPIO_Pin_14;	   //charge pwm +ir2214
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;	   //ir2214 ctl
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	   //DE
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;	   //RE
  GPIO_Init(GPIOB, &GPIO_InitStructure);

   /* Configure USART1 Tx (PA.09) as alternate function push-pull */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure USART1 Rx (PA.10) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


   /* TIM2 configuration */
  TIM_TimeBaseStructure.TIM_Period = 24000;          
  TIM_TimeBaseStructure.TIM_Prescaler = 0;       
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;
  TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
  
  TIM2->DIER=1;
    TIM_Cmd(TIM2, ENABLE);




  
  

  TIM_TimeBaseStructure.TIM_Period =65534 ;
  TIM_TimeBaseStructure.TIM_Prescaler = 1;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);



  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = TIM_TimeBaseStructure.TIM_Period>>1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;

  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;


  /*TIM_OC2Init(TIM16, &TIM_OCInitStructure);


  TIM_OC2PreloadConfig(TIM16, TIM_OCPreload_Enable);




  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 10000;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;*/

  TIM_OC1Init(TIM16, &TIM_OCInitStructure);


  TIM_OC1PreloadConfig(TIM16, TIM_OCPreload_Enable);
  






  TIM_TimeBaseStructure.TIM_Period = 65534;
  TIM_TimeBaseStructure.TIM_Prescaler = 4;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
   TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);



  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = TIM_TimeBaseStructure.TIM_Period>>1;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;

  TIM_OC2Init(TIM1, &TIM_OCInitStructure);


  TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);




  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_Pulse = TIM_TimeBaseStructure.TIM_Period>>1;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;

  TIM_OC1Init(TIM1, &TIM_OCInitStructure);


  TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

//TIM16->DIER=1;

TIM16->CCER=0x5;
TIM16->BDTR=(1<<15) ;
TIM_Cmd(TIM16, ENABLE);
x=TIM16->CCER;
x=TIM16->CR2;



  TIM_TimeBaseStructure.TIM_Period = 65535;
  TIM_TimeBaseStructure.TIM_Prescaler = 3;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

//

  TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure);
  


  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 32768;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

  TIM_OC1Init(TIM17, &TIM_OCInitStructure);

  TIM_OC1PreloadConfig(TIM17, TIM_OCPreload_Enable);

TIM17->BDTR=(1<<15) ;

TIM_Cmd(TIM17, ENABLE);				   //Shift




GPIO_SetBits(GPIOB,GPIO_Pin_12);

//GPIO_ResetBits(GPIOB,GPIO_Pin_12);

   //TIM1->CR1|=(1<<5);
//      TIM1->BDTR=(1<<15)+0xFF;

   i=TIM1->CR1;

   //+elay(1000000);


   TIM_Cmd(TIM1, ENABLE);
TIM_CtrlPWMOutputs(TIM1, ENABLE);

/*

  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  DAC_InitStructure.DAC_Trigger = DAC_Trigger_None ;
  DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;
  DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_TriangleAmplitude_2047;
  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;
  DAC_Init(DAC_Channel_1, &DAC_InitStructure);

  DAC_InitStructure.DAC_LFSRUnmask_TriangleAmplitude = DAC_TriangleAmplitude_1023;
  DAC_Init(DAC_Channel_2, &DAC_InitStructure);

  DAC_Cmd(DAC_Channel_1, ENABLE);

  DAC_Cmd(DAC_Channel_2, ENABLE);

  DAC_SetDualChannelData(DAC_Align_12b_L, 0x100, 0x8000);

*/   
//__wfi();   










  /*GPIOA Configuration: TIM3 channel 1 and 2 as alternate function push-pull */
  //GPIO_InitStructure.GPIO_Pin =   GPIO_Pin_7;
  //GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  //GPIO_Init(GPIOA, &GPIO_InitStructure);


  


  

  

    
  /* ADC1 configuration ------------------------------------------------------*/
/* ADC1 Configuration ------------------------------------------------------*/
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
  


    /* ADC1 regular channel14 configuration */

  USART_InitStructure.USART_BaudRate = 115200;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

//  USART1->CR1=(1<<13)|(1<<7)|(1<<5)|(1<<3)|(1<<2);
//  USART1->CR3=(1<<11);
//    USART1->BRR=0xD0;


  /* Configure the USART1 */
  USART_Init(USART1, &USART_InitStructure);

  /* Enable the USART Transmoit interrupt: this interrupt is generated when the
     USART1 transmit data register is empty */
  //USART_ITConfig(USART1, USART_IT_TXE, ENABLE);

  /* Enable the USART Receive interrupt: this interrupt is generated when the
     USART1 receive data register is not empty */
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);


  /* Enable USART1 */
USART_Cmd(USART1, ENABLE);      


goto no_flash;



no_flash:








while(ADC_GetCalibrationStatus(ADC1));
  //ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_55Cycles5);
//  ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 1, ADC_SampleTime_55Cycles5);
//  ADC_RegularChannelConfig(ADC1, ADC_Channel_5, 1, ADC_SampleTime_55Cycles5);
//  ADC_RegularChannelConfig(ADC1, ADC_Channel_6, 1, ADC_SampleTime_55Cycles5);

    /* Start ADC1 Software Conversion */ 
  




/*  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed =  GPIO_Speed_2MHz;
  
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB, GPIO_Pin_14|GPIO_Pin_15);
*/

  /* Infinite loop */

	DE_OFF;
	RE_ON;


Params.HV_Value=3000;
Params.Freq=4000;
Params.HV_Offset=32768;
Params.Set_Null=32768;
FlashRestore();
Params.Flags_Read=0;
Params.ID1=*UNIQUE_ID1;
Params.ID2=(*UNIQUE_ID1)>>16;
Params.ID3=(*UNIQUE_ID2);
NVIC_Configuration();

while(1)
{

if(Params.HV_Value)
{
y=9000*3350/Params.HV_Value;
}
else y=65534;

if(y<9000) y=9000;
if(y>65534) y=65534;
TIM16->ARR=y;
TIM16->CCR1=y>>1;

if(Params.Freq)
{
y=24000000/2/5*10/Params.Freq;
}
else y=65534;

if(y<2400) y=2400;
if(y>65534) y=65534;
TIM1->ARR=y;
TIM1->CCR1=y>>1;
TIM1->CCR2=y>>1;

TIM17->CCR1=32768+32767*(Params.HV_Offset-32768)/90;


__wfi();
}

  while (1)
  {
  
  TIM3->CCR1=128-128*(Params.HV_Offset-32768)/90; //Cont2Ass	
  TIM16->CCR1=128+128*(Params.Set_Null-32768)/200; //Set_Null

  if ((Params.Neg_Value*3000>>16)>Params.Neg_Tresh_Danger)
  PI_DANGER_ON;
  else
  PI_DANGER_OFF;
  

  if ((Params.Pos_Value*3000>>16)>Params.Pos_Tresh_Danger)
  PF_DANGER_ON;
  else
  PF_DANGER_OFF;

  if ((Params.Neg_Value*3000>>16)>Params.Neg_Tresh)
  PI_TRESH_ON;
  else
  PI_TRESH_OFF;
  

  if ((Params.Pos_Value*3000>>16)>Params.Pos_Tresh)
  PF_TRESH_ON;
  else
  PF_TRESH_OFF;



if (Params.Flags_Write&2) Kmax_ON; else Kmax_OFF;

if (Params.Flags_Write&1)  
{
                        __disable_irq();
                        __disable_fiq();
						FlashStore();
                        __enable_irq();
                        __enable_fiq();
//						Params.Flags_Write&=1;
}

  //i=TIM17->CCER;					  
  
  
  //TIM3->CCR2=2*Params.HV_Value*256/10/200/3.3; //2*Um

  TIM3->CCR2=1+256*Params.HV_Value/3350; //2*Um

 
  if((Params.Freq>499)&&(Params.Freq<8001))
  {
  TIM17->ARR=10*24000000/(Params.Freq*2)/4;  //2*Freq
  TIM17->CCR1=TIM17->ARR/2;

  }	
  


  

  }
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
