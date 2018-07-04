#line 1 "md5c.c"




























 

#line 1 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"







































 



 



 
    






  


 
  


 

#line 75 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"


















 










 
   








            
#line 122 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"





 






 
#line 143 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 



 



 
#line 162 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"




 
typedef enum IRQn
{
 
  NonMaskableInt_IRQn         = -14,     
  MemoryManagement_IRQn       = -12,     
  BusFault_IRQn               = -11,     
  UsageFault_IRQn             = -10,     
  SVCall_IRQn                 = -5,      
  DebugMonitor_IRQn           = -4,      
  PendSV_IRQn                 = -2,      
  SysTick_IRQn                = -1,      

 
  WWDG_IRQn                   = 0,       
  PVD_IRQn                    = 1,       
  TAMPER_IRQn                 = 2,       
  RTC_IRQn                    = 3,       
  FLASH_IRQn                  = 4,       
  RCC_IRQn                    = 5,       
  EXTI0_IRQn                  = 6,       
  EXTI1_IRQn                  = 7,       
  EXTI2_IRQn                  = 8,       
  EXTI3_IRQn                  = 9,       
  EXTI4_IRQn                  = 10,      
  DMA1_Channel1_IRQn          = 11,      
  DMA1_Channel2_IRQn          = 12,      
  DMA1_Channel3_IRQn          = 13,      
  DMA1_Channel4_IRQn          = 14,      
  DMA1_Channel5_IRQn          = 15,      
  DMA1_Channel6_IRQn          = 16,      
  DMA1_Channel7_IRQn          = 17,      


  ADC1_2_IRQn                 = 18,      
  USB_HP_CAN1_TX_IRQn         = 19,      
  USB_LP_CAN1_RX0_IRQn        = 20,      
  CAN1_RX1_IRQn               = 21,      
  CAN1_SCE_IRQn               = 22,      
  EXTI9_5_IRQn                = 23,      
  TIM1_BRK_IRQn               = 24,      
  TIM1_UP_IRQn                = 25,      
  TIM1_TRG_COM_IRQn           = 26,      
  TIM1_CC_IRQn                = 27,      
  TIM2_IRQn                   = 28,      
  TIM3_IRQn                   = 29,      
  I2C1_EV_IRQn                = 31,      
  I2C1_ER_IRQn                = 32,      
  SPI1_IRQn                   = 35,      
  USART1_IRQn                 = 37,      
  USART2_IRQn                 = 38,      
  EXTI15_10_IRQn              = 40,      
  RTCAlarm_IRQn               = 41,      
  USBWakeUp_IRQn              = 42           


#line 242 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 270 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 296 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 341 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 381 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 426 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 472 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"
} IRQn_Type;



 

#line 1 "D:\\tools\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"
 




















 






















 




 


 

 













#line 89 "D:\\tools\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"


 







#line 119 "D:\\tools\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"

#line 1 "D:\\tools\\Keil\\ARM\\RV31\\Inc\\stdint.h"
 
 





 









#line 25 "D:\\tools\\Keil\\ARM\\RV31\\Inc\\stdint.h"







 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     
typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;

     
typedef   signed       __int64 intmax_t;
typedef unsigned       __int64 uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     


     


     


     

     


     


     


     

     



     



     


     
    
 



#line 196 "D:\\tools\\Keil\\ARM\\RV31\\Inc\\stdint.h"

     







     










     











#line 260 "D:\\tools\\Keil\\ARM\\RV31\\Inc\\stdint.h"



 


#line 121 "D:\\tools\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"
#line 1 "D:\\tools\\Keil\\ARM\\CMSIS\\Include\\core_cmInstr.h"
 




















 





 



 


 









 







 







 






 








 







 







 









 









 

__attribute__((section(".rev16_text"))) static __inline __asm uint32_t __REV16(uint32_t value)
{
  rev16 r0, r0
  bx lr
}








 

__attribute__((section(".revsh_text"))) static __inline __asm int32_t __REVSH(int32_t value)
{
  revsh r0, r0
  bx lr
}










 











 









 









 









 











 











 











 







 










 










 









 






#line 618 "D:\\tools\\Keil\\ARM\\CMSIS\\Include\\core_cmInstr.h"

   

#line 122 "D:\\tools\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"
#line 1 "D:\\tools\\Keil\\ARM\\CMSIS\\Include\\core_cmFunc.h"
 




















 





 



 


 





 
 






 
static __inline uint32_t __get_CONTROL(void)
{
  register uint32_t __regControl         __asm("control");
  return(__regControl);
}







 
static __inline void __set_CONTROL(uint32_t control)
{
  register uint32_t __regControl         __asm("control");
  __regControl = control;
}







 
static __inline uint32_t __get_IPSR(void)
{
  register uint32_t __regIPSR          __asm("ipsr");
  return(__regIPSR);
}







 
static __inline uint32_t __get_APSR(void)
{
  register uint32_t __regAPSR          __asm("apsr");
  return(__regAPSR);
}







 
static __inline uint32_t __get_xPSR(void)
{
  register uint32_t __regXPSR          __asm("xpsr");
  return(__regXPSR);
}







 
static __inline uint32_t __get_PSP(void)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  return(__regProcessStackPointer);
}







 
static __inline void __set_PSP(uint32_t topOfProcStack)
{
  register uint32_t __regProcessStackPointer  __asm("psp");
  __regProcessStackPointer = topOfProcStack;
}







 
static __inline uint32_t __get_MSP(void)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  return(__regMainStackPointer);
}







 
static __inline void __set_MSP(uint32_t topOfMainStack)
{
  register uint32_t __regMainStackPointer     __asm("msp");
  __regMainStackPointer = topOfMainStack;
}







 
static __inline uint32_t __get_PRIMASK(void)
{
  register uint32_t __regPriMask         __asm("primask");
  return(__regPriMask);
}







 
static __inline void __set_PRIMASK(uint32_t priMask)
{
  register uint32_t __regPriMask         __asm("primask");
  __regPriMask = (priMask);
}








 







 








 
static __inline uint32_t  __get_BASEPRI(void)
{
  register uint32_t __regBasePri         __asm("basepri");
  return(__regBasePri);
}







 
static __inline void __set_BASEPRI(uint32_t basePri)
{
  register uint32_t __regBasePri         __asm("basepri");
  __regBasePri = (basePri & 0xff);
}







 
static __inline uint32_t __get_FAULTMASK(void)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  return(__regFaultMask);
}







 
static __inline void __set_FAULTMASK(uint32_t faultMask)
{
  register uint32_t __regFaultMask       __asm("faultmask");
  __regFaultMask = (faultMask & (uint32_t)1);
}




#line 293 "D:\\tools\\Keil\\ARM\\CMSIS\\Include\\core_cmFunc.h"


#line 612 "D:\\tools\\Keil\\ARM\\CMSIS\\Include\\core_cmFunc.h"

 


#line 123 "D:\\tools\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"








 
#line 153 "D:\\tools\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"

 






 
#line 169 "D:\\tools\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"

 












 


 





 


 
typedef union
{
  struct
  {

    uint32_t _reserved0:27;               





    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} APSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       
    uint32_t _reserved0:23;               
  } b;                                    
  uint32_t w;                             
} IPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t ISR:9;                       

    uint32_t _reserved0:15;               





    uint32_t T:1;                         
    uint32_t IT:2;                        
    uint32_t Q:1;                         
    uint32_t V:1;                         
    uint32_t C:1;                         
    uint32_t Z:1;                         
    uint32_t N:1;                         
  } b;                                    
  uint32_t w;                             
} xPSR_Type;



 
typedef union
{
  struct
  {
    uint32_t nPRIV:1;                     
    uint32_t SPSEL:1;                     
    uint32_t FPCA:1;                      
    uint32_t _reserved0:29;               
  } b;                                    
  uint32_t w;                             
} CONTROL_Type;

 






 


 
typedef struct
{
  volatile uint32_t ISER[8];                  
       uint32_t RESERVED0[24];
  volatile uint32_t ICER[8];                  
       uint32_t RSERVED1[24];
  volatile uint32_t ISPR[8];                  
       uint32_t RESERVED2[24];
  volatile uint32_t ICPR[8];                  
       uint32_t RESERVED3[24];
  volatile uint32_t IABR[8];                  
       uint32_t RESERVED4[56];
  volatile uint8_t  IP[240];                  
       uint32_t RESERVED5[644];
  volatile  uint32_t STIR;                     
}  NVIC_Type;

 



 






 


 
typedef struct
{
  volatile const  uint32_t CPUID;                    
  volatile uint32_t ICSR;                     
  volatile uint32_t VTOR;                     
  volatile uint32_t AIRCR;                    
  volatile uint32_t SCR;                      
  volatile uint32_t CCR;                      
  volatile uint8_t  SHP[12];                  
  volatile uint32_t SHCSR;                    
  volatile uint32_t CFSR;                     
  volatile uint32_t HFSR;                     
  volatile uint32_t DFSR;                     
  volatile uint32_t MMFAR;                    
  volatile uint32_t BFAR;                     
  volatile uint32_t AFSR;                     
  volatile const  uint32_t PFR[2];                   
  volatile const  uint32_t DFR;                      
  volatile const  uint32_t ADR;                      
  volatile const  uint32_t MMFR[4];                  
  volatile const  uint32_t ISAR[5];                  
       uint32_t RESERVED0[5];
  volatile uint32_t CPACR;                    
} SCB_Type;

 















 






























 




#line 396 "D:\\tools\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"

 





















 









 


















 










































 









 









 















 






 


 
typedef struct
{
       uint32_t RESERVED0[1];
  volatile const  uint32_t ICTR;                     



       uint32_t RESERVED1[1];

} SCnSCB_Type;

 



 










 






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t LOAD;                     
  volatile uint32_t VAL;                      
  volatile const  uint32_t CALIB;                    
} SysTick_Type;

 












 



 



 









 






 


 
typedef struct
{
  volatile  union
  {
    volatile  uint8_t    u8;                   
    volatile  uint16_t   u16;                  
    volatile  uint32_t   u32;                  
  }  PORT [32];                           
       uint32_t RESERVED0[864];
  volatile uint32_t TER;                      
       uint32_t RESERVED1[15];
  volatile uint32_t TPR;                      
       uint32_t RESERVED2[15];
  volatile uint32_t TCR;                      
       uint32_t RESERVED3[29];                                  
  volatile  uint32_t IWR;                      
  volatile const  uint32_t IRR;                      
  volatile uint32_t IMCR;                     
       uint32_t RESERVED4[43];                                  
  volatile  uint32_t LAR;                      
  volatile const  uint32_t LSR;                      
       uint32_t RESERVED5[6];                                   
  volatile const  uint32_t PID4;                     
  volatile const  uint32_t PID5;                     
  volatile const  uint32_t PID6;                     
  volatile const  uint32_t PID7;                     
  volatile const  uint32_t PID0;                     
  volatile const  uint32_t PID1;                     
  volatile const  uint32_t PID2;                     
  volatile const  uint32_t PID3;                     
  volatile const  uint32_t CID0;                     
  volatile const  uint32_t CID1;                     
  volatile const  uint32_t CID2;                     
  volatile const  uint32_t CID3;                     
} ITM_Type;

 



 



























 



 



 



 









   






 


 
typedef struct
{
  volatile uint32_t CTRL;                     
  volatile uint32_t CYCCNT;                   
  volatile uint32_t CPICNT;                   
  volatile uint32_t EXCCNT;                   
  volatile uint32_t SLEEPCNT;                 
  volatile uint32_t LSUCNT;                   
  volatile uint32_t FOLDCNT;                  
  volatile const  uint32_t PCSR;                     
  volatile uint32_t COMP0;                    
  volatile uint32_t MASK0;                    
  volatile uint32_t FUNCTION0;                
       uint32_t RESERVED0[1];
  volatile uint32_t COMP1;                    
  volatile uint32_t MASK1;                    
  volatile uint32_t FUNCTION1;                
       uint32_t RESERVED1[1];
  volatile uint32_t COMP2;                    
  volatile uint32_t MASK2;                    
  volatile uint32_t FUNCTION2;                
       uint32_t RESERVED2[1];
  volatile uint32_t COMP3;                    
  volatile uint32_t MASK3;                    
  volatile uint32_t FUNCTION3;                
} DWT_Type;

 






















































 



 



 



 



 



 



 



























   






 


 
typedef struct
{
  volatile uint32_t SSPSR;                    
  volatile uint32_t CSPSR;                    
       uint32_t RESERVED0[2];
  volatile uint32_t ACPR;                     
       uint32_t RESERVED1[55];
  volatile uint32_t SPPR;                     
       uint32_t RESERVED2[131];
  volatile const  uint32_t FFSR;                     
  volatile uint32_t FFCR;                     
  volatile const  uint32_t FSCR;                     
       uint32_t RESERVED3[759];
  volatile const  uint32_t TRIGGER;                  
  volatile const  uint32_t FIFO0;                    
  volatile const  uint32_t ITATBCTR2;                
       uint32_t RESERVED4[1];
  volatile const  uint32_t ITATBCTR0;                
  volatile const  uint32_t FIFO1;                    
  volatile uint32_t ITCTRL;                   
       uint32_t RESERVED5[39];
  volatile uint32_t CLAIMSET;                 
  volatile uint32_t CLAIMCLR;                 
       uint32_t RESERVED7[8];
  volatile const  uint32_t DEVID;                    
  volatile const  uint32_t DEVTYPE;                  
} TPI_Type;

 



 



 












 






 



 





















 



 





















 



 



 


















 






   


#line 1107 "D:\\tools\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"






 


 
typedef struct
{
  volatile uint32_t DHCSR;                    
  volatile  uint32_t DCRSR;                    
  volatile uint32_t DCRDR;                    
  volatile uint32_t DEMCR;                    
} CoreDebug_Type;

 




































 






 







































 






 

 
#line 1227 "D:\\tools\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"

#line 1236 "D:\\tools\\Keil\\ARM\\CMSIS\\Include\\core_cm3.h"






 










 

 



 




 










 
static __inline void NVIC_SetPriorityGrouping(uint32_t PriorityGroup)
{
  uint32_t reg_value;
  uint32_t PriorityGroupTmp = (PriorityGroup & (uint32_t)0x07);                

  reg_value  =  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR;                                                    
  reg_value &= ~((0xFFFFUL << 16) | (7UL << 8));              
  reg_value  =  (reg_value                                 |
                ((uint32_t)0x5FA << 16) |
                (PriorityGroupTmp << 8));                                      
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR =  reg_value;
}







 
static __inline uint32_t NVIC_GetPriorityGrouping(void)
{
  return ((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8)) >> 8);    
}







 
static __inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}







 
static __inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}











 
static __inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}







 
static __inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}







 
static __inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[((uint32_t)(IRQn) >> 5)] = (1 << ((uint32_t)(IRQn) & 0x1F));  
}










 
static __inline uint32_t NVIC_GetActive(IRQn_Type IRQn)
{
  return((uint32_t)((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IABR[(uint32_t)(IRQn) >> 5] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));  
}










 
static __inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[((uint32_t)(IRQn) & 0xF)-4] = ((priority << (8 - 4)) & 0xff); }  
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[(uint32_t)(IRQn)] = ((priority << (8 - 4)) & 0xff);    }         
}












 
static __inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)(((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[((uint32_t)(IRQn) & 0xF)-4] >> (8 - 4)));  }  
  else {
    return((uint32_t)(((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[(uint32_t)(IRQn)]           >> (8 - 4)));  }  
}













 
static __inline uint32_t NVIC_EncodePriority (uint32_t PriorityGroup, uint32_t PreemptPriority, uint32_t SubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 4) ? 4 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 4) < 7) ? 0 : PriorityGroupTmp - 7 + 4;

  return (
           ((PreemptPriority & ((1 << (PreemptPriorityBits)) - 1)) << SubPriorityBits) |
           ((SubPriority     & ((1 << (SubPriorityBits    )) - 1)))
         );
}













 
static __inline void NVIC_DecodePriority (uint32_t Priority, uint32_t PriorityGroup, uint32_t* pPreemptPriority, uint32_t* pSubPriority)
{
  uint32_t PriorityGroupTmp = (PriorityGroup & 0x07);           
  uint32_t PreemptPriorityBits;
  uint32_t SubPriorityBits;

  PreemptPriorityBits = ((7 - PriorityGroupTmp) > 4) ? 4 : 7 - PriorityGroupTmp;
  SubPriorityBits     = ((PriorityGroupTmp + 4) < 7) ? 0 : PriorityGroupTmp - 7 + 4;

  *pPreemptPriority = (Priority >> SubPriorityBits) & ((1 << (PreemptPriorityBits)) - 1);
  *pSubPriority     = (Priority                   ) & ((1 << (SubPriorityBits    )) - 1);
}





 
static __inline void NVIC_SystemReset(void)
{
  __dsb(0xF);                                                     
 
  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR  = ((0x5FA << 16)      |
                 (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR & (7UL << 8)) |
                 (1UL << 2));                    
  __dsb(0xF);                                                      
  while(1);                                                     
}

 



 




 

















 
static __inline uint32_t SysTick_Config(uint32_t ticks)
{
  if (ticks > (0xFFFFFFUL << 0))  return (1);             

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD  = (ticks & (0xFFFFFFUL << 0)) - 1;       
  NVIC_SetPriority (SysTick_IRQn, 0);
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL   = 0;                                           
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL  = (1UL << 2) |
                   (1UL << 1)   |
                   (1UL << 0);                     
  return (0);                                                   
}



 



 




 

extern volatile int32_t ITM_RxBuffer;                     












 
static __inline uint32_t ITM_SendChar (uint32_t ch)
{
  if ((((ITM_Type *) (0xE0000000UL) )->TCR & (1UL << 0))                  &&       
      (((ITM_Type *) (0xE0000000UL) )->TER & (1UL << 0)        )                    )      
  {
    while (((ITM_Type *) (0xE0000000UL) )->PORT[0].u32 == 0);
    ((ITM_Type *) (0xE0000000UL) )->PORT[0].u8 = (uint8_t) ch;
  }
  return (ch);
}








 
static __inline int32_t ITM_ReceiveChar (void) {
  int32_t ch = -1;                            

  if (ITM_RxBuffer != 0x5AA55AA5) {
    ch = ITM_RxBuffer;
    ITM_RxBuffer = 0x5AA55AA5;        
  }

  return (ch);
}








 
static __inline int32_t ITM_CheckChar (void) {

  if (ITM_RxBuffer == 0x5AA55AA5) {
    return (0);                                  
  } else {
    return (1);                                  
  }
}

 





#line 479 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"
#line 1 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\system_stm32f10x.h"



















 



 



   
  


 









 



 




 

extern uint32_t SystemCoreClock;           



 



 



 



 



 



 
  
extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);


 









 
  


   
 
#line 480 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"
#line 481 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



   

 
typedef int32_t  s32;
typedef int16_t s16;
typedef int8_t  s8;

typedef const int32_t sc32;   
typedef const int16_t sc16;   
typedef const int8_t sc8;    

typedef volatile int32_t  vs32;
typedef volatile int16_t  vs16;
typedef volatile int8_t   vs8;

typedef volatile const int32_t vsc32;   
typedef volatile const int16_t vsc16;   
typedef volatile const int8_t vsc8;    

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

typedef const uint32_t uc32;   
typedef const uint16_t uc16;   
typedef const uint8_t uc8;    

typedef volatile uint32_t  vu32;
typedef volatile uint16_t vu16;
typedef volatile uint8_t  vu8;

typedef volatile const uint32_t vuc32;   
typedef volatile const uint16_t vuc16;   
typedef volatile const uint8_t vuc8;    

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;


typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

 





 



    



 

typedef struct
{
  volatile uint32_t SR;
  volatile uint32_t CR1;
  volatile uint32_t CR2;
  volatile uint32_t SMPR1;
  volatile uint32_t SMPR2;
  volatile uint32_t JOFR1;
  volatile uint32_t JOFR2;
  volatile uint32_t JOFR3;
  volatile uint32_t JOFR4;
  volatile uint32_t HTR;
  volatile uint32_t LTR;
  volatile uint32_t SQR1;
  volatile uint32_t SQR2;
  volatile uint32_t SQR3;
  volatile uint32_t JSQR;
  volatile uint32_t JDR1;
  volatile uint32_t JDR2;
  volatile uint32_t JDR3;
  volatile uint32_t JDR4;
  volatile uint32_t DR;
} ADC_TypeDef;



 

typedef struct
{
  uint32_t  RESERVED0;
  volatile uint16_t DR1;
  uint16_t  RESERVED1;
  volatile uint16_t DR2;
  uint16_t  RESERVED2;
  volatile uint16_t DR3;
  uint16_t  RESERVED3;
  volatile uint16_t DR4;
  uint16_t  RESERVED4;
  volatile uint16_t DR5;
  uint16_t  RESERVED5;
  volatile uint16_t DR6;
  uint16_t  RESERVED6;
  volatile uint16_t DR7;
  uint16_t  RESERVED7;
  volatile uint16_t DR8;
  uint16_t  RESERVED8;
  volatile uint16_t DR9;
  uint16_t  RESERVED9;
  volatile uint16_t DR10;
  uint16_t  RESERVED10; 
  volatile uint16_t RTCCR;
  uint16_t  RESERVED11;
  volatile uint16_t CR;
  uint16_t  RESERVED12;
  volatile uint16_t CSR;
  uint16_t  RESERVED13[5];
  volatile uint16_t DR11;
  uint16_t  RESERVED14;
  volatile uint16_t DR12;
  uint16_t  RESERVED15;
  volatile uint16_t DR13;
  uint16_t  RESERVED16;
  volatile uint16_t DR14;
  uint16_t  RESERVED17;
  volatile uint16_t DR15;
  uint16_t  RESERVED18;
  volatile uint16_t DR16;
  uint16_t  RESERVED19;
  volatile uint16_t DR17;
  uint16_t  RESERVED20;
  volatile uint16_t DR18;
  uint16_t  RESERVED21;
  volatile uint16_t DR19;
  uint16_t  RESERVED22;
  volatile uint16_t DR20;
  uint16_t  RESERVED23;
  volatile uint16_t DR21;
  uint16_t  RESERVED24;
  volatile uint16_t DR22;
  uint16_t  RESERVED25;
  volatile uint16_t DR23;
  uint16_t  RESERVED26;
  volatile uint16_t DR24;
  uint16_t  RESERVED27;
  volatile uint16_t DR25;
  uint16_t  RESERVED28;
  volatile uint16_t DR26;
  uint16_t  RESERVED29;
  volatile uint16_t DR27;
  uint16_t  RESERVED30;
  volatile uint16_t DR28;
  uint16_t  RESERVED31;
  volatile uint16_t DR29;
  uint16_t  RESERVED32;
  volatile uint16_t DR30;
  uint16_t  RESERVED33; 
  volatile uint16_t DR31;
  uint16_t  RESERVED34;
  volatile uint16_t DR32;
  uint16_t  RESERVED35;
  volatile uint16_t DR33;
  uint16_t  RESERVED36;
  volatile uint16_t DR34;
  uint16_t  RESERVED37;
  volatile uint16_t DR35;
  uint16_t  RESERVED38;
  volatile uint16_t DR36;
  uint16_t  RESERVED39;
  volatile uint16_t DR37;
  uint16_t  RESERVED40;
  volatile uint16_t DR38;
  uint16_t  RESERVED41;
  volatile uint16_t DR39;
  uint16_t  RESERVED42;
  volatile uint16_t DR40;
  uint16_t  RESERVED43;
  volatile uint16_t DR41;
  uint16_t  RESERVED44;
  volatile uint16_t DR42;
  uint16_t  RESERVED45;    
} BKP_TypeDef;
  


 

typedef struct
{
  volatile uint32_t TIR;
  volatile uint32_t TDTR;
  volatile uint32_t TDLR;
  volatile uint32_t TDHR;
} CAN_TxMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t RIR;
  volatile uint32_t RDTR;
  volatile uint32_t RDLR;
  volatile uint32_t RDHR;
} CAN_FIFOMailBox_TypeDef;



 
  
typedef struct
{
  volatile uint32_t FR1;
  volatile uint32_t FR2;
} CAN_FilterRegister_TypeDef;



 
  
typedef struct
{
  volatile uint32_t MCR;
  volatile uint32_t MSR;
  volatile uint32_t TSR;
  volatile uint32_t RF0R;
  volatile uint32_t RF1R;
  volatile uint32_t IER;
  volatile uint32_t ESR;
  volatile uint32_t BTR;
  uint32_t  RESERVED0[88];
  CAN_TxMailBox_TypeDef sTxMailBox[3];
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
  uint32_t  RESERVED1[12];
  volatile uint32_t FMR;
  volatile uint32_t FM1R;
  uint32_t  RESERVED2;
  volatile uint32_t FS1R;
  uint32_t  RESERVED3;
  volatile uint32_t FFA1R;
  uint32_t  RESERVED4;
  volatile uint32_t FA1R;
  uint32_t  RESERVED5[8];

  CAN_FilterRegister_TypeDef sFilterRegister[14];



} CAN_TypeDef;



 
typedef struct
{
  volatile uint32_t CFGR;
  volatile uint32_t OAR;
  volatile uint32_t PRES;
  volatile uint32_t ESR;
  volatile uint32_t CSR;
  volatile uint32_t TXD;
  volatile uint32_t RXD;  
} CEC_TypeDef;



 

typedef struct
{
  volatile uint32_t DR;
  volatile uint8_t  IDR;
  uint8_t   RESERVED0;
  uint16_t  RESERVED1;
  volatile uint32_t CR;
} CRC_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t SWTRIGR;
  volatile uint32_t DHR12R1;
  volatile uint32_t DHR12L1;
  volatile uint32_t DHR8R1;
  volatile uint32_t DHR12R2;
  volatile uint32_t DHR12L2;
  volatile uint32_t DHR8R2;
  volatile uint32_t DHR12RD;
  volatile uint32_t DHR12LD;
  volatile uint32_t DHR8RD;
  volatile uint32_t DOR1;
  volatile uint32_t DOR2;



} DAC_TypeDef;



 

typedef struct
{
  volatile uint32_t IDCODE;
  volatile uint32_t CR;	
}DBGMCU_TypeDef;



 

typedef struct
{
  volatile uint32_t CCR;
  volatile uint32_t CNDTR;
  volatile uint32_t CPAR;
  volatile uint32_t CMAR;
} DMA_Channel_TypeDef;

typedef struct
{
  volatile uint32_t ISR;
  volatile uint32_t IFCR;
} DMA_TypeDef;



 

typedef struct
{
  volatile uint32_t MACCR;
  volatile uint32_t MACFFR;
  volatile uint32_t MACHTHR;
  volatile uint32_t MACHTLR;
  volatile uint32_t MACMIIAR;
  volatile uint32_t MACMIIDR;
  volatile uint32_t MACFCR;
  volatile uint32_t MACVLANTR;              
       uint32_t RESERVED0[2];
  volatile uint32_t MACRWUFFR;              
  volatile uint32_t MACPMTCSR;
       uint32_t RESERVED1[2];
  volatile uint32_t MACSR;                  
  volatile uint32_t MACIMR;
  volatile uint32_t MACA0HR;
  volatile uint32_t MACA0LR;
  volatile uint32_t MACA1HR;
  volatile uint32_t MACA1LR;
  volatile uint32_t MACA2HR;
  volatile uint32_t MACA2LR;
  volatile uint32_t MACA3HR;
  volatile uint32_t MACA3LR;                
       uint32_t RESERVED2[40];
  volatile uint32_t MMCCR;                  
  volatile uint32_t MMCRIR;
  volatile uint32_t MMCTIR;
  volatile uint32_t MMCRIMR;
  volatile uint32_t MMCTIMR;                
       uint32_t RESERVED3[14];
  volatile uint32_t MMCTGFSCCR;             
  volatile uint32_t MMCTGFMSCCR;
       uint32_t RESERVED4[5];
  volatile uint32_t MMCTGFCR;
       uint32_t RESERVED5[10];
  volatile uint32_t MMCRFCECR;
  volatile uint32_t MMCRFAECR;
       uint32_t RESERVED6[10];
  volatile uint32_t MMCRGUFCR;
       uint32_t RESERVED7[334];
  volatile uint32_t PTPTSCR;
  volatile uint32_t PTPSSIR;
  volatile uint32_t PTPTSHR;
  volatile uint32_t PTPTSLR;
  volatile uint32_t PTPTSHUR;
  volatile uint32_t PTPTSLUR;
  volatile uint32_t PTPTSAR;
  volatile uint32_t PTPTTHR;
  volatile uint32_t PTPTTLR;
       uint32_t RESERVED8[567];
  volatile uint32_t DMABMR;
  volatile uint32_t DMATPDR;
  volatile uint32_t DMARPDR;
  volatile uint32_t DMARDLAR;
  volatile uint32_t DMATDLAR;
  volatile uint32_t DMASR;
  volatile uint32_t DMAOMR;
  volatile uint32_t DMAIER;
  volatile uint32_t DMAMFBOCR;
       uint32_t RESERVED9[9];
  volatile uint32_t DMACHTDR;
  volatile uint32_t DMACHRDR;
  volatile uint32_t DMACHTBAR;
  volatile uint32_t DMACHRBAR;
} ETH_TypeDef;



 

typedef struct
{
  volatile uint32_t IMR;
  volatile uint32_t EMR;
  volatile uint32_t RTSR;
  volatile uint32_t FTSR;
  volatile uint32_t SWIER;
  volatile uint32_t PR;
} EXTI_TypeDef;



 

typedef struct
{
  volatile uint32_t ACR;
  volatile uint32_t KEYR;
  volatile uint32_t OPTKEYR;
  volatile uint32_t SR;
  volatile uint32_t CR;
  volatile uint32_t AR;
  volatile uint32_t RESERVED;
  volatile uint32_t OBR;
  volatile uint32_t WRPR;
#line 920 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"
} FLASH_TypeDef;



 
  
typedef struct
{
  volatile uint16_t RDP;
  volatile uint16_t USER;
  volatile uint16_t Data0;
  volatile uint16_t Data1;
  volatile uint16_t WRP0;
  volatile uint16_t WRP1;
  volatile uint16_t WRP2;
  volatile uint16_t WRP3;
} OB_TypeDef;



 

typedef struct
{
  volatile uint32_t BTCR[8];   
} FSMC_Bank1_TypeDef; 



 
  
typedef struct
{
  volatile uint32_t BWTR[7];
} FSMC_Bank1E_TypeDef;



 
  
typedef struct
{
  volatile uint32_t PCR2;
  volatile uint32_t SR2;
  volatile uint32_t PMEM2;
  volatile uint32_t PATT2;
  uint32_t  RESERVED0;   
  volatile uint32_t ECCR2; 
} FSMC_Bank2_TypeDef;  



 
  
typedef struct
{
  volatile uint32_t PCR3;
  volatile uint32_t SR3;
  volatile uint32_t PMEM3;
  volatile uint32_t PATT3;
  uint32_t  RESERVED0;   
  volatile uint32_t ECCR3; 
} FSMC_Bank3_TypeDef; 



 
  
typedef struct
{
  volatile uint32_t PCR4;
  volatile uint32_t SR4;
  volatile uint32_t PMEM4;
  volatile uint32_t PATT4;
  volatile uint32_t PIO4; 
} FSMC_Bank4_TypeDef; 



 

typedef struct
{
  volatile uint32_t CRL;
  volatile uint32_t CRH;
  volatile uint32_t IDR;
  volatile uint32_t ODR;
  volatile uint32_t BSRR;
  volatile uint32_t BRR;
  volatile uint32_t LCKR;
} GPIO_TypeDef;



 

typedef struct
{
  volatile uint32_t EVCR;
  volatile uint32_t MAPR;
  volatile uint32_t EXTICR[4];
  uint32_t RESERVED0;
  volatile uint32_t MAPR2;  
} AFIO_TypeDef;


 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t OAR1;
  uint16_t  RESERVED2;
  volatile uint16_t OAR2;
  uint16_t  RESERVED3;
  volatile uint16_t DR;
  uint16_t  RESERVED4;
  volatile uint16_t SR1;
  uint16_t  RESERVED5;
  volatile uint16_t SR2;
  uint16_t  RESERVED6;
  volatile uint16_t CCR;
  uint16_t  RESERVED7;
  volatile uint16_t TRISE;
  uint16_t  RESERVED8;
} I2C_TypeDef;



 

typedef struct
{
  volatile uint32_t KR;
  volatile uint32_t PR;
  volatile uint32_t RLR;
  volatile uint32_t SR;
} IWDG_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CSR;
} PWR_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFGR;
  volatile uint32_t CIR;
  volatile uint32_t APB2RSTR;
  volatile uint32_t APB1RSTR;
  volatile uint32_t AHBENR;
  volatile uint32_t APB2ENR;
  volatile uint32_t APB1ENR;
  volatile uint32_t BDCR;
  volatile uint32_t CSR;










} RCC_TypeDef;



 

typedef struct
{
  volatile uint16_t CRH;
  uint16_t  RESERVED0;
  volatile uint16_t CRL;
  uint16_t  RESERVED1;
  volatile uint16_t PRLH;
  uint16_t  RESERVED2;
  volatile uint16_t PRLL;
  uint16_t  RESERVED3;
  volatile uint16_t DIVH;
  uint16_t  RESERVED4;
  volatile uint16_t DIVL;
  uint16_t  RESERVED5;
  volatile uint16_t CNTH;
  uint16_t  RESERVED6;
  volatile uint16_t CNTL;
  uint16_t  RESERVED7;
  volatile uint16_t ALRH;
  uint16_t  RESERVED8;
  volatile uint16_t ALRL;
  uint16_t  RESERVED9;
} RTC_TypeDef;



 

typedef struct
{
  volatile uint32_t POWER;
  volatile uint32_t CLKCR;
  volatile uint32_t ARG;
  volatile uint32_t CMD;
  volatile const uint32_t RESPCMD;
  volatile const uint32_t RESP1;
  volatile const uint32_t RESP2;
  volatile const uint32_t RESP3;
  volatile const uint32_t RESP4;
  volatile uint32_t DTIMER;
  volatile uint32_t DLEN;
  volatile uint32_t DCTRL;
  volatile const uint32_t DCOUNT;
  volatile const uint32_t STA;
  volatile uint32_t ICR;
  volatile uint32_t MASK;
  uint32_t  RESERVED0[2];
  volatile const uint32_t FIFOCNT;
  uint32_t  RESERVED1[13];
  volatile uint32_t FIFO;
} SDIO_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t SR;
  uint16_t  RESERVED2;
  volatile uint16_t DR;
  uint16_t  RESERVED3;
  volatile uint16_t CRCPR;
  uint16_t  RESERVED4;
  volatile uint16_t RXCRCR;
  uint16_t  RESERVED5;
  volatile uint16_t TXCRCR;
  uint16_t  RESERVED6;
  volatile uint16_t I2SCFGR;
  uint16_t  RESERVED7;
  volatile uint16_t I2SPR;
  uint16_t  RESERVED8;  
} SPI_TypeDef;



 

typedef struct
{
  volatile uint16_t CR1;
  uint16_t  RESERVED0;
  volatile uint16_t CR2;
  uint16_t  RESERVED1;
  volatile uint16_t SMCR;
  uint16_t  RESERVED2;
  volatile uint16_t DIER;
  uint16_t  RESERVED3;
  volatile uint16_t SR;
  uint16_t  RESERVED4;
  volatile uint16_t EGR;
  uint16_t  RESERVED5;
  volatile uint16_t CCMR1;
  uint16_t  RESERVED6;
  volatile uint16_t CCMR2;
  uint16_t  RESERVED7;
  volatile uint16_t CCER;
  uint16_t  RESERVED8;
  volatile uint16_t CNT;
  uint16_t  RESERVED9;
  volatile uint16_t PSC;
  uint16_t  RESERVED10;
  volatile uint16_t ARR;
  uint16_t  RESERVED11;
  volatile uint16_t RCR;
  uint16_t  RESERVED12;
  volatile uint16_t CCR1;
  uint16_t  RESERVED13;
  volatile uint16_t CCR2;
  uint16_t  RESERVED14;
  volatile uint16_t CCR3;
  uint16_t  RESERVED15;
  volatile uint16_t CCR4;
  uint16_t  RESERVED16;
  volatile uint16_t BDTR;
  uint16_t  RESERVED17;
  volatile uint16_t DCR;
  uint16_t  RESERVED18;
  volatile uint16_t DMAR;
  uint16_t  RESERVED19;
} TIM_TypeDef;



 
 
typedef struct
{
  volatile uint16_t SR;
  uint16_t  RESERVED0;
  volatile uint16_t DR;
  uint16_t  RESERVED1;
  volatile uint16_t BRR;
  uint16_t  RESERVED2;
  volatile uint16_t CR1;
  uint16_t  RESERVED3;
  volatile uint16_t CR2;
  uint16_t  RESERVED4;
  volatile uint16_t CR3;
  uint16_t  RESERVED5;
  volatile uint16_t GTPR;
  uint16_t  RESERVED6;
} USART_TypeDef;



 

typedef struct
{
  volatile uint32_t CR;
  volatile uint32_t CFR;
  volatile uint32_t SR;
} WWDG_TypeDef;



 
  


 











 




#line 1312 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 1335 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



#line 1354 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"




















 
  


   

#line 1454 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 



 
  
  

 
    
 
 
 

 
 
 
 
 

 



 



 


 
 
 
 
 

 











 
#line 1515 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"




 





 
 
 
 
 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 





 



 






 
 
 
 
 

 
#line 1691 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 1698 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
 








 








 






#line 1734 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 











 











 













 






#line 1850 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"




#line 1870 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 





#line 1883 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 1902 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 1911 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 1919 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



















#line 1944 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"












 













#line 1976 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"





#line 1990 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 1997 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 2007 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"











 


















#line 2043 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2051 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



















#line 2076 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"












 













#line 2108 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"





#line 2122 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 2129 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 2139 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"











 








 








   
#line 2178 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 2273 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 2300 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"
 
 
 
 
 
 

 




































































 




































































 
#line 2462 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2480 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2498 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 2515 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2533 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2552 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 

 






 
#line 2579 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"






 








 









 








 








 









 










 




#line 2654 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 










#line 2685 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 





 
#line 2700 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2709 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

   
#line 2718 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2727 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 





 
#line 2742 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2751 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

   
#line 2760 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2769 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 





 
#line 2784 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2793 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

   
#line 2802 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2811 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 





 
#line 2826 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2835 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

   
#line 2844 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2853 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 2862 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 2871 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 2881 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
 
 
 
 

 





 


 


 




 
 
 
 
 

 
#line 2945 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 2980 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3015 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3050 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3085 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 





 





 





 





 





 





 





 





 






 
#line 3152 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 



 









 
#line 3176 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"




 




 
#line 3192 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 





 
#line 3214 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
 





 
#line 3229 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"
 
#line 3236 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 




 






 


 


 


 
 
 
 
 

 
#line 3285 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3307 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3329 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3351 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3373 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3395 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
 
 
 
 

 
#line 3431 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3461 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3471 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"















 
#line 3495 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"















 
#line 3519 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"















 
#line 3543 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"















 
#line 3567 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"















 
#line 3591 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"















 
#line 3615 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"















 


 


 


 


 


 


 


 


 


 



 


 


 



 


 


 


 



 


 


 


 


 
 
 
 
 

 






 
#line 3716 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 3725 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"















  
 
#line 3748 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"


















 








































 


















































 


 


 


 


 


 


 
#line 3883 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 3890 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 3897 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 3904 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"







 
#line 3918 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 3925 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 3932 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 3939 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 3946 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 3953 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 3961 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 3968 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 3975 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 3982 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 3989 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 3996 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 4004 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 4011 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 4018 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 4025 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"





 


 


 


 


 



 
 
 
 
 

 









































 



 


 


 


 


 


 


 



 



 



 


 


 



 
 
 
 
 
 





 






 


 
#line 4167 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 4177 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 


 


 
 
 
 
 

 
















 









#line 4225 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 

























 
#line 4268 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 4282 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 4292 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 




























 





















 




























 





















 
#line 4411 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
#line 4446 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"





#line 4457 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 4465 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 4472 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 


 
 
 
 
 

 




 
#line 4494 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
 
 
 
 

 


 





 


 



 
 
 
 
 

 
#line 4556 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 
#line 4568 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"







 


 
 
 
 
 

 











#line 4606 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 











#line 4629 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 











#line 4652 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 











#line 4675 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 








































 








































 








































 








































 


































 


































 


































 


































 



























 



























 



























 
#line 5072 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5081 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5090 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5101 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 5111 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 5121 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 5131 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5142 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 5152 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 5162 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 5172 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5183 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 5193 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 5203 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 5213 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5224 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 5234 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 5244 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 5254 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5265 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 5275 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 5285 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 5295 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5306 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 5316 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 5326 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 5336 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5347 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 5357 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 5367 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

#line 5377 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 


 


 
 
 
 
 

 




 












 


 






#line 5425 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 


 


 


 


 


 


 


 


 
















 


 
#line 5495 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5510 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5536 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 


 


 
 
 
 
 

 
 























 























 























 























 























 























 























 























 
 
#line 5757 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 5769 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 






 
#line 5786 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



     


 
 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 


 

 


 


 


 


 


 


 


 


 

 


#line 5930 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 


#line 5942 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 


#line 5954 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 


#line 5966 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 


#line 5978 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 


#line 5990 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6002 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6014 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 

 


#line 6028 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6040 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6052 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6064 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6076 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6088 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6100 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6112 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6124 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6136 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6148 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6160 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6172 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6184 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6196 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 


#line 6208 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 
 
 
 
 

 
 
#line 6228 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6239 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6257 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"











 





 





 
#line 6295 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 












 
#line 6316 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
 






 




 





 





 






 




 





 





 






   




 





 





 





 




 





 





 





 




 





 





 
 


 
#line 6456 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6473 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6490 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6507 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6541 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6575 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6609 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6643 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6677 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6711 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6745 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6779 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6813 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6847 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6881 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6915 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6949 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 6983 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7017 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7051 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7085 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7119 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7153 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7187 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7221 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7255 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7289 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7323 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7357 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7391 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7425 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7459 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
 
 
 
 

 









#line 7486 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7494 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7504 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 


 


 


 


 





















 




 
 
 
 
 

 
#line 7565 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7574 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"







 



#line 7595 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 



 


 
#line 7620 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7630 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 




 


 
 
 
 
 

 
#line 7656 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 


 



 
#line 7680 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7689 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"







 
#line 7709 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
#line 7720 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 
 
 
 
 

 


#line 7749 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 









#line 7783 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 
 
 
 
 

 









 


 


 





 
#line 7823 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"

 


 









 


 

 



 



 



 



 



 



 



 



#line 8287 "D:\\tools\\Keil\\ARM\\Inc\\ST\\STM32F10x\\stm32f10x.h"



 

 

  







 

















 









 

  

 

 
#line 32 "md5c.c"

#line 1 "D:\\tools\\Keil\\ARM\\RV31\\Inc\\string.h"
 
 
 
 




 








 











#line 37 "D:\\tools\\Keil\\ARM\\RV31\\Inc\\string.h"


  
  typedef unsigned int size_t;








extern __declspec(__nothrow) void *memcpy(void * __restrict  ,
                    const void * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) void *memmove(void *  ,
                    const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   







 
extern __declspec(__nothrow) char *strcpy(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncpy(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 

extern __declspec(__nothrow) char *strcat(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) char *strncat(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 






 

extern __declspec(__nothrow) int memcmp(const void *  , const void *  , size_t  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strcmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 
extern __declspec(__nothrow) int strncmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcasecmp(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   





 
extern __declspec(__nothrow) int strncasecmp(const char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






 
extern __declspec(__nothrow) int strcoll(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   







 

extern __declspec(__nothrow) size_t strxfrm(char * __restrict  , const char * __restrict  , size_t  ) __attribute__((__nonnull__(2)));
   













 


#line 184 "D:\\tools\\Keil\\ARM\\RV31\\Inc\\string.h"
extern __declspec(__nothrow) void *memchr(const void *  , int  , size_t  ) __attribute__((__nonnull__(1)));

   





 

#line 200 "D:\\tools\\Keil\\ARM\\RV31\\Inc\\string.h"
extern __declspec(__nothrow) char *strchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   




 

extern __declspec(__nothrow) size_t strcspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   




 

#line 223 "D:\\tools\\Keil\\ARM\\RV31\\Inc\\string.h"
extern __declspec(__nothrow) char *strpbrk(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   




 

#line 238 "D:\\tools\\Keil\\ARM\\RV31\\Inc\\string.h"
extern __declspec(__nothrow) char *strrchr(const char *  , int  ) __attribute__((__nonnull__(1)));

   





 

extern __declspec(__nothrow) size_t strspn(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));
   



 

#line 261 "D:\\tools\\Keil\\ARM\\RV31\\Inc\\string.h"
extern __declspec(__nothrow) char *strstr(const char *  , const char *  ) __attribute__((__nonnull__(1,2)));

   





 

extern __declspec(__nothrow) char *strtok(char * __restrict  , const char * __restrict  ) __attribute__((__nonnull__(2)));
extern __declspec(__nothrow) char *_strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

extern __declspec(__nothrow) char *strtok_r(char *  , const char *  , char **  ) __attribute__((__nonnull__(2,3)));

   

































 

extern __declspec(__nothrow) void *memset(void *  , int  , size_t  ) __attribute__((__nonnull__(1)));
   



 
extern __declspec(__nothrow) char *strerror(int  );
   





 
extern __declspec(__nothrow) size_t strlen(const char *  ) __attribute__((__nonnull__(1)));
   



 

extern __declspec(__nothrow) size_t strlcpy(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   
















 

extern __declspec(__nothrow) size_t strlcat(char *  , const char *  , size_t  ) __attribute__((__nonnull__(1,2)));
   






















 

extern __declspec(__nothrow) void _membitcpybl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpybb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpyhb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitcpywb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovebb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovehb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewl(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
extern __declspec(__nothrow) void _membitmovewb(void *  , const void *  , int  , int  , size_t  ) __attribute__((__nonnull__(1,2)));
    














































 







#line 493 "D:\\tools\\Keil\\ARM\\RV31\\Inc\\string.h"



 
#line 38 "md5c.c"


#line 1 "global.h"

 





 




 
typedef unsigned char *POINTER;


 
typedef unsigned short int UINT2;

 
typedef unsigned long int UINT4;
#line 31 "global.h"




 
#line 41 "md5c.c"
#line 1 "md5.h"


 





















 



 
typedef struct MD5Context {
  unsigned long state[4];	 
  unsigned long count[2];	 
  unsigned char buffer[64];	 
} MD5_CTX;

void   MD5Init (MD5_CTX *);
void   MD5Update (MD5_CTX *, const unsigned char *, unsigned int);
void   MD5Final (unsigned char [16], MD5_CTX *);
char * MD5End(MD5_CTX *, char *);
char * MD5File(const char *, char *);
char * MD5Data(const unsigned char *, unsigned int, char *);

#line 42 "md5c.c"







static void MD5Transform (UINT4 [4], const unsigned char [64]);














 

static void
Encode (output, input, len)
	unsigned char *output;
	UINT4 *input;
	unsigned int len;
{
	unsigned int i, j;

	for (i = 0, j = 0; j < len; i++, j += 4) {
		output[j] = (unsigned char)(input[i] & 0xff);
		output[j+1] = (unsigned char)((input[i] >> 8) & 0xff);
		output[j+2] = (unsigned char)((input[i] >> 16) & 0xff);
		output[j+3] = (unsigned char)((input[i] >> 24) & 0xff);
	}
}




 

static void
Decode (output, input, len)
	UINT4 *output;
	const unsigned char *input;
	unsigned int len;
{
	unsigned int i, j;

	for (i = 0, j = 0; j < len; i++, j += 4)
		output[i] = ((UINT4)input[j]) | (((UINT4)input[j+1]) << 8) |
		    (((UINT4)input[j+2]) << 16) | (((UINT4)input[j+3]) << 24);
}


static unsigned char PADDING[64] = {
  0x80, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

 





 





 
#line 140 "md5c.c"

 

void
MD5Init (context)
	MD5_CTX *context;
{

	context->count[0] = context->count[1] = 0;

	 
	context->state[0] = 0x67452301;
	context->state[1] = 0xefcdab89;
	context->state[2] = 0x98badcfe;
	context->state[3] = 0x10325476;
}





 

void MD5Update (context, input, inputLen)
	MD5_CTX *context;
	const unsigned char *input;
	unsigned int inputLen;
{
	unsigned int i, index, partLen;

	 
	index = (unsigned int)((context->count[0] >> 3) & 0x3F);

	 
	if ((context->count[0] += ((UINT4)inputLen << 3))
	    < ((UINT4)inputLen << 3))
		context->count[1]++;
	context->count[1] += ((UINT4)inputLen >> 29);

	partLen = 64 - index;

	 
	if (inputLen >= partLen) {
		memcpy((void *)&context->buffer[index], (void *)input,
		    partLen);
		MD5Transform (context->state, context->buffer);

		for (i = partLen; i + 63 < inputLen; i += 64)
			MD5Transform (context->state, &input[i]);

		index = 0;
	}
	else
		i = 0;

	 
	memcpy ((void *)&context->buffer[index], (void *)&input[i],
	    inputLen-i);
}




 

void
MD5Final (digest, context)
	unsigned char digest[16];
	MD5_CTX *context;
{
	unsigned char bits[8];
	unsigned int index, padLen;

	 
	Encode (bits, context->count, 8);

	 
	index = (unsigned int)((context->count[0] >> 3) & 0x3f);
	padLen = (index < 56) ? (56 - index) : (120 - index);
	MD5Update (context, PADDING, padLen);

	 
	MD5Update (context, bits, 8);

	 
	Encode (digest, context->state, 16);

	 
	memset ((void *)context, 0, sizeof (*context));
}

 

static void
MD5Transform (state, block)
	UINT4 state[4];
	const unsigned char block[64];
{
	UINT4 a = state[0], b = state[1], c = state[2], d = state[3], x[16];

	Decode (x, block, 64);

	 




	{ (a) += ((((b)) & ((c))) | ((~(b)) & ((d)))) + (x[ 0]) + (UINT4)(0xd76aa478); (a) = ((((a)) << ((7))) | (((a)) >> (32-((7))))); (a) += (b); };  
	{ (d) += ((((a)) & ((b))) | ((~(a)) & ((c)))) + (x[ 1]) + (UINT4)(0xe8c7b756); (d) = ((((d)) << ((12))) | (((d)) >> (32-((12))))); (d) += (a); };  
	{ (c) += ((((d)) & ((a))) | ((~(d)) & ((b)))) + (x[ 2]) + (UINT4)(0x242070db); (c) = ((((c)) << ((17))) | (((c)) >> (32-((17))))); (c) += (d); };  
	{ (b) += ((((c)) & ((d))) | ((~(c)) & ((a)))) + (x[ 3]) + (UINT4)(0xc1bdceee); (b) = ((((b)) << ((22))) | (((b)) >> (32-((22))))); (b) += (c); };  
	{ (a) += ((((b)) & ((c))) | ((~(b)) & ((d)))) + (x[ 4]) + (UINT4)(0xf57c0faf); (a) = ((((a)) << ((7))) | (((a)) >> (32-((7))))); (a) += (b); };  
	{ (d) += ((((a)) & ((b))) | ((~(a)) & ((c)))) + (x[ 5]) + (UINT4)(0x4787c62a); (d) = ((((d)) << ((12))) | (((d)) >> (32-((12))))); (d) += (a); };  
	{ (c) += ((((d)) & ((a))) | ((~(d)) & ((b)))) + (x[ 6]) + (UINT4)(0xa8304613); (c) = ((((c)) << ((17))) | (((c)) >> (32-((17))))); (c) += (d); };  
	{ (b) += ((((c)) & ((d))) | ((~(c)) & ((a)))) + (x[ 7]) + (UINT4)(0xfd469501); (b) = ((((b)) << ((22))) | (((b)) >> (32-((22))))); (b) += (c); };  
	{ (a) += ((((b)) & ((c))) | ((~(b)) & ((d)))) + (x[ 8]) + (UINT4)(0x698098d8); (a) = ((((a)) << ((7))) | (((a)) >> (32-((7))))); (a) += (b); };  
	{ (d) += ((((a)) & ((b))) | ((~(a)) & ((c)))) + (x[ 9]) + (UINT4)(0x8b44f7af); (d) = ((((d)) << ((12))) | (((d)) >> (32-((12))))); (d) += (a); };  
	{ (c) += ((((d)) & ((a))) | ((~(d)) & ((b)))) + (x[10]) + (UINT4)(0xffff5bb1); (c) = ((((c)) << ((17))) | (((c)) >> (32-((17))))); (c) += (d); };  
	{ (b) += ((((c)) & ((d))) | ((~(c)) & ((a)))) + (x[11]) + (UINT4)(0x895cd7be); (b) = ((((b)) << ((22))) | (((b)) >> (32-((22))))); (b) += (c); };  
	{ (a) += ((((b)) & ((c))) | ((~(b)) & ((d)))) + (x[12]) + (UINT4)(0x6b901122); (a) = ((((a)) << ((7))) | (((a)) >> (32-((7))))); (a) += (b); };  
	{ (d) += ((((a)) & ((b))) | ((~(a)) & ((c)))) + (x[13]) + (UINT4)(0xfd987193); (d) = ((((d)) << ((12))) | (((d)) >> (32-((12))))); (d) += (a); };  
	{ (c) += ((((d)) & ((a))) | ((~(d)) & ((b)))) + (x[14]) + (UINT4)(0xa679438e); (c) = ((((c)) << ((17))) | (((c)) >> (32-((17))))); (c) += (d); };  
	{ (b) += ((((c)) & ((d))) | ((~(c)) & ((a)))) + (x[15]) + (UINT4)(0x49b40821); (b) = ((((b)) << ((22))) | (((b)) >> (32-((22))))); (b) += (c); };  

	 




	{ (a) += ((((b)) & ((d))) | (((c)) & (~(d)))) + (x[ 1]) + (UINT4)(0xf61e2562); (a) = ((((a)) << ((5))) | (((a)) >> (32-((5))))); (a) += (b); };  
	{ (d) += ((((a)) & ((c))) | (((b)) & (~(c)))) + (x[ 6]) + (UINT4)(0xc040b340); (d) = ((((d)) << ((9))) | (((d)) >> (32-((9))))); (d) += (a); };  
	{ (c) += ((((d)) & ((b))) | (((a)) & (~(b)))) + (x[11]) + (UINT4)(0x265e5a51); (c) = ((((c)) << ((14))) | (((c)) >> (32-((14))))); (c) += (d); };  
	{ (b) += ((((c)) & ((a))) | (((d)) & (~(a)))) + (x[ 0]) + (UINT4)(0xe9b6c7aa); (b) = ((((b)) << ((20))) | (((b)) >> (32-((20))))); (b) += (c); };  
	{ (a) += ((((b)) & ((d))) | (((c)) & (~(d)))) + (x[ 5]) + (UINT4)(0xd62f105d); (a) = ((((a)) << ((5))) | (((a)) >> (32-((5))))); (a) += (b); };  
	{ (d) += ((((a)) & ((c))) | (((b)) & (~(c)))) + (x[10]) + (UINT4)(0x2441453); (d) = ((((d)) << ((9))) | (((d)) >> (32-((9))))); (d) += (a); };  
	{ (c) += ((((d)) & ((b))) | (((a)) & (~(b)))) + (x[15]) + (UINT4)(0xd8a1e681); (c) = ((((c)) << ((14))) | (((c)) >> (32-((14))))); (c) += (d); };  
	{ (b) += ((((c)) & ((a))) | (((d)) & (~(a)))) + (x[ 4]) + (UINT4)(0xe7d3fbc8); (b) = ((((b)) << ((20))) | (((b)) >> (32-((20))))); (b) += (c); };  
	{ (a) += ((((b)) & ((d))) | (((c)) & (~(d)))) + (x[ 9]) + (UINT4)(0x21e1cde6); (a) = ((((a)) << ((5))) | (((a)) >> (32-((5))))); (a) += (b); };  
	{ (d) += ((((a)) & ((c))) | (((b)) & (~(c)))) + (x[14]) + (UINT4)(0xc33707d6); (d) = ((((d)) << ((9))) | (((d)) >> (32-((9))))); (d) += (a); };  
	{ (c) += ((((d)) & ((b))) | (((a)) & (~(b)))) + (x[ 3]) + (UINT4)(0xf4d50d87); (c) = ((((c)) << ((14))) | (((c)) >> (32-((14))))); (c) += (d); };  
	{ (b) += ((((c)) & ((a))) | (((d)) & (~(a)))) + (x[ 8]) + (UINT4)(0x455a14ed); (b) = ((((b)) << ((20))) | (((b)) >> (32-((20))))); (b) += (c); };  
	{ (a) += ((((b)) & ((d))) | (((c)) & (~(d)))) + (x[13]) + (UINT4)(0xa9e3e905); (a) = ((((a)) << ((5))) | (((a)) >> (32-((5))))); (a) += (b); };  
	{ (d) += ((((a)) & ((c))) | (((b)) & (~(c)))) + (x[ 2]) + (UINT4)(0xfcefa3f8); (d) = ((((d)) << ((9))) | (((d)) >> (32-((9))))); (d) += (a); };  
	{ (c) += ((((d)) & ((b))) | (((a)) & (~(b)))) + (x[ 7]) + (UINT4)(0x676f02d9); (c) = ((((c)) << ((14))) | (((c)) >> (32-((14))))); (c) += (d); };  
	{ (b) += ((((c)) & ((a))) | (((d)) & (~(a)))) + (x[12]) + (UINT4)(0x8d2a4c8a); (b) = ((((b)) << ((20))) | (((b)) >> (32-((20))))); (b) += (c); };  

	 




	{ (a) += (((b)) ^ ((c)) ^ ((d))) + (x[ 5]) + (UINT4)(0xfffa3942); (a) = ((((a)) << ((4))) | (((a)) >> (32-((4))))); (a) += (b); };  
	{ (d) += (((a)) ^ ((b)) ^ ((c))) + (x[ 8]) + (UINT4)(0x8771f681); (d) = ((((d)) << ((11))) | (((d)) >> (32-((11))))); (d) += (a); };  
	{ (c) += (((d)) ^ ((a)) ^ ((b))) + (x[11]) + (UINT4)(0x6d9d6122); (c) = ((((c)) << ((16))) | (((c)) >> (32-((16))))); (c) += (d); };  
	{ (b) += (((c)) ^ ((d)) ^ ((a))) + (x[14]) + (UINT4)(0xfde5380c); (b) = ((((b)) << ((23))) | (((b)) >> (32-((23))))); (b) += (c); };  
	{ (a) += (((b)) ^ ((c)) ^ ((d))) + (x[ 1]) + (UINT4)(0xa4beea44); (a) = ((((a)) << ((4))) | (((a)) >> (32-((4))))); (a) += (b); };  
	{ (d) += (((a)) ^ ((b)) ^ ((c))) + (x[ 4]) + (UINT4)(0x4bdecfa9); (d) = ((((d)) << ((11))) | (((d)) >> (32-((11))))); (d) += (a); };  
	{ (c) += (((d)) ^ ((a)) ^ ((b))) + (x[ 7]) + (UINT4)(0xf6bb4b60); (c) = ((((c)) << ((16))) | (((c)) >> (32-((16))))); (c) += (d); };  
	{ (b) += (((c)) ^ ((d)) ^ ((a))) + (x[10]) + (UINT4)(0xbebfbc70); (b) = ((((b)) << ((23))) | (((b)) >> (32-((23))))); (b) += (c); };  
	{ (a) += (((b)) ^ ((c)) ^ ((d))) + (x[13]) + (UINT4)(0x289b7ec6); (a) = ((((a)) << ((4))) | (((a)) >> (32-((4))))); (a) += (b); };  
	{ (d) += (((a)) ^ ((b)) ^ ((c))) + (x[ 0]) + (UINT4)(0xeaa127fa); (d) = ((((d)) << ((11))) | (((d)) >> (32-((11))))); (d) += (a); };  
	{ (c) += (((d)) ^ ((a)) ^ ((b))) + (x[ 3]) + (UINT4)(0xd4ef3085); (c) = ((((c)) << ((16))) | (((c)) >> (32-((16))))); (c) += (d); };  
	{ (b) += (((c)) ^ ((d)) ^ ((a))) + (x[ 6]) + (UINT4)(0x4881d05); (b) = ((((b)) << ((23))) | (((b)) >> (32-((23))))); (b) += (c); };  
	{ (a) += (((b)) ^ ((c)) ^ ((d))) + (x[ 9]) + (UINT4)(0xd9d4d039); (a) = ((((a)) << ((4))) | (((a)) >> (32-((4))))); (a) += (b); };  
	{ (d) += (((a)) ^ ((b)) ^ ((c))) + (x[12]) + (UINT4)(0xe6db99e5); (d) = ((((d)) << ((11))) | (((d)) >> (32-((11))))); (d) += (a); };  
	{ (c) += (((d)) ^ ((a)) ^ ((b))) + (x[15]) + (UINT4)(0x1fa27cf8); (c) = ((((c)) << ((16))) | (((c)) >> (32-((16))))); (c) += (d); };  
	{ (b) += (((c)) ^ ((d)) ^ ((a))) + (x[ 2]) + (UINT4)(0xc4ac5665); (b) = ((((b)) << ((23))) | (((b)) >> (32-((23))))); (b) += (c); };  

	 




	{ (a) += (((c)) ^ (((b)) | (~(d)))) + (x[ 0]) + (UINT4)(0xf4292244); (a) = ((((a)) << ((6))) | (((a)) >> (32-((6))))); (a) += (b); };  
	{ (d) += (((b)) ^ (((a)) | (~(c)))) + (x[ 7]) + (UINT4)(0x432aff97); (d) = ((((d)) << ((10))) | (((d)) >> (32-((10))))); (d) += (a); };  
	{ (c) += (((a)) ^ (((d)) | (~(b)))) + (x[14]) + (UINT4)(0xab9423a7); (c) = ((((c)) << ((15))) | (((c)) >> (32-((15))))); (c) += (d); };  
	{ (b) += (((d)) ^ (((c)) | (~(a)))) + (x[ 5]) + (UINT4)(0xfc93a039); (b) = ((((b)) << ((21))) | (((b)) >> (32-((21))))); (b) += (c); };  
	{ (a) += (((c)) ^ (((b)) | (~(d)))) + (x[12]) + (UINT4)(0x655b59c3); (a) = ((((a)) << ((6))) | (((a)) >> (32-((6))))); (a) += (b); };  
	{ (d) += (((b)) ^ (((a)) | (~(c)))) + (x[ 3]) + (UINT4)(0x8f0ccc92); (d) = ((((d)) << ((10))) | (((d)) >> (32-((10))))); (d) += (a); };  
	{ (c) += (((a)) ^ (((d)) | (~(b)))) + (x[10]) + (UINT4)(0xffeff47d); (c) = ((((c)) << ((15))) | (((c)) >> (32-((15))))); (c) += (d); };  
	{ (b) += (((d)) ^ (((c)) | (~(a)))) + (x[ 1]) + (UINT4)(0x85845dd1); (b) = ((((b)) << ((21))) | (((b)) >> (32-((21))))); (b) += (c); };  
	{ (a) += (((c)) ^ (((b)) | (~(d)))) + (x[ 8]) + (UINT4)(0x6fa87e4f); (a) = ((((a)) << ((6))) | (((a)) >> (32-((6))))); (a) += (b); };  
	{ (d) += (((b)) ^ (((a)) | (~(c)))) + (x[15]) + (UINT4)(0xfe2ce6e0); (d) = ((((d)) << ((10))) | (((d)) >> (32-((10))))); (d) += (a); };  
	{ (c) += (((a)) ^ (((d)) | (~(b)))) + (x[ 6]) + (UINT4)(0xa3014314); (c) = ((((c)) << ((15))) | (((c)) >> (32-((15))))); (c) += (d); };  
	{ (b) += (((d)) ^ (((c)) | (~(a)))) + (x[13]) + (UINT4)(0x4e0811a1); (b) = ((((b)) << ((21))) | (((b)) >> (32-((21))))); (b) += (c); };  
	{ (a) += (((c)) ^ (((b)) | (~(d)))) + (x[ 4]) + (UINT4)(0xf7537e82); (a) = ((((a)) << ((6))) | (((a)) >> (32-((6))))); (a) += (b); };  
	{ (d) += (((b)) ^ (((a)) | (~(c)))) + (x[11]) + (UINT4)(0xbd3af235); (d) = ((((d)) << ((10))) | (((d)) >> (32-((10))))); (d) += (a); };  
	{ (c) += (((a)) ^ (((d)) | (~(b)))) + (x[ 2]) + (UINT4)(0x2ad7d2bb); (c) = ((((c)) << ((15))) | (((c)) >> (32-((15))))); (c) += (d); };  
	{ (b) += (((d)) ^ (((c)) | (~(a)))) + (x[ 9]) + (UINT4)(0xeb86d391); (b) = ((((b)) << ((21))) | (((b)) >> (32-((21))))); (b) += (c); };  

	state[0] += a;
	state[1] += b;
	state[2] += c;
	state[3] += d;

	 
	memset ((void *)x, 0, sizeof (x));
}
