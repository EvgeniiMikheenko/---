//==============================================================================
//
//==============================================================================

#ifndef _APP_TYPES_H_
#define _APP_TYPES_H_

#include <stdint.h>

#define MAX_PEAKS   						8
#define MAX_POINTS  						8192

#define PEAK_PARAM_TABLE_SIZE   			(MAX_PEAKS * 2 * 3)
#define PEAK_FOUND_TABLE_SIZE   			(MAX_PEAKS * 2)

//#define PASSWORD_REGISTERS_COUNT			(8 /* Password */ + 2 /* CRC */)

typedef struct { 	// Peaks_Table_Struct
    unsigned short Time;
    unsigned short Value;
} Peaks_Table_Struct;

//Data to FPGA
typedef struct { 	// FPGA_Out_Struct
    unsigned short PointsScale;
    unsigned short FPGA_Flags;
    unsigned short hv_impulse_length;
    unsigned short hv_polarity_length;
    unsigned short PeakParamTable[PEAK_PARAM_TABLE_SIZE];
    unsigned short Dummy[9];
} FPGA_Out_Struct;

//Data form FPGA
typedef struct {	// FPGA_In_Struct
    //union
    //unsigned short  Flags;
    //unsigned Disperse;
    //unsigned short Dummy[4];
    unsigned short     Flags_FPGA2;
    //        unsigned short PeaksFoundTable[PEAK_FOUND_TABLE_SIZE];
    Peaks_Table_Struct PeaksFoundTable[MAX_PEAKS * 2];
    unsigned short     Flags_FPGA;
    unsigned short     PeakCount;
    unsigned short     Shelf;
    unsigned short     Integral;
    unsigned short     Dummy1[30];
    unsigned short     Points[MAX_POINTS];
    //unsigned short Dummy2[9];
} FPGA_In_Struct;



//#pragma pack(4)
#pragma pack (push, 4)
typedef struct {	// FPGA_In_Peaks_Struct

    unsigned short     Flags_FPGA2;

    Peaks_Table_Struct PeaksFoundTable[MAX_PEAKS * 2];
    unsigned short     Flags_FPGA;
    unsigned short     PeakCount;
    unsigned short     Dummy1[46];

} FPGA_In_Peaks_Struct;


//Data to us
typedef struct {	// CPU_In_Struct
    //unsigned short Flags;
    unsigned short Flow_Total_Setpoint;
    unsigned short Flow_In_Setpoint;
    unsigned short Pump_Level;
    unsigned short Valve_Level;
    unsigned short T1_Setpoint;
    unsigned short T2_Setpoint;
    unsigned short HV_Setpoint;
    unsigned short Dummy[31];
    unsigned short Flow_Set_Period;
    unsigned short Dummy1[8];
    unsigned short Substance;
    unsigned short RTC_Sec;
    unsigned short RTC_Min;
    unsigned short RTC_Hour;
    unsigned short RTC_Day;
    unsigned short RTC_Month;
    unsigned short RTC_Year;
    unsigned short RTC_CTL;
    unsigned short Dummy2[10];

} CPU_In_Struct;

typedef struct {	// CPU_Out_Struct

    unsigned short Flags;
    short          Temp1;
    short          Temp2;
    unsigned short Heater_Power;

    unsigned short Temp_Case;
    unsigned short Athm_Pressure;
    unsigned short Valve_Level;
    unsigned short Pump_Total_Flow;
    unsigned short HV_Value;
    unsigned short Pump_Time_Low;
    unsigned short Pump_Time_High;

    unsigned short Humidity;

    unsigned short Flow_Total;
    unsigned short Flow_In;
    unsigned short Device_Mode;
    unsigned short Substance;
    unsigned short FirmWare_Version;

    unsigned short Dummy1[46];

} CPU_Out_Struct;

#pragma anon_unions
typedef union  { 	// FloatRegister_t
	float value;
	//uint32_t hex;
	struct {
		uint16_t Low;
		uint16_t Hi;
	};
} FloatRegister_t;

typedef struct { 	// PidRegsParams_t
	FloatRegister_t kP;
	FloatRegister_t kI;
	FloatRegister_t kD;
} PidRegsParams_t;

typedef struct { 	// PidRegsData_t
	FloatRegister_t pValue;
	FloatRegister_t iValue;
	FloatRegister_t dValue;
	FloatRegister_t errSumm;
	FloatRegister_t outValue;
	FloatRegister_t sp;
} PidRegsData_t;

typedef struct { 	// DeviceConfig_t
	uint16_t 		UsartBaudrate; 			// RW
	uint16_t 		UsartStopBits; 			// RW
	//
	uint16_t 		MbAddress; 				// RW
	// Нижнее значение по оси Y, ниже которого сигнал, при анализе не воспринимается
	uint16_t		WAYlimit;				// RW
	// Флаг использования нижнего значения по оси Y из репера
	uint16_t		WAYlimitInReperY;		// RW
	// Минимальное значение dY на подъем
	uint16_t		WAMinDYForUp;			// RW
	// Время выдержки не ниже минимального dY для начала определения подъема 
	uint16_t		WAOnTimeForUp;			// RW
	// Время выдержки ниже минимального dY, для сброса определения подъема
	uint16_t		WAOffTimeForUp;			// RW
	// Теже параметры, но для определения спуска
	uint16_t		WAMinDYForDown;			// RW
	uint16_t		WAOnTimeForDown;		// RW
	uint16_t		WAOffTimeForDown;		// RW
	//
	PidRegsParams_t Heater1PidFactors; 		// RW
	PidRegsParams_t FlowInPidFactors; 		// RW
	PidRegsParams_t FlowTotalPidFactors; 	// RW
} DeviceConfig_t;


typedef struct {	// Params_Struct

    //char Dummy[5-4];
    FPGA_Out_Struct FPGA_OUT;
    CPU_In_Struct   CPU_IN;
    unsigned short  CRC;
    CPU_Out_Struct  CPU_OUT;
    FPGA_In_Struct  FPGA_IN;
    //  FPGA_Data_Struct FPGA_Data;
	//--------------------------------
	// Config
	DeviceConfig_t	Config;
	uint16_t 		ConfigCRC;
	//--------------------------------
	PidRegsData_t	Heater1PidData;			// RO
	PidRegsData_t	FlowInPidData;			// RO
	PidRegsData_t	FlowTotalPidData;		// RO
	//uint16_t		Password[PASSWORD_REGISTERS_COUNT];
	
} Params_Struct;

#define MB_MAXPARAMS (sizeof(Params_Struct))

typedef struct { 	// HV_Packet_Struct
    //unsigned char  Sign;
    unsigned char  Flags;
    unsigned short HV_Value;
} HV_Packet_Struct;

typedef struct {	// ADC_Data_Struct
    unsigned short Data;
    unsigned short Error_Low;
    unsigned short Error_High;
    unsigned short Fail;
    unsigned short Fail_Param;
	bool		   Enable;
} ADC_Data_Struct;

#pragma anon_unions
typedef union  {	// Flags_t
	uint32_t Value;
	struct {
		unsigned IsCheckResultEn 		: 1; /* 0 */
		unsigned IsLastFound 			: 1; /* 1 */
		unsigned IsCurFound				: 1; /* 2 */
		unsigned IsHackEn				: 1; /* 3 */
		unsigned IsBlowDown				: 1; /* 4 */
		unsigned IsLockError			: 1; /* 5 */
		unsigned IsConfigChange			: 1; /* 6 */
		unsigned IsPidFactorsChange		: 1; /* 7 */
		unsigned IsFlowStable			: 1; /* 8 */
		unsigned IsTemp1Stable			: 1; /* 9 */
		unsigned IsModeOk				: 1; /* 10 */
		unsigned IsWaDataInvalid		: 1; /* 11 */
		unsigned IsWaReset 				: 1; /* 12 */
		unsigned IsBlowDownAlarm		: 1; /* 13 */
		unsigned IsDebugDisplayReset	: 1; /* 14 */
		unsigned IsNeedReadAdc			: 1; /* 15 */
		unsigned IsNeedUpdateDisplay	: 1; /* 16 */
	};
} Flags_t;

#pragma pack(pop)

#endif // _APP_TYPES_H_
