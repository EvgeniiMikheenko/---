//=============================================================================
//
//=============================================================================

#ifndef _HOST_H_
#define _HOST_H_

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

//#include "project.h"
//#include "board.h"

static const uint16_t InfoSizeRegNum	= 60000;
static const uint16_t InfoRegNum		= 60001;
static const uint16_t PasswordRegNum	= 59950;

#define MODBUS_PASSWORD					"LGAR MODBUS PASSWORD"
#define MODBUS_PASSWORD_LEN				20

#ifdef _USE_SIP1_DEVICE_
	#define DEVICE_INFO								"SIP-1 LI"
#else
	#ifdef _USE_SIP100_DEVICE_
		#ifdef _USE_DISPLAY_STRINGS_IPRIT_LUIZIT_
			#define DEVICE_INFO						"SIP-100 LI"
		#else
			#define DEVICE_INFO						"SIP-100"
		#endif
	#else
		#ifdef _USE_GS2R_DEVICE_
			#define DEVICE_INFO						"GS-2R"
		#else
			#ifdef _USE_SIP_SHIP_DEVICE_
				#define DEVICE_INFO					"SIP-SHIP"
			#else
				#define DEVICE_INFO					"SIP-100"
			#endif
		#endif
	#endif
#endif


void host_init(void);

void host_timer_tick(uint32_t us);

void host_reset(void);

#endif // _HOST_H_
