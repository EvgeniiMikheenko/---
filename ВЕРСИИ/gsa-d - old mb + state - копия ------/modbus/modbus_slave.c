//=============================================================================
//							modbus_slave.c
//=============================================================================

#include "modbus_slave.h"
#include <stm32f10x_tim.h>

//------------------------------------------------------------------------------
// Private

//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// ������� 					: mb_slave_init
// �������� 				: ������������� ������ modbus-slave ���������� 
// ������� ��������� 		: ��������� �� ��������� MbSlaveParam_t
// �������� ��������� 		: -
// ������������ ��������	: -
//------------------------------------------------------------------------------
void mb_slave_init(lpMbSlaveParam_t lpParam) {
	if(lpParam == NULL)
		return;
	
	uint32_t baudrate = mb_slave_convert_baudrate((uint16_t)lpParam->serialBaudrate);
	
	if(lpParam->lpUartInit != NULL) {
		// ������������� UART
		(*lpParam->lpUartInit)(baudrate, lpParam->stopBits);
	}
	
	// ������������ ������� ��������� ���������� modbus
	lpParam->timeout = (((4 * 10 * 1000000L) / baudrate));
	
	lpParam->Flags.value = 0;
	lpParam->Flags.isRxDataMain = 1;
	lpParam->hostTimeLeft = 0;
	lpParam->timeLeft = 0;
	lpParam->rxCount = 0;
	lpParam->txCount = 0;
	lpParam->errorCount = 0;
	
	mb_slave_stop_tx(lpParam);
}

//------------------------------------------------------------------------------
// ������� 					: mb_slave_rx_func
// �������� 				: ������� ������ ���������� ����� ��� modbus-slave
//							: ����������, ��� ������� ���������� �������� � 
//							: ���������� �� ������ ����� USART
// ������� ��������� 		: lpParam 	- ��������� �� ��������� ������ ����������
//							: data 		- �������� ����
//							: frameError, overflowError - ����� ������ ������
// �������� ��������� 		: -
// ������������ ��������	: -
//------------------------------------------------------------------------------
void mb_slave_rx_func(lpMbSlaveParam_t lpParam, uint8_t data, bool frameError, bool overflowError) {
	if(lpParam == NULL)
		return;
	
	if(lpParam->Flags.isParsedStart) {
		// ������� ����� ��� �� ���������
		lpParam->timeLeft = 0;
		return;
	}
	
	if(frameError || overflowError) {
		// ������ ����� - ���������� ���� �����
		lpParam->rxCount = 0;
		// ���������� ���� ���������� � ������
		lpParam->Flags.isRxDataMain = 1;
		return;
	}
	
	if(lpParam->rxCount >= lpParam->rxBufSize) {
		// ������������ ������
		lpParam->rxCount = 0;
		lpParam->Flags.isRxDataMain = 1;
		return;
	}
	
	// ���������� �����
	lpParam->timeLeft = 0;
	if(!lpParam->Flags.isRxDataMain) {
		return;
	}
	
	if(lpParam->rxCount == 0) {
		#ifdef STOP_TIMER_ENABLE
				Timer_start();
		#endif
		
		
		
		
		// ������ ������ ���� ������ - ����� ����������
		// ��� ��������� ����� ��������� �����
		// ����� = 0 - ����������������� ������, ������� ��������� �� ����
		if(!((data == lpParam->address) || (data == 0))) {
			// ����� �� ������
			lpParam->Flags.isRxDataMain = 1;
			lpParam->Flags.enableTimer = 1;
			return;
		}
	}
	
	// ���������� ���� �������� ����� � ������
	lpParam->Flags.isHostTimeout = 0;
	// ��������� ����� ��������� ����� ��������� ����������
	lpParam->hostTimeLeft = 0;
	// �������� ������
	lpParam->rxBuf[lpParam->rxCount] = data;
	lpParam->rxCount++;
	// ���������� ���� ���������� ������ �������
	lpParam->Flags.enableTimer = 1;
	lpParam->timeLeft = 0;
	
	if(lpParam->lpSetLed != NULL) 
		(*lpParam->lpSetLed)(true);
	
	// ��� ����������������� CRC16 - ������������ ��������� �������� CRC ������� ��������� �����,
	// , ����� �� ������� CRC ����� ������ ����� ��������� ������
	lpParam->crcInfo.counter = lpParam->rxCount;
	lpParam->crcInfo.crc = CalcCrc16Next(lpParam->crcInfo.crc, data, lpParam->crcInfo.counter == 1);
}

//------------------------------------------------------------------------------
// ������� 					: mb_slave_stop_tx
// �������� 				: ������� ���������� �������� ������ ������
// ������� ��������� 		: lpParam - ��������� �� ��������� ������ ����������
// �������� ��������� 		:
// ������������ ��������	:
//------------------------------------------------------------------------------
void mb_slave_stop_tx(lpMbSlaveParam_t lpParam) {
	if(lpParam == NULL)
		return;
	
	lpParam->rxCount = 0;
	lpParam->Flags.enableParseRxData = 0;
	lpParam->Flags.isParsedStart = 0;
	
	if(lpParam->lpSetTxEn != NULL) {
		(*lpParam->lpSetTxEn)(false);
	}
	
	if(lpParam->lpSetLed != NULL) {
		(*lpParam->lpSetLed)(false);
	}
}

//------------------------------------------------------------------------------
// ������� 					: mb_slave_send_error
// �������� 				: ������� �������� ������ ������
// ������� ��������� 		: lpParam 		- ��������� �� ��������� ������ ����������
//							: func 			- ����� �������, � ������� �������� ������
//							: errorCode 	- ��� ������
// �������� ��������� 		:
// ������������ ��������	:
//------------------------------------------------------------------------------
void mb_slave_send_error(lpMbSlaveParam_t lpParam, uint8_t func, uint8_t errorCode) {
	if(lpParam == NULL)
		return;
	
	uint32_t size = 5;
	//lpModbusPacket_t lpPacket = (lpModbusPacket_t)lpParam->txBuf;
	lpParam->txBuf[0] = lpParam->address;
	lpParam->txBuf[1] = func | 0x80;
	lpParam->txBuf[2] = errorCode;
	WriteCrc16(lpParam->txBuf, size);
	mb_slave_start_tx(lpParam, size);
	
	lpParam->errorCount++;
}

//------------------------------------------------------------------------------
// ������� 					: mb_slave_start_tx
// �������� 				: ������� ������ ����������� �������� ������ ������
// ������� ��������� 		: lpParam 		- ��������� �� ��������� ������ ����������
//							: size			- ���������� ���� ��� ��������
// �������� ��������� 		:
// ������������ ��������	:
//------------------------------------------------------------------------------
void mb_slave_start_tx(lpMbSlaveParam_t lpParam, uint32_t size) {
	
	#ifdef STOP_TIMER_ENABLE
		Timer_stop();
	#endif
	
	
	if(lpParam == NULL)
		return;
	
	if(lpParam->lpSetTxEn != NULL)
		(*lpParam->lpSetTxEn)(true);
	
	if(lpParam->lpSetLed != NULL)
		(*lpParam->lpSetLed)(true);
	
	lpParam->txCount = size;
	lpParam->txIndex = 1;
	
	if(lpParam->lpUartSend != NULL)
		(*lpParam->lpUartSend)(lpParam->txBuf[0]);
	
}

//------------------------------------------------------------------------------
// ������� 					: mb_slave_parce_packet
// �������� 				: ������� ��������� ��������� ������
// ������� ��������� 		: lpParam 		- ��������� �� ��������� ������ ����������
// �������� ��������� 		:
// ������������ ��������	:
//------------------------------------------------------------------------------
void mb_slave_parce_packet(lpMbSlaveParam_t lpParam) {
	if(lpParam == NULL)
		return;
	
	if(lpParam->rxBuf == NULL)
		return;
	
	if(lpParam->lpRegisters == NULL)
		return;
	
	//lpModbusPacket_t lpPacket, lpSendPacket;
	uint16_t size, startAddr, count, endAddr, reg;
	//uint8_t addr;
	bool result = false;
	uint32_t tmp, byteCount;
	uint8_t *lpRxBuf;
	uint8_t *lpTxBuf;
	MbErrorCode_t errCode;
	bool updateEepromEn;
	
	//__istate_t intr = __get_interrupt_state();
	
	//__disable_interrupt();
	// ��������� ���� ������ ��������� ������
	if(lpParam->Flags.isParsedStart == 1) {
		// ����� ��� ��������������
		//__set_interrupt_state(intr);
		return;
	}
	// ���������� ���� ������ ��������� ������
	lpParam->Flags.isParsedStart = 1;
	
	//__set_interrupt_state(intr);
	
	if(lpParam->rxCount <= 4) {
		// ������ ������ �� ����� ���� <= 4
		mb_slave_stop_tx(lpParam);
		return;
	}
	
	if(lpParam->crcInfo.crc != 0) {
		// ������ CRC
		mb_slave_stop_tx(lpParam);
		return;
	}
	
	//lpPacket = (lpModbusPacket_t)lpParam->rxBuf;
	//lpSendPacket = (lpModbusPacket_t)lpParam->txBuf;
	
	if(!((lpParam->rxBuf[0] == lpParam->address) || (lpParam->rxBuf[0] == 0))) {
		// ����� ��������
		mb_slave_stop_tx(lpParam);
		return;
	}
	
	switch(lpParam->rxBuf[1]) {
//		case MODBUS_CMD_READ_COILS: 			// = 0x01 // ������ ������� �������
//			
//			break;
//		case MODBUS_CMD_READ_DINPUTS: 			// = 0x02 // ������ ��������� ���������� ������
//			
//			break;
		case MODBUS_CMD_READ_HOLDING_REGS: 		// = 0x03 // ������ ���������� ���������
			startAddr = (lpParam->rxBuf[2] << 8) | lpParam->rxBuf[3];//lpPacket->RdHoldingRegs.startAddr;
			count = (lpParam->rxBuf[4] << 8) | lpParam->rxBuf[5];//lpPacket->RdHoldingRegs.count;
			endAddr = startAddr + count;
			
			lpParam->Flags.isPasswordValid = 0;
			// ��������� ��������� �����
			if(/* (startAddr >= 0) && */ (startAddr < lpParam->registersCount)) {
				// ��������� ���������� ���������
				if((count == 0) || (count > MODBUS_REGS_IOCOUNT_MAX)) {
					mb_slave_send_error(lpParam, lpParam->rxBuf[1], ErrInvalidDataValue);
					break;
				}
				
				// ��������� ����� �� ������� ����������� ��������� �������
				if(endAddr > lpParam->registersCount) {
					mb_slave_send_error(lpParam, lpParam->rxBuf[1], ErrInvalidDataAddress);
					break;
				}
				
				lpParam->txCount = 3 + (count << 1) + 2;
				lpParam->txBuf[0] = lpParam->address;
				lpParam->txBuf[1] = lpParam->rxBuf[1];
				lpParam->txBuf[2] = (uint8_t)count << 1;
				
				for(uint32_t i = startAddr, j = 3; i < endAddr; i++, j += 2) {
					reg = lpParam->lpRegisters[i];
					lpParam->txBuf[j] = (uint8_t)(reg >> 8);
					lpParam->txBuf[j + 1] = (uint8_t)(reg);
				}
			}
			else {
				if(lpParam->lpGetDeviceInfo == NULL) {
					mb_slave_send_error(lpParam, lpParam->rxBuf[1], ErrInvalidFunctionCode);
					break;
				}
				
				(*lpParam->lpGetDeviceInfo)(&lpParam->txBuf[3], lpParam->txBufSize, &byteCount);
				if(byteCount == 0){
					mb_slave_send_error(lpParam, lpParam->rxBuf[1], ErrInvalidFunctionCode);
					break;
				}
				
				byteCount /= 2;
				
				if((startAddr == lpParam->deviceInfoSizeRegNum)) {
					if(count != 1) {
						mb_slave_send_error(lpParam, lpParam->rxBuf[1], ErrInvalidDataValue);
						break;
					}
					
					lpParam->txCount = 3 + (count << 1) + 2;
					lpParam->txBuf[0] = lpParam->rxBuf[0]; // GetDeviceAddr();
					lpParam->txBuf[1] = lpParam->rxBuf[1];
					lpParam->txBuf[2] = (uint8_t)count << 1;
					lpParam->txBuf[3] = (uint8_t)byteCount >> 8;
					lpParam->txBuf[4] = (uint8_t)(byteCount);
				}
				else if(startAddr == lpParam->deviceInfoRegNum) {
					
					if(count != byteCount){
						mb_slave_send_error(lpParam, lpParam->rxBuf[1], ErrInvalidDataValue);
						break;
					}
					
					size = 3 + byteCount;
					if((size + 3) >= MOBUS_MAX_PACKET_SIZE) {
						mb_slave_send_error(lpParam, lpParam->rxBuf[1], ErrInvalidFunctionCode);
						break;
					}
					lpParam->txBuf[0] = lpParam->rxBuf[0]; // GetDeviceAddr();
					lpParam->txBuf[1] = lpParam->rxBuf[1];
					lpParam->txBuf[2] = (uint8_t)count << 1;
					lpParam->txCount = 3 + (count << 1) + 2;
				}
				else {
					mb_slave_send_error(lpParam, lpParam->rxBuf[1], ErrInvalidDataAddress);
					break;
				}
			}
			
			size = lpParam->txCount;
			WriteCrc16(lpParam->txBuf, size);
			mb_slave_start_tx(lpParam, size);
			
			break;
//		case MODBUS_CMD_READ_INPUT_REGS: 		// = 0x04 // ������ ���������� ������� ���������
//			
//			break;
//		case MODBUS_CMD_WRITE_SINGLE_COIL: 		// = 0x05 // ��������� ���������� ������ � ON ��� OFF
//			
//			break;
		case MODBUS_CMD_WRITE_SINGLE_REG: 		// = 0x06 // ������ � ��������� �������
			startAddr = (lpParam->rxBuf[2] << 8) | lpParam->rxBuf[3];
			reg = (lpParam->rxBuf[4] << 8) | lpParam->rxBuf[5];
		
			if(startAddr >= lpParam->registersCount) {
				if(lpParam->Flags.isPasswordValid == 0) {
					mb_slave_send_error(lpParam, lpParam->rxBuf[1], ErrInvalidDataAddress);
					break;
				}
			}
			
			lpParam->Flags.isPasswordValid = 0;
			
			if((lpParam->regIndexes.regAddrDeviceAddress > 0) && (startAddr == lpParam->regIndexes.regAddrDeviceAddress)) {
				
				if(lpParam->Flags.isConfigChangeLock) {
					mb_slave_send_error(lpParam, lpParam->rxBuf[1], ErrInvalidDataAddress);
					break;
				}
				
				// ������ � ������� ������ ����������
				if((reg == 0) || (reg > MODBUS_DEVICE_ADDRESS_MAX)) {
					mb_slave_send_error(lpParam, lpParam->rxBuf[1], ErrInvalidDataValue);
					break;
				}
				
				lpParam->lpRegisters[startAddr] = reg;
				lpParam->address = (uint8_t)reg;
				result = true;
				lpParam->Flags.eepromNeedUpdate = 1;
				
			}
			else if((lpParam->regIndexes.regAddrBaudrate > 0) && (startAddr == lpParam->regIndexes.regAddrBaudrate)) {
				// ����� �������� �����
				if(lpParam->Flags.isConfigChangeLock) {
					mb_slave_send_error(lpParam, lpParam->rxBuf[1], ErrInvalidDataAddress);
					break;
				}
				tmp = mb_slave_convert_baudrate(reg);
				if(tmp == 0) {
					mb_slave_send_error(lpParam, lpParam->rxBuf[1], ErrInvalidDataValue);
					break;
				}
				
				result = false;
				lpParam->lpRegisters[startAddr] = reg;
				lpParam->Flags.eepromNeedUpdate = 1;
				
				if(lpParam->lpUartInit != NULL)
					(*lpParam->lpUartInit)(tmp, lpParam->stopBits);
				
				break;
			}
			else if((lpParam->regIndexes.regAddrHostTimeout > 0) && (startAddr == lpParam->regIndexes.regAddrHostTimeout)) {
				// ������ � ������� �������� ����� � ������
				if(lpParam->Flags.isConfigChangeLock) {
					mb_slave_send_error(lpParam, lpParam->rxBuf[1], ErrInvalidDataAddress);
					break;
				}
				
				lpParam->hostTimeOut = reg;
				lpParam->lpRegisters[startAddr] = reg;
				lpParam->Flags.eepromNeedUpdate = 1;
				result = true;
			}
			else { // ������ � ��������� ��������
				if(lpParam->lpGetRegWriteAccess != NULL) {
					if(!(*lpParam->lpGetRegWriteAccess)(startAddr)) {
						mb_slave_send_error(lpParam, lpParam->rxBuf[1], ErrInvalidDataAddress);
						break;
					}
				}
				
				lpParam->lpRegisters[startAddr] = reg;
				result = true;
			}
			
			if(!result)
				break;
			
			lpRxBuf = lpParam->rxBuf;
			lpTxBuf = lpParam->txBuf;
			
			for(uint32_t i = 0; i < lpParam->rxCount; i++) {
				*lpTxBuf++ = *lpRxBuf++;
			}
			
			mb_slave_start_tx(lpParam, lpParam->rxCount);
			
			break;
//		case MODBUS_CMD_READ_DEVICE_STATUS: 	// = 0x07 // ������ ������� ����������
//			
//			break;
//		case MODBUS_CMD_WRITE_MULTI_COILS: 		// = 0x0F // ��������� ��������� ������� � ON ��� OFF
//			
//			break;
		case MODBUS_CMD_WRITE_MULTI_REGS: 		// = 0x10 // ������ � ��������� ���������
			startAddr = (lpParam->rxBuf[2] << 8) | lpParam->rxBuf[3];
			count = (lpParam->rxBuf[4] << 8) | lpParam->rxBuf[5];
			endAddr = startAddr + count;
			byteCount = lpParam->rxBuf[6];
		
			if(!(/* (count >= 0) && */ (count < MODBUS_REGS_IOCOUNT_MAX) && (byteCount == (count << 1)))) {
				mb_slave_send_error(lpParam, lpParam->rxBuf[1], ErrInvalidDataValue);
				break;
			}
			
			if(startAddr >= lpParam->registersCount) {
				if(startAddr == lpParam->passwordRegNum) {
					// ��������� ����� ������
					if(count != lpParam->passwordLen) {
						mb_slave_send_error(lpParam, lpParam->rxBuf[1], ErrInvalidDataAddress);
						break;
					}
					// ��������� ������
					lpParam->Flags.isPasswordValid = 1;
					for(int i = 0; i < count; i++) {
						if(lpParam->rxBuf[(7 + 1) + (i * 2)] == lpParam->password[i])
							continue;
						
						lpParam->Flags.isPasswordValid = 0;
						break;
					}
				}
				else {
					mb_slave_send_error(lpParam, lpParam->rxBuf[1], ErrInvalidDataAddress);
					break;
				}
			}
			else {
				
				if(lpParam->Flags.isPasswordValid == 0) {
					mb_slave_send_error(lpParam, lpParam->rxBuf[1], ErrInvalidFunctionCode);
					break;
				}
				
				if(endAddr > lpParam->registersCount) {
					mb_slave_send_error(lpParam, lpParam->rxBuf[1], ErrInvalidDataAddress);
					break;
				}
				updateEepromEn = false;
				errCode = NoError;
				
				for(uint32_t i = startAddr, j = 7; i < endAddr; i++, j += 2) {
					reg = (lpParam->rxBuf[j] << 8) | lpParam->rxBuf[j + 1];
					//lpParam->lpRegisters[i] = reg;
									
					if((lpParam->regIndexes.regAddrDeviceAddress > 0) && (i == lpParam->regIndexes.regAddrDeviceAddress)) {
					
						if(lpParam->Flags.isConfigChangeLock) {
							errCode = ErrInvalidDataAddress; //mb_slave_send_error(lpParam, lpPacket->func, ErrInvalidDataAddress);
							break;
						}
						
						// ������ � ������� ������ ����������
						if((reg == 0) || (reg > MODBUS_DEVICE_ADDRESS_MAX)) {
							errCode = ErrInvalidDataValue; //mb_slave_send_error(lpParam, lpPacket->func, ErrInvalidDataValue);
							break;
						}
						
						lpParam->lpRegisters[i] = reg;
						lpParam->address = (uint8_t)reg;
						updateEepromEn = true;
						continue;
					}
					
					if((lpParam->regIndexes.regAddrBaudrate > 0) && (i == lpParam->regIndexes.regAddrBaudrate)) {
						// ����� �������� �����
						if(lpParam->Flags.isConfigChangeLock) {
							errCode = ErrInvalidDataAddress; //mb_slave_send_error(lpParam, lpPacket->func, ErrInvalidDataAddress);
							break;
						}
						tmp = mb_slave_convert_baudrate(reg);
						if(tmp == 0) {
							errCode = ErrInvalidDataValue; //mb_slave_send_error(lpParam, lpPacket->func, ErrInvalidDataValue);
							break;
						}
						
						lpParam->lpRegisters[i] = reg;
						updateEepromEn = true;
						
						if(lpParam->lpUartInit != NULL)
							(*lpParam->lpUartInit)(tmp, lpParam->stopBits);
						
						continue;
					}
					
					if((lpParam->regIndexes.regAddrHostTimeout > 0) && (i == lpParam->regIndexes.regAddrHostTimeout)) {
						// ������ � ������� �������� ����� � ������
						if(lpParam->Flags.isConfigChangeLock) {
							errCode = ErrInvalidDataAddress; //mb_slave_send_error(lpParam, lpPacket->func, ErrInvalidDataAddress);
							break;
						}
						
						lpParam->hostTimeOut = reg;
						lpParam->lpRegisters[i] = reg;
						updateEepromEn = true;
						continue;
					}
					
					// ������ � ��������� ��������
					if(lpParam->lpGetRegWriteAccess != NULL) {
						if(!(*lpParam->lpGetRegWriteAccess)(i)) {
							//mb_slave_send_error(lpParam, lpParam->rxBuf[1], ErrInvalidDataAddress);
							//break;
							continue;
						}
					}
					lpParam->lpRegisters[i] = reg;
					result = true;
				}
				
				if(errCode != NoError) {
					mb_slave_send_error(lpParam, lpParam->rxBuf[1], errCode);
					break;
				}
				
				if(updateEepromEn)
					lpParam->Flags.eepromNeedUpdate = 1;
			}
//			lpParam->txBuf[0] = lpParam->address;
//			lpParam->txBuf[1] = lpParam->rxBuf[1];
//			lpSendPacket->WrMultRegs.startAddr = startAddr;
//			lpSendPacket->WrMultRegs.count = count;
			
			size = 1 /* addr */ + 1 /* func */ + 2 /* startAddr */ + 2 /* regsCount */ + 2 /* crc16 */ ;
			for(int i = 0; i < size - 2; i++) {
				lpParam->txBuf[i] = lpParam->rxBuf[i];
			}
			lpParam->txCount = size;
			WriteCrc16(lpParam->txBuf, size);
			mb_slave_start_tx(lpParam, size);
			
			break;
		default:
			mb_slave_send_error(lpParam, lpParam->rxBuf[1], ErrInvalidFunctionCode);
			lpParam->Flags.isPasswordValid = 0;
			break;
	}
}

//------------------------------------------------------------------------------
// ������� 					: 
// �������� 				: 
// ������� ��������� 		: 
// �������� ��������� 		:
// ������������ ��������	:
//------------------------------------------------------------------------------
uint32_t mb_slave_convert_baudrate(uint16_t regValue) {
	uint32_t result = 0;
	
	switch(regValue) {
		case bps9600:
			result = 9600;
			break;
		case bps14400:
			result = 14400;
			break;
		case bps19200:
			result = 19200;
			break;
		case bps38400:
			result = 38400;
			break;
		case bps57600:
			result = 57600;
			break;
		case bps115200:
			result = 115200;
			break;
		default:
			
			break;
	}
	return result;
}

uint16_t mb_baudrate_to_reg_value(uint32_t baudrate) {
	uint16_t result;
	
	switch(baudrate) {
		case 9600:
			result = (uint16_t)bps9600;
			break;
		case 14400:
			result = (uint16_t)bps14400;
			break;
		case 19200:
			result = (uint16_t)bps19200;
			break;
		case 38400:
			result = (uint16_t)bps38400;
			break;
		case 57600:
			result = (uint16_t)bps57600;
			break;
//		case 115200:
//			result = bps115200;
//			break;
		default:
			result = bps115200;
			break;
	}
	return result;
}

//------------------------------------------------------------------------------
// ������� 					: 
// �������� 				: 
// ������� ��������� 		: 
// �������� ��������� 		:
// ������������ ��������	:
//------------------------------------------------------------------------------
void mb_slave_timer_tick(lpMbSlaveParam_t lpParam, uint32_t us) {
	if(lpParam == NULL)
		return;
	
	static uint32_t time = 0;
	time += us;
	if(time >= 1000) {
		lpParam->hostTimeLeft++;
		time -= 1000;
	}
	
	if(lpParam->Flags.enableTimer == 0)
		return;
	
	if(lpParam->Flags.isParsedStart)
		return;
	
	lpParam->timeLeft += us;
	if(lpParam->timeLeft < lpParam->timeout)
		return;
	
	if(lpParam->lpSetLed != NULL)
		(*lpParam->lpSetLed)(false);
	
	lpParam->Flags.enableParseRxData = 1;
	lpParam->Flags.isRxDataMain = 1;
	mb_slave_parce_packet(lpParam);	
	lpParam->Flags.enableTimer = 0;
}





#ifdef  STOP_TIMER_ENABLE

void Timer_stop( void )
{
		
		TIM_Cmd(TIM4, DISABLE);
		//TIM_DeInit(TIM4);

};

void Timer_start( void )
{
		TIM_Cmd(TIM4, ENABLE);
};



#endif			//STOP_TIMER_ENABLE


