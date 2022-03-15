#ifndef	__COIL_H_
#define __COIL_H_

#include "main.h"
#include "Modbus.h"
#include "CoilReg_Table.h"


uint16_t GetCoil_Index(uint16_t bit);
uint8_t ReverseBitOrder(uint8_t ucOriData);
uint16_t DataLen(uint16_t adr , uint16_t len);
void GetCoilData(uint8_t* txBuf ,uint8_t *coilTable, uint16_t nthBit);

#endif
