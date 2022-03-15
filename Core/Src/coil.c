#include <stdio.h>
#include<stdint.h>


#include "coil.h"

uint16_t GetCoil_Index(uint16_t bit)
{
	uint16_t i = 0;

	while (bit >= i* 8)
	{
		i++;
	}

	return i - 1;
}

uint8_t ReverseBitOrder(uint8_t ucOriData)
{
	uint8_t ucData, ucCnt, ucTemp;

	for (ucCnt = 0, ucData = 0; ucCnt < 8; ucCnt++)
	{
		ucTemp = (ucOriData >> ucCnt) & 0x01;

		ucData = (ucData << 1) | ucTemp;
	}

	return ucData;
}

uint16_t DataLen(uint16_t adr , uint16_t len)
{
	int bitCnt = (adr + len -1 );
	uint16_t byte = 0;


	if (bitCnt < 0)
	{
		bitCnt * -1;
	}

	while (bitCnt >= (byte * 8))
	{
		byte++;
	}

	return byte;
}

void GetCoilData(uint8_t* txBuf ,uint8_t *coilTable, uint16_t nthBit)
{
	uint8_t* txBufPtr = txBuf[TxData];
	uint8_t* headPtr = &coilTable[GetCoil_Index(nthBit)];
	uint8_t* nextPtr = headPtr + 1;
	uint8_t cnt =  txBuf[TxLen];//DataLen(MB_Handle taget.adr , MB_Handle taget.len);

	uint8_t	temp = 0;
	uint8_t i = 0;



	printf("txBufPtr: %d\n",* txBufPtr);
	while (i != cnt)
	{

		temp = *headPtr << nthBit | *nextPtr >> (8 - nthBit);
		*txBufPtr = ReverseBitOrder(temp);

		txBufPtr++;
		headPtr++;
		nextPtr++;
		i++;
	}


}

