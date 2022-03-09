/*
 * Modbus.c
 *
 *  Created on: Feb 17, 2022
 *      Author: KimGiun
 */

#include "Modbus.h"


volatile uint16_t RX_DataCnt = 0;
uint8_t MB_RxBuf[RXBUFLEN] = {0,};
uint8_t MB_TXBuf[TXBUFLEN] = {0,};
uint8_t RX_Data = 0;

MB_Handle packetHandle;
TABLE table;

void ModbusSlave()
{
	Rx_TimerStartStop();
	frameCplt();
	GetRX_Packet();
}

bool Search_ID(uint8_t taget)
{
	if(Table_SlaveID[taget] == FULL)
	{
		return TURE;
	}
	else
	{
		return FALSE;
	}
}

void TableInit()
{
	memset(table.slaveId, EMPTY , SLAVEIDLEN);
}
void packetInit()
{
	packetHandle.id = 0;
	packetHandle.FuncCode = 0;
	packetHandle.adr = 0;
	packetHandle.len = 0;
	memset(packetHandle.data,0,DATALEN);
	packetHandle.crc_high = 0;
	packetHandle.crc_low = 0;
	packetHandle.RX_Flag = OFF;
	packetHandle.TX_Flag = OFF;
}

void RX_Buf_Init()
{
	memset(MB_RxBuf,0,RXBUFLEN);
	RX_Data = 0;
	RX_DataCnt =0;
}

void frameCplt()
{
	if(RX_DataCnt == 8)
	{
		packetHandle.RX_Flag = ON;
	}
	else
	{
		packetHandle.RX_Flag = OFF;
	}
}

uint8_t GetRX_Packet()
{
	static uint8_t RX_State = GetID;

	switch(RX_State)
	{
		case GetID:
		{
			if( Search_ID(MB_RxBuf[SlaveID]) )
			{
				packetHandle.id = MB_RxBuf[SlaveID];
				RX_State = GetFunCode;
			}
			else
			{
				RX_Buf_Init();
				packetHandle.RX_Flag = OFF;
				RX_State = Ready;
			}
			break;
		}
		case GetFunCode:
		{
			packetHandle.FuncCode = MB_RxBuf[FuncCode];
			RX_State = GetAdr;
			break;
		}
		case GetAdr:
		{
			packetHandle.adr = (uint16_t)( MB_RxBuf[AdrHigh] | MB_RxBuf[AdrLow] );
			RX_State = Getlen;
			break;
		}
		case Getlen:
		{
			packetHandle.len = (uint16_t)( MB_RxBuf[LenHigh] | MB_RxBuf[LenLow] );
			RX_State = GetCrc;
			break;
		}
		case GetCrc:
		{
			packetHandle.crc_high = MB_RxBuf[CrcHigh];
			packetHandle.crc_low = MB_RxBuf[CrcLow];

			packetHandle.TX_Flag = ON;
			RX_Buf_Init();
			RX_State = Ready;
			break;
		}
	}
}

uint8_t GetTX_Packet()
{
	static uint8_t TX_State = GetID;

	switch(TX_State)
	{
		case GetID:
		{
			MB_TXBuf[SlaveID] = packetHandle.id;
			TX_State = GetFunCode;

			break;
		}

		case GetFunCode:
		{
			MB_TXBuf[FuncCode] = packetHandle.FuncCode;
			TX_State = Getlen;

			break;
		}

		case Getlen:
		{
			uint16_t getLen = packetHandle.len * 2;
			MB_TXBuf[TxLen] = getLen;
			break;
		}

		case GetData:
		{
			GetCoilRegData();
			break;
		}

		case GetCrc:
		{

			break;
		}

	}

}
void GetHandleData()//copy CoilRegData -> packHandle.data[]
{
	uint16_t *dataPtr = GetCoilRegTable();
	uint16_t headIdex = packetHandle.adr;
	uint16_t tailIdex = packetHandle.len;

	uint16_t len = headIdex - tailIdex;

	if(len < 0)
	{
		len = len * -1;
	}

	memcpy(packetHandle.data , dataPtr[headIdex] , len);

}

void GetCoilRegData()//copy packHandle.data[] -> MB_TXBuf[data]
{
	GetHandleData();

	int cnt = packetHandle.len * 2;
	uint8_t *bufPtr = &MB_TXBuf[3];
	uint16_t *packetPtr = packetHandle.data;

	while(cnt >= 0)
	{
		*bufPtr ++= *packetPtr >> 8;
		*bufPtr ++= *packetPtr;
		packetPtr++;
		cnt--;
	}

}

uint8_t GetCoilRegTable()
{
	uint8_t coilRegAdr = GetTableAdr();

	switch(coilRegAdr)
	{
		case Coil_1:
		{
			return CoilTable_1;
			break;
		}

		case Coil_10000:
		{
			//return CoilTable_1;
			break;
		}

		case Reg_30000:
		{
			return RegTable_30000;
			break;
		}

		case Reg_40000:
		{
			//return CoilTable_1;
			break;
		}
	}

}

uint8_t GetTableAdr()
{
	if(MB_TXBuf[FuncCode] == 1 || MB_TXBuf[FuncCode] == 5 || MB_TXBuf[FuncCode] == 15)
	{
		return Coil_1;
	}
	else if(MB_TXBuf[FuncCode] == 2)
	{
		return Coil_10000;
	}
	else if(MB_TXBuf[FuncCode] == 4)
	{
		return Reg_30000;
	}
	else if(MB_TXBuf[FuncCode] == 3 || MB_TXBuf[FuncCode] == 6 || MB_TXBuf[FuncCode] == 16)
	{
		return Reg_40000;
	}

}


//
//void showPacket(uint8_t *packet , uint16_t len)
//{
//	uint8_t buf[len];
//
//	memcpy(buf,packet,len);
//	RX_Buf_Init();
//
//	if(len < 8)// if intput packet RX.
//	{
//		printf("TX_Packet:");
//	}
//	else
//	{
//		printf("RX_Packet:");
//	}
//
//	for (int i = 0; i < len; i++)
//	{
//		if (buf[i] <= 0x0A)
//		{
//			printf("0%x ", buf[i]);
//		}
//		else
//		{
//			printf("%x ", buf[i]);
//		}
//	}
//	printf("\n");
//
//}

void GetCRC(uint8_t *packet)
{
	uint16_t crc  = crc16(packet, 6);
	uint8_t CRC_High = crc >> 8;
	uint8_t CRC_Low = crc;

	packet[CRC_HIGH_NIBBLE] = CRC_High;
	packet[CRC_LOW_NIBBLE] = CRC_Low;


}

//
//void getHexData(char* String, BYTE* returnBytes, uint8_t len)
//{
//	int i = 0;
//	BYTE data[256];
//	int j = 0;
//
//
//	char* test = String + i;
//
//	while (*(String + i) != '\0') {
//
//			BYTE ten = *(String + i);
//			BYTE one = *(String + i + 1);
//
// 			if (ten >= 'a' && ten <= 'f') {
//				ten = (((ten - 'a') + 10) * 16);
//			}
//			else if (ten >= '0' && ten <= '9') {
//				ten = ((ten - '0') * 16);
//			}
//
//			if (one >= 'a' && one <= 'f') {
//				one = ((one - 'a') + 10);
//			}
//			else if (one >= '0' && one <= '9') {
//				one = (one - '0');
//			}
//
//			data[j] = (BYTE)(ten + one);
//			j++;
//			i = i + 2;
//	}
//
//	memcpy(returnBytes, data, j);
//	memset(String, 0, len);
//
//}
//

//uint8_t FindData(uint8_t *packet)
//{
//	uint8_t tempBuff[RX_DataCnt];
//	memset(tempBuff,0,RX_DataCnt);
//
//	memcpy(tempBuff,packet,RX_DataCnt);
//
//	memset(packet,0,RX_DataCnt);
//
//	while(1)
//	{
//		if(tempBuff[slaveID_Index] != 0)
//		{
//			break;
//		}
//
//		slaveID_Index++;
//	}
//
//	for (int i = 0; i < RX_DataCnt ; i++)
//	{
//		packet[i] = tempBuff[slaveID_Index++];
//	}
//	slaveID_Index = 0;
//	RX_CrcFlag = 0;
//
//	return 1;
//
//}
//
//uint8_t CheckCRC()
//{
//	uint16_t tempBuf[RX_DataCnt];
//
//	memcpy(tempBuf,MB_RxBuf ,RX_DataCnt);
//
//	uint16_t tempCRC = crc16(tempBuf, RX_DataCnt - 2);
//	uint16_t RX_CRC = ( MB_RxBuf[RX_DataCnt -2] << 8) | ( MB_RxBuf[RX_DataCnt-1]) ;
//
//	if(tempCRC == RX_CRC)
//	{
//		return 1;
//	}
//
//	return 0;
//}
//



