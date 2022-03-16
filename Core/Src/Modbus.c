/*
 * Modbus.c
 *
 *  Created on: Feb 17, 2022
 *      Author: KimGiun
 */

#include "Modbus.h"
#include "coil.h"

volatile uint16_t RX_DataCnt = 0;
uint8_t MB_RxBuf[RXBUFLEN] = {0,};
uint8_t MB_TXBuf[TXBUFLEN] = {0,};
uint8_t RX_Data = 0;

MB_Handle packetHandle;
TABLE table;
extern UART_HandleTypeDef huart2;

uint8_t Search_ID(uint8_t taget)
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

void MB_Slave()
{
	Rx_TimerStartStop();
	frameCplt();

	uint8_t mode = SelectMode(packetHandle.RX_Flag ,packetHandle.TX_Flag);

	switch(mode)
	{
		case RXMODE:
		{
			GetRX_Packet();
			break;
		}

		case TXMODE:
		{
			GetTX_Packet();
			break;
		}

	}

}

uint8_t SelectMode(uint8_t RX_Flag , uint8_t TX_Flag)
{
	if(RX_Flag == ON)
	{
		return RXMODE;
	}

	if(TX_Flag == ON)
	{
		return TXMODE;
	}
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
	RX_DataCnt = 0;
}

void frameCplt()
{
	if(RX_DataCnt == 8)
	{
		SetHandleFlag(&packetHandle.RX_Flag , ON);

	}
	else if(RX_DataCnt == 0)
	{
		SetHandleFlag(&packetHandle.RX_Flag , OFF);
	}

}

bool SetHandleFlag(uint8_t *flag , uint8_t onOff)
{

	uint8_t *RxAdr = &packetHandle.RX_Flag;
	uint8_t *TxAdr = &packetHandle.TX_Flag;
	uint8_t flagAdr = 0;
	uint8_t state = onOff;

	if(flag == RxAdr)
	{
		flagAdr = RXADR;
	}

	else if(flag == TxAdr)
	{
		flagAdr = TXADR;
	}

	switch(flagAdr)
	{
		case RXADR:
		{
			if(state == ON)
			{
				*flag = ON;
				return TURE;
				break;
			}
			else if(state == OFF)
			{
				*flag = OFF;
				return FALSE;
				break;
			}
		}

		case TXADR:
		{
			if(state == ON)
			{
				*flag = ON;
				return TURE;
				break;
			}
			else if(state == OFF)
			{
				*flag = OFF;
				return FALSE;
				break;
			}
		}
	}
}

uint8_t GetRX_Packet()
{
	static uint8_t RX_State = Ready;

	switch(RX_State)
	{
		case Ready:
		{
			if( Search_ID(MB_RxBuf[SlaveID]) == TURE  )
			{
				RX_State = GetID;
			}
			else
			{
				RX_Buf_Init();
				packetHandle.RX_Flag = OFF;
				RX_State = Ready;
			}
			break;
		}
		case GetID:
		{

			packetHandle.id = MB_RxBuf[SlaveID];
			RX_State = GetFunCode;
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

			SetHandleFlag(&packetHandle.TX_Flag , ON);
			SetHandleFlag(&packetHandle.RX_Flag , OFF);
			RX_Buf_Init();
printf("TX-%02x %02x %02x %02x %02x %02x\n",
		packetHandle.id, packetHandle.FuncCode, packetHandle.adr, packetHandle.len , packetHandle.crc_high, packetHandle.crc_low);
			RX_State = Ready;
			return RX_OK;
			break;
		}
	}
}

void GetTX_Packet()
{

	static uint8_t TX_State = Ready;
	static uint8_t DataType = Null;

	switch(TX_State)
	{
		case Ready:
		{
			TX_State = GetID;
			break;
		}

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
			uint16_t tablePtr = GetTableAdr();//want Coil?? Reg??

			if(tablePtr == Coil_1 || tablePtr == Coil_10000)//GetCoilLen
			{
				MB_TXBuf[TxLen] = DataLen(packetHandle.adr, packetHandle.len);
				TX_State = GetData;
				DataType = Coil;
			}
			else
			{
				MB_TXBuf[TxLen] = packetHandle.len * 2;
				TX_State = GetData;
				DataType = Reg;
			}

			break;
		}

		case GetData:
		{
			switch(DataType)
			{
				case Coil:
				{
					GetCoilData(MB_TXBuf, GetCoilRegTable(), packetHandle.adr);
					TX_State = GetCrc;
					break;
				}

				case Reg:
				{
					GetRegData();
					TX_State = GetCrc;
					break;
				}
			}

			break;
		}

		case GetCrc:
		{	uint16_t TxCrc = crc16(MB_TXBuf,MB_TXBuf[TxLen] + 3 );

			uint8_t crcHighIndex = MB_TXBuf[TxLen] + 3;
			uint8_t crcLowIndex = MB_TXBuf[TxLen] + 4;

			MB_TXBuf[crcHighIndex] = TxCrc >> 8;
			MB_TXBuf[crcLowIndex] = TxCrc;
showPacket(MB_TXBuf, MB_TXBuf[TxLen]+5);
			SetHandleFlag(&packetHandle.TX_Flag , OFF);
			HAL_UART_Transmit(&huart2, MB_TXBuf, MB_TXBuf[TxLen]+5, 0xffff);
			memset(MB_TXBuf,0,1024);

			TX_State = Ready;
			break;
		}

	}

}


void GetRegData()//copy packHandle.data[] -> MB_TXBuf[data]
{
		GetHandleData();

		uint8_t *bufPtr = &MB_TXBuf[3];
		uint16_t *packetPtr = packetHandle.data;

		int cnt = packetHandle.adr - packetHandle.len ;
		if(cnt < 0)
		{
			cnt = cnt * -1;
		}

		while(cnt >= 0)
		{
			*bufPtr ++= *packetPtr >> 8;
			*bufPtr ++= *packetPtr;
			packetPtr++;
			cnt--;
		}
}

void GetHandleData()//copy CoilRegData -> packHandle.data[]
{
	uint16_t *dataPtr = GetCoilRegTable();
	uint16_t headIndex = packetHandle.adr;
	uint16_t tailIndex = packetHandle.len;

	int len = headIndex - tailIndex;

	if(len < 0)
	{
		len = len * -1;
	}

	memcpy(packetHandle.data , &dataPtr[headIndex] , len * 2);// why len * 2 : sizeof = 1byte
}

uint16_t* GetCoilRegTable()
{
	uint16_t coilRegAdr = GetTableAdr();

	switch(coilRegAdr)
	{
		case Coil_1:
		{
			return &CoilTable_1;
			break;
		}

		case Coil_10000:
		{
			return &CoilTable_10000;
			break;
		}

		case Reg_30000:
		{
			return &RegTable_30000;
			break;
		}

		case Reg_40000:
		{
			return &RegTable_40000;
			break;
		}
	}

}

uint16_t GetTableAdr()
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

void showPacket(uint8_t *packet , uint16_t len)
{
	uint8_t buf[len];

	memcpy(buf,packet,len);
	RX_Buf_Init();

	int mode = SelectMode(packetHandle.RX_Flag ,packetHandle.TX_Flag);

	switch(mode)
	{
		case RXMODE:
		{
			printf("RX_Packet:");
			break;
		}
		case TXMODE:
		{
			printf("TX_Packet:");
			break;
		}
	}

	for (int i = 0; i < len; i++)
	{
		if (buf[i] <= 0x0A)
		{
			printf("0%x ", buf[i]);
		}
		else
		{
			printf("%x ", buf[i]);
		}
	}
	printf("\n");

}

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



