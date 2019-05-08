#include "mhscpu_crc.h"

#define XOR_OUT_SEL_SET			((uint32_t)0x10)
#define XOR_OUT_SEL_RESET		(~(uint32_t)0x10)

#define REV_OUT_SEL_SET			((uint32_t)0x08)
#define REV_OUT_SEL_RESET		(~(uint32_t)0x08)

#define REV_IN_SEL_SET			((uint32_t)0x04)
#define REV_IN_SEL_RESET		(~(uint32_t)0x04)

#define TYPE_SEL_SET		    ((uint32_t)0x02)
#define TYPE_SEL_RESET			(~(uint32_t)0x02)

#define POLY_SEL_SET		    ((uint32_t)0x01)
#define POLY_SEL_RESET			(~(uint32_t)0x01)

#define CRC_INIT_VALUE_0		((uint32_t)0)
#define CRC16_INIT_VALUE_FF	((uint32_t)0xffff)
#define CRC32_INIT_VALUE_FF	((uint32_t)0xffffffff)

uint32_t CRC_CalcBlockCRC(uint32_t CRC_type, uint8_t * p, uint32_t len)
{
	uint32_t crcVal = 0;
	switch(CRC_type)
	{
		case CRC_16:
			CRC->CRC_CSR =  REV_OUT_SEL_SET | REV_IN_SEL_SET;
			CRC->CRC_INI = CRC_INIT_VALUE_0;
			break;
		case CRC_16_Modbus:
			CRC->CRC_CSR = REV_OUT_SEL_SET | REV_IN_SEL_SET;
			CRC->CRC_INI = CRC16_INIT_VALUE_FF;
			break;
		case CRC_CCITT_0xffff:
			CRC->CRC_CSR = POLY_SEL_SET;
			CRC->CRC_INI = CRC16_INIT_VALUE_FF;
			break;
		case CRC_CCITT_XModem:
			CRC->CRC_CSR = POLY_SEL_SET;
			CRC->CRC_INI = CRC_INIT_VALUE_0;
			break;
		case CRC_32:
			CRC->CRC_CSR = XOR_OUT_SEL_SET | REV_OUT_SEL_SET | REV_IN_SEL_SET | TYPE_SEL_SET;
			CRC->CRC_INI = CRC32_INIT_VALUE_FF;
			break;
		default:
			break;
	}
	
	while(len != 0)
	{
		CRC->CRC_DATA.DIN = *p++;
		len--;
	}
	crcVal = CRC->CRC_DATA.DOUT;
	
	if(CRC_type != CRC_32)
		crcVal &= CRC16_INIT_VALUE_FF;
	
	return crcVal;
	
}


uint32_t CRC_Calc(CRC_ConfigTypeDef *CRC_Config, uint8_t *p, uint32_t len)
{
	uint32_t crcVal;
	
	CRC->CRC_CSR = 0;
	
	switch((uint32_t)(CRC_Config->CRC_Poly))
	{
		case (uint32_t)CRC_Poly_16_1021:
			CRC->CRC_CSR = POLY_SEL_SET;
			break;
		case (uint32_t)CRC_Poly_16_8005:
			CRC->CRC_CSR &= ~(POLY_SEL_SET);
			break;
		case (uint32_t)CRC_Poly_32_04C11DB7:
			CRC->CRC_CSR = TYPE_SEL_SET;
			break;
	}
	
	if(CRC_PolyMode_Reversed == CRC_Config->CRC_PolyMode)
	{
		CRC->CRC_CSR |= REV_OUT_SEL_SET | REV_IN_SEL_SET;
	}
	else
	{
		CRC->CRC_CSR &= ~(REV_OUT_SEL_SET | REV_IN_SEL_SET);
	}
	
	if(CRC_Poly_16_1021 == CRC_Config->CRC_Poly || CRC_Poly_16_8005 == CRC_Config->CRC_Poly)
	{
		CRC->CRC_INI = (uint16_t)CRC_Config->CRC_Init_Value;
	}
	else if(CRC_Poly_32_04C11DB7 == CRC_Config->CRC_Poly)
	{
		CRC->CRC_INI = (uint32_t)CRC_Config->CRC_Init_Value;
	}
	

	while(len != 0)
	{
		CRC->CRC_DATA.DIN = *p++;
		len--;
	}
	crcVal = CRC->CRC_DATA.DOUT;
	
	
	if(CRC_Poly_16_1021 == CRC_Config->CRC_Poly || CRC_Poly_16_8005 == CRC_Config->CRC_Poly)
	{
		crcVal ^= (uint16_t)CRC_Config->CRC_Xor_Value;
		crcVal &= CRC16_INIT_VALUE_FF;
	}
	else if(CRC_Poly_32_04C11DB7 == CRC_Config->CRC_Poly)
	{
		crcVal ^= (uint32_t)CRC_Config->CRC_Xor_Value;
	}

	
	
	return crcVal;
	
}
