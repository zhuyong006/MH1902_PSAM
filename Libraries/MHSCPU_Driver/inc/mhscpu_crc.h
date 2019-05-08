#ifndef __MHSCPU_CRC_H
#define __MHSCPU_CRC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "mhscpu.h"


#define CRC_16								((uint32_t)0x01)
#define CRC_16_Modbus						((uint32_t)0x02)
#define CRC_CCITT_0xffff					((uint32_t)0x03)
#define CRC_CCITT_XModem					((uint32_t)0x04)
#define CRC_32								((uint32_t)0x05)


uint32_t CRC_CalcBlockCRC(uint32_t CRC_type, uint8_t *p, uint32_t len);


typedef enum
{
	CRC_Poly_16_1021 = 0x01,
	CRC_Poly_16_8005 = 0x02,
	CRC_Poly_32_04C11DB7 = 0x03
}CRC_Poly_TypeDef;

typedef enum
{
	CRC_PolyMode_Normal = 0x01,
	CRC_PolyMode_Reversed = 0x02,
}CRC_PolyMode_TypeDef;

typedef struct
{
	CRC_Poly_TypeDef CRC_Poly;
	CRC_PolyMode_TypeDef CRC_PolyMode;
	uint32_t CRC_Init_Value;
	uint32_t CRC_Xor_Value;
}CRC_ConfigTypeDef;

uint32_t CRC_Calc(CRC_ConfigTypeDef *CRC_Config, uint8_t *p, uint32_t len);
	 
	 
#ifdef __cplusplus
}
#endif

#endif
