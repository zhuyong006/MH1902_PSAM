#include "mhscpu_sensor.h"
#include "stdio.h"

/******************************************************************************/
/*                                                                            */
/*						SENSOR Control Unit Block			                  */
/*                                                                            */
/******************************************************************************/

/****************  Bit definition for SEN_EXT_CFG register  *******************/
#define SEN_EXT_CFG_EXTS_LEVEL						(0x0FFFUL)
#define SEN_EXT_CFG_FREQ							(BIT(13)|BIT(12))
#define SEN_EXT_CFG_FREQ_0							BIT(12)
#define SEN_EXT_CFG_FREQ_1							BIT(13)
#define SEN_EXT_CFG_EXTS_PROC						BIT(15)
#define SEN_EXT_CFG_GF								(BIT(17)|BIT(16))
#define SEN_EXT_CFG_GF_0							BIT(16)
#define SEN_EXT_CFG_GF_1							BIT(17)
#define SEN_EXT_CFG_GF_EN							BIT(18)
#define SEN_EXT_CFG_PUPU_EN							BIT(19)
#define SEN_EXT_CFG_PUPU_HOLD_TIME					(BIT(21)|BIT(20))
#define SEN_EXT_CFG_PUPU_HOLD_TIME_0				BIT(20)
#define SEN_EXT_CFG_PUPU_HOLD_TIME_1				BIT(21)
#define SEN_EXT_CFG_PUPU_DETECT_TIME				(BIT(23)|BIT(22))
#define SEN_EXT_CFG_PUPU_DETECT_TIME_0				BIT(22)
#define SEN_EXT_CFG_PUPU_DETECT_TIME_1				BIT(23)
#define SEN_EXT_CFG_PUPU_FREQ						BIT(24)
/****************  Bit definition for SEN_BRIDGE register  ********************/
#define SEN_BRIDGE_READ_START						BIT(0)
#define SEN_BRIDGE_READ_READY						BIT(1)
/*****************  Bit definition for SEN_SOFT_EN register  *******************/
#define SEN_SOFT_EN_SOFT_ATTACK_EN					BIT(0)
/****************  Bit definition for SEN_SOFT_ATTACK register  ********************/
#define SEN_SOFT_ATTACK_SOFT_ATTACK    				BIT(0)
/****************  Bit definition for SEN_SOFT_LOCK register  ********************/
#define SEN_SOFT_LOCK_SOFT_ATTACK_LOCK        		BIT(31)


#define SEN_ENABLE									((uint32_t)0xAA)
#define SEN_DISABLE									((uint32_t)0x55)

typedef enum
{
	SENSOR_EN_EXT0,
	SENSOR_EN_EXT1,
	SENSOR_EN_EXT2,
	SENSOR_EN_EXT3,
	SENSOR_EN_EXT4,
	SENSOR_EN_EXT5,
	SENSOR_EN_EXT6,
	SENSOR_EN_EXT7,
	SENSOR_EN_EXT8,
	SENSOR_EN_EXT9,
	SENSOR_EN_EXT10,
	SENSOR_EN_EXT11,
	SENSOR_EN_VH,
	SENSOR_EN_VL,
	SENSOR_EN_TH,
	SENSOR_EN_TL,
	SENSOR_EN_XTAL32,
	SENSOR_EN_MESH,
	SENSOR_EN_VOLGLITCH,
}SENSOR_EN_Def;

/**
  * @brief  
  * @param  
  * @retval None
  */
uint32_t SENSOR_EXTInit(SENSOR_EXTInitTypeDef *SENSOR_EXTInitStruct)
{
	volatile uint32_t tmpSensorExtType = 0;
	volatile uint32_t tmpSensorExtPort = 0;
	volatile uint32_t tmpSensorExtPortEn = 0;
	uint32_t i;
    
	if (SET == SENSOR_EXTIsRuning())
		return 3;
	
	//simple rule check,each pin can be used only one mode
	if ((SENSOR_EXTInitStruct->SENSOR_Port_StaticHighEffect | SENSOR_EXTInitStruct->SENSOR_Port_StaticLowEffect) &
		SENSOR_EXTInitStruct->SENSOR_Port_Dynamic)
	{
		return 1;
	}
	
	if (SENSOR_EXTInitStruct->SENSOR_Port_StaticHighEffect & SENSOR_EXTInitStruct->SENSOR_Port_StaticLowEffect)
	{
		return 2;
	}

	//Set SEN_EXT_TYPE register
	for(i = 0; i < 6; i++)
	{
		if(0 != (SENSOR_EXTInitStruct->SENSOR_Port_Dynamic & (0x03 << (i * 2))))
		{
			//clean ExtPort type
			tmpSensorExtType &= ~(0x03 << (i * 2));
			//set ExtPort type 1
			tmpSensorExtType |= (0x01 << (i * 2));
		}
	}
	tmpSensorExtType |= (SENSOR_EXTInitStruct->SENSOR_Port_PullUp << 12);
	
	SENSOR->SEN_EXT_TYPE = tmpSensorExtType;

	//Set SEN_EXT_CFG register
	//clean ExtPort Static Attack Effect Level
	SENSOR->SEN_EXT_CFG &= ~SEN_EXT_CFG_EXTS_LEVEL;
	//set ExtPort Static Attack Effect Level
	SENSOR->SEN_EXT_CFG &= ~SENSOR_EXTInitStruct->SENSOR_Port_StaticLowEffect;
	SENSOR->SEN_EXT_CFG |= SENSOR_EXTInitStruct->SENSOR_Port_StaticHighEffect;
	
	//clean Dynamci Sensor ExtPort Frequency
	SENSOR->SEN_EXT_CFG &= ~SEN_EXT_CFG_FREQ;
	//set Dynamci Sensor ExtPort Frequency
	SENSOR->SEN_EXT_CFG |= SENSOR_EXTInitStruct->SENSOR_DynamicFrequency << 12;

	//set Dyanmci Sensor Glitch
	SENSOR->SEN_EXT_CFG &= ~SEN_EXT_CFG_GF_EN;
	//clean ExtPort Glitch Frequency
	SENSOR->SEN_EXT_CFG &= ~SEN_EXT_CFG_GF;
	//set ExtPort Glitch Frequency
	SENSOR->SEN_EXT_CFG |= SENSOR_EXTInitStruct->SENSOR_GlitchFrequency << 16;
	//enable/disable Dyanmci Sensor Glitch
	if (DISABLE != SENSOR_EXTInitStruct->SENSOR_GlitchEnable)
	{
		SENSOR->SEN_EXT_CFG |= SEN_EXT_CFG_GF_EN;
	}

	//disable ExtPort PUPU resistance
	SENSOR->SEN_EXT_CFG &= ~SEN_EXT_CFG_PUPU_EN;
	//config freq/holdtime/detectime
	SENSOR->SEN_EXT_CFG &= ~SEN_EXT_CFG_PUPU_FREQ;
	SENSOR->SEN_EXT_CFG &= ~SEN_EXT_CFG_PUPU_DETECT_TIME;
	SENSOR->SEN_EXT_CFG &= ~SEN_EXT_CFG_PUPU_HOLD_TIME;
	SENSOR->SEN_EXT_CFG |= SENSOR_EXTInitStruct->SENSOR_PUPU_Frequency << 24;
	SENSOR->SEN_EXT_CFG |= SENSOR_EXTInitStruct->SENSOR_PUPU_DetectTime << 22;
	SENSOR->SEN_EXT_CFG |= SENSOR_EXTInitStruct->SENSOR_PUPU_HoldTime << 20;
	//enable/disable ExtPort PUPU resistance
	if (DISABLE != SENSOR_EXTInitStruct->SENSOR_PUPU_Enable)
	{
		SENSOR->SEN_EXT_CFG |= SEN_EXT_CFG_PUPU_EN;
	}

	//setting SEN_EN[x] register to effect the Sensor Port
	tmpSensorExtPort = SENSOR_EXTInitStruct->SENSOR_Port_StaticHighEffect |
					   SENSOR_EXTInitStruct->SENSOR_Port_StaticLowEffect |
					   SENSOR_EXTInitStruct->SENSOR_Port_Dynamic;
	
	//Enable or Disable Ext Sensors
	if (DISABLE != SENSOR_EXTInitStruct->SENSOR_Port_Enable)
		tmpSensorExtPortEn = SEN_ENABLE;
	else
		tmpSensorExtPortEn = SEN_DISABLE;
	
	if (tmpSensorExtPort & SENSOR_Port_S0)
		SENSOR->SEN_EN[SENSOR_EN_EXT0] = tmpSensorExtPortEn;
	else
		SENSOR->SEN_EN[SENSOR_EN_EXT0] = SEN_DISABLE;
	
	if (tmpSensorExtPort & SENSOR_Port_S1)
		SENSOR->SEN_EN[SENSOR_EN_EXT1] = tmpSensorExtPortEn;
	else
		SENSOR->SEN_EN[SENSOR_EN_EXT1] = SEN_DISABLE;
	
	if (tmpSensorExtPort & SENSOR_Port_S2)
		SENSOR->SEN_EN[SENSOR_EN_EXT2] = tmpSensorExtPortEn;
	else
		SENSOR->SEN_EN[SENSOR_EN_EXT2] = SEN_DISABLE;
	
	if (tmpSensorExtPort & SENSOR_Port_S3)
		SENSOR->SEN_EN[SENSOR_EN_EXT3] = tmpSensorExtPortEn;
	else
		SENSOR->SEN_EN[SENSOR_EN_EXT3] = SEN_DISABLE;
	
	if (tmpSensorExtPort & SENSOR_Port_S4)
		SENSOR->SEN_EN[SENSOR_EN_EXT4] = tmpSensorExtPortEn;
	else
		SENSOR->SEN_EN[SENSOR_EN_EXT4] = SEN_DISABLE;
	
	if (tmpSensorExtPort & SENSOR_Port_S5)
		SENSOR->SEN_EN[SENSOR_EN_EXT5] = tmpSensorExtPortEn;
	else
		SENSOR->SEN_EN[SENSOR_EN_EXT5] = SEN_DISABLE;
	
	if (tmpSensorExtPort & SENSOR_Port_S6)
		SENSOR->SEN_EN[SENSOR_EN_EXT6] = tmpSensorExtPortEn;
	else
		SENSOR->SEN_EN[SENSOR_EN_EXT6] = SEN_DISABLE;
	
	if (tmpSensorExtPort & SENSOR_Port_S7)
		SENSOR->SEN_EN[SENSOR_EN_EXT7] = tmpSensorExtPortEn;
	else
		SENSOR->SEN_EN[SENSOR_EN_EXT7] = SEN_DISABLE;
	
	if (tmpSensorExtPort & SENSOR_Port_S8)
		SENSOR->SEN_EN[SENSOR_EN_EXT8] = tmpSensorExtPortEn;
	else
		SENSOR->SEN_EN[SENSOR_EN_EXT8] = SEN_DISABLE;
	
	if (tmpSensorExtPort & SENSOR_Port_S9)
		SENSOR->SEN_EN[SENSOR_EN_EXT9] = tmpSensorExtPortEn;
	else
		SENSOR->SEN_EN[SENSOR_EN_EXT9] = SEN_DISABLE;
	
	if (tmpSensorExtPort & SENSOR_Port_S10)
		SENSOR->SEN_EN[SENSOR_EN_EXT10] = tmpSensorExtPortEn;
	else
		SENSOR->SEN_EN[SENSOR_EN_EXT10] = SEN_DISABLE;
	
	if (tmpSensorExtPort & SENSOR_Port_S11)
		SENSOR->SEN_EN[SENSOR_EN_EXT11] = tmpSensorExtPortEn;
	else
		SENSOR->SEN_EN[SENSOR_EN_EXT11] = SEN_DISABLE;	

	return 0;
}


uint32_t SENSOR_EXTPortCmd(uint32_t SENSOR_Port, FunctionalState NewState)
{
	volatile uint32_t tmpSensorExtPortEn = 0;
	
	if (SET == SENSOR_EXTIsRuning())
		return 3;
	
	if (DISABLE != NewState)
		tmpSensorExtPortEn = SEN_ENABLE;
	else
		tmpSensorExtPortEn = SEN_DISABLE;
	
	if (SENSOR_Port & SENSOR_Port_S0)
		SENSOR->SEN_EN[SENSOR_EN_EXT0] = tmpSensorExtPortEn;
	if (SENSOR_Port & SENSOR_Port_S1)
		SENSOR->SEN_EN[SENSOR_EN_EXT1] = tmpSensorExtPortEn;
	if (SENSOR_Port & SENSOR_Port_S2)
		SENSOR->SEN_EN[SENSOR_EN_EXT2] = tmpSensorExtPortEn;
	if (SENSOR_Port & SENSOR_Port_S3)
		SENSOR->SEN_EN[SENSOR_EN_EXT3] = tmpSensorExtPortEn;
	if (SENSOR_Port & SENSOR_Port_S4)
		SENSOR->SEN_EN[SENSOR_EN_EXT4] = tmpSensorExtPortEn;
	if (SENSOR_Port & SENSOR_Port_S5)
		SENSOR->SEN_EN[SENSOR_EN_EXT5] = tmpSensorExtPortEn;
	if (SENSOR_Port & SENSOR_Port_S6)
		SENSOR->SEN_EN[SENSOR_EN_EXT6] = tmpSensorExtPortEn;
	if (SENSOR_Port & SENSOR_Port_S7)
		SENSOR->SEN_EN[SENSOR_EN_EXT7] = tmpSensorExtPortEn;
	if (SENSOR_Port & SENSOR_Port_S8)
		SENSOR->SEN_EN[SENSOR_EN_EXT8] = tmpSensorExtPortEn;
	if (SENSOR_Port & SENSOR_Port_S9)
		SENSOR->SEN_EN[SENSOR_EN_EXT9] = tmpSensorExtPortEn;
	if (SENSOR_Port & SENSOR_Port_S10)
		SENSOR->SEN_EN[SENSOR_EN_EXT10] = tmpSensorExtPortEn;
	if (SENSOR_Port & SENSOR_Port_S11)
		SENSOR->SEN_EN[SENSOR_EN_EXT11] = tmpSensorExtPortEn;
	
	return 0;
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void SENSOR_AttackRespMode(SENSOR_RespModeTypeDef SENSOR_ResponseMode)
{
	assert_param(IS_SENSOR_RESP_MODE(SENSOR_ResponseMode));

	if (SENSOR_CPUReset == SENSOR_ResponseMode)
	{
		SENSOR->SEN_EXT_CFG &= ~SEN_EXT_CFG_EXTS_PROC;
	} 
	else if (SENSOR_Interrupt == SENSOR_ResponseMode)
	{
		SENSOR->SEN_EXT_CFG |= SEN_EXT_CFG_EXTS_PROC;
	}
}

/**
  * @brief  
  * @param  
  * @retval None
  */
uint32_t SENSOR_EXTCmd(FunctionalState NewState)
{
	if (DISABLE != NewState)
		SENSOR->SEN_EXTS_START = SEN_ENABLE;
	else
		SENSOR->SEN_EXTS_START = SEN_DISABLE;
	return 0;
}

/**
  * @brief  
  * @param  
  * @retval None
  */
FlagStatus SENSOR_EXTIsRuning(void)
{
	if (SENSOR->SEN_EXTS_START & BIT(31))
		return SET;
	else
		return RESET;
}

/**
  * @brief  
  * @param  
  * @retval None
  */
uint32_t SENSOR_ANACmd(uint32_t SENSOR_ANA, FunctionalState NewState)
{
	volatile uint32_t tmpSensorExtPortEn = 0;
	assert_param(IS_SENSOR_ANA(SENSOR_ANA));

	if (DISABLE != NewState)
		tmpSensorExtPortEn = SEN_ENABLE;
	else
		tmpSensorExtPortEn = SEN_DISABLE;

	if (SENSOR_ANA & SENSOR_ANA_VOL_HIGH)
		SENSOR->SEN_EN[SENSOR_EN_VH] = tmpSensorExtPortEn;
	if (SENSOR_ANA & SENSOR_ANA_VOL_LOW)
		SENSOR->SEN_EN[SENSOR_EN_VL] = tmpSensorExtPortEn;
	if (SENSOR_ANA & SENSOR_ANA_TEMPER_HIGH)
		SENSOR->SEN_EN[SENSOR_EN_TH] = tmpSensorExtPortEn;
	if (SENSOR_ANA & SENSOR_ANA_TEMPER_LOW)
		SENSOR->SEN_EN[SENSOR_EN_TL] = tmpSensorExtPortEn;
	if (SENSOR_ANA & SENSOR_ANA_XTAL32K)
		SENSOR->SEN_EN[SENSOR_EN_XTAL32] = tmpSensorExtPortEn;
	if (SENSOR_ANA & SENSOR_ANA_MESH)
		SENSOR->SEN_EN[SENSOR_EN_MESH] = tmpSensorExtPortEn;
	if (SENSOR_ANA & SENSOR_ANA_VOLGLITCH)
		SENSOR->SEN_EN[SENSOR_EN_VOLGLITCH] = tmpSensorExtPortEn;
	
	return 0;
}

void SENSOR_Lock(uint32_t SENSOR_LOCK)
{
	assert_param(IS_SENSOR_LOCK(SENSOR_LOCK));
	SENSOR->SEN_LOCK = SENSOR_LOCK;
}

uint32_t SENSOR_SetRand(uint32_t Rand)
{
	if (0 != Rand && ~0 != Rand)
	{
		SENSOR->SEN_RNG_INI = Rand;
		return 0;
	}
	else
	{
		return 1;
	}
}
/**
  * @brief  
  * @param  
  * @retval None
  */
void SENSOR_Soft_Enable(void)
{
	SENSOR->SEN_SOFT_EN |= SEN_SOFT_EN_SOFT_ATTACK_EN;
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void SENSOR_SoftAttack(void)
{
    /* soft attack unlock */
	SENSOR->SEN_SOFT_LOCK &= ~SEN_SOFT_LOCK_SOFT_ATTACK_LOCK;
    /* Start soft attack */ 
	SENSOR->SEN_SOFT_ATTACK |= SEN_SOFT_ATTACK_SOFT_ATTACK;
}

/**
  * @brief  
  * @param  ITState
  * @retval None
  */
int32_t SENSOR_GetITStatus(uint32_t ITState)
{
	int32_t bitstatus = 0;
    assert_param(IS_SENSOR_IT(ITState));
	if ((SENSOR->SEN_STATE & ITState) != 0)
	{
		bitstatus = 1;
	}
	else
	{
		bitstatus = 0;
	}
	return bitstatus;
}


/**
  * @brief  
  * @param  None
  * @retval None
  */
int32_t SENSOR_GetITStatusReg(void)
{
	return SENSOR->SEN_STATE;
}


/**
  * @brief  
  * @param  
  * @retval None
  */
void SENSOR_ClearITPendingBit(void)
{
	/* Write Clear all intp state */
    SENSOR->SEN_STATE = 0;
}

/**************************      (C) COPYRIGHT Megahunt    *****END OF FILE****/
