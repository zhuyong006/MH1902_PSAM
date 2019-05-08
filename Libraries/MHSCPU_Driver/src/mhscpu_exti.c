#include "mhscpu_exti.h"

/** @defgroup EXTI 
  * @brief EXTI driver modules
  * @{
  */

/** @defgroup EXTI_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @defgroup EXTI_Private_Defines
  * @{
  */


/**
  * @}
  */

/** @defgroup EXTI_Private_Macros
  * @{
  */
#define	EXIT_Num						4
#define	EXIT_Pin_Num					16
/**
  * @}
  */

/** @defgroup EXTI_Private_Variables
  * @{
  */

/**
  * @}
  */

/** @defgroup EXTI_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */


/**
  * @method	EXTI_DeInit
  * @brief	
  * @param	
  * @retval 
  */
void EXTI_DeInit(void)
{
	uint32_t i;
	for (i = 0; i < EXIT_Num; i++)
	{
		GPIO->INTP_TYPE_STA[i].INTP_TYPE = 0;
		GPIO->INTP_TYPE_STA[i].INTP_STA = 0xFFFF;
	}
}

/**
  * @method	EXTI_LineConfig
  * @brief	
  * @param	EXTI_Line
  * @param	EXTI_PinSource
  * @param	EXTI_Trigger
  * @retval 
  */
void EXTI_LineConfig(uint32_t EXTI_Line, uint32_t EXTI_PinSource, EXTI_TriggerTypeDef EXTI_Trigger)
{
	uint32_t i;
	assert_param(IS_EXTI_LINE(EXTI_Line)); 
	assert_param(IS_EXTI_PIN_SOURCE(EXTI_PinSource)); 
	assert_param(IS_EXTI_TRIGGER(EXTI_Trigger)); 
	
	for (i = 0; i < EXIT_Pin_Num; i++)
	{
		if (EXTI_PinSource & (0x01 << i))
		{
			GPIO->INTP_TYPE_STA[EXTI_Line].INTP_TYPE &= ~(0x03 << (i * 2));
			GPIO->INTP_TYPE_STA[EXTI_Line].INTP_TYPE |= (EXTI_Trigger << (i * 2));
		}
	}
}

/**
  * @method	EXTI_GetITStatus
  * @brief	
  * @param	
  * @retval 
  */
uint32_t EXTI_GetITStatus(void)
{
	uint32_t u32ret = 0;
	u32ret |= GPIO->INTP[0] << 0;
	u32ret |= GPIO->INTP[1] << 1;
	u32ret |= GPIO->INTP[2] << 2;
	u32ret |= GPIO->INTP[3] << 3;

	return u32ret;
}

/**
  * @method	EXTI_GetITLineStatus
  * @brief	
  * @param	EXTI_Line
  * @retval 
  */
uint32_t EXTI_GetITLineStatus(uint32_t EXTI_Line)
{
	return (GPIO->INTP_TYPE_STA[EXTI_Line].INTP_STA);
}

/**
  * @method	EXTI_ClearITPendingBit
  * @brief	
  * @param	EXTI_Line
  * @retval 
  */
void EXTI_ClearITPendingBit(uint32_t EXTI_Line)
{
	GPIO->INTP_TYPE_STA[EXTI_Line].INTP_STA = 0xFFFF;
}
