#include "mhscpu_rtc.h"

#define RTC_CS_RTC_CLR						((uint32_t)0x10)
#define RTC_READY_FLAG						((uint32_t)0x08)
#define RTC_IT_ALARM						((uint32_t)0x04)
#define RTC_LOCK_TIM						((uint32_t)0x02)
#define RTC_FLAG_ALARM						((uint32_t)0x01)


/**
  * @brief  
  * @param  
  * @retval None
  */
FlagStatus RTC_Is_Ready(void)
{
	if ((RTC->RTC_CS & RTC_READY_FLAG) != (uint32_t)RESET)
	{
		return SET;
	}

	return RESET;
	
}


/**
  * @brief  
  * @param  
  * @retval None
  */
void RTC_ResetCounter(void)
{
	while(RTC_Is_Ready() == RESET);  //RTC can't config when in reset status;
	
	RTC->RTC_CS |= RTC_CS_RTC_CLR;
}

/**
  * @brief  
  * @param  
  * @retval None
  */
uint32_t RTC_GetCounter(void)
{
	uint32_t CurrentCounter = 0;
	
	while(RTC_Is_Ready() == RESET);  //RTC can't config when in reset status;
	
	RTC->RTC_CS |= RTC_LOCK_TIM;
	CurrentCounter = RTC->RTC_TIM;
	RTC->RTC_CS &= ~RTC_LOCK_TIM;
	
	return CurrentCounter;
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void RTC_SetAlarm(uint32_t AlarmValue)
{
	while(RTC_Is_Ready() == RESET); //RTC can't config when in reset status;
	
	RTC->RTC_ARM = AlarmValue;
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void RTC_SetRefRegister(uint32_t RefRegValue)
{
	while(RTC_Is_Ready() == RESET); //RTC can't config when in reset status;
	
	RTC->RTC_REF = RefRegValue; //at the same time tim set to 0
}

/**
  * @brief  
  * @param  
  * @retval
  */
uint32_t RTC_GetRefRegister(void)
{
	while(RTC_Is_Ready() == RESET);
	
	return RTC->RTC_REF;
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void RTC_ITConfig(FunctionalState NewState)
{
    assert_param(IS_FUNCTIONAL_STATE(NewState));
    if (DISABLE != NewState)
    {
        while(RTC_Is_Ready() == RESET); //RTC can't config when in reset status;
        RTC->RTC_CS |= RTC_IT_ALARM;
    }
    else 
    {
        while(RTC_Is_Ready() == RESET); //RTC can't config when in reset status;
        RTC->RTC_CS &= ~RTC_IT_ALARM;
    }
}

/**
  * @brief  
  * @param  
  * @retval None
  */
void RTC_ClearITPendingBit(void)
{
	while(RTC_Is_Ready() == RESET); //RTC can't config when in reset status;
	
	RTC->RTC_INTCLR = 0;
}

/**
  * @brief  
  * @param  
  * @retval None
  */
ITStatus RTC_GetITStatus(void)
{
	if ((RTC->RTC_CS & RTC_FLAG_ALARM) != (uint32_t)RESET)
	{
		return SET;
	}

	return RESET;
}

/**
  * @brief  
  * @param  
  * @retval None
  */
uint32_t RTC_GetAttrackTime(void)
{
	while(RTC_Is_Ready() == RESET);

    return (RTC->RTC_ATTA_TIM);
}
/**************************      (C) COPYRIGHT Megahunt    *****END OF FILE****/
