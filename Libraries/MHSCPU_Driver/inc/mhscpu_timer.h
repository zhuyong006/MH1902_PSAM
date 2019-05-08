#ifndef __MHSCPU_TIMER_H
#define __MHSCPU_TIMER_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "mhscpu.h"

typedef enum
{
	TIM_0		= 0,
	TIM_1		= 1,
	TIM_2		= 2,
	TIM_3		= 3,
	TIM_4		= 4,
	TIM_5		= 5
}TIM_NumTypeDef;

typedef struct 
{
	TIM_NumTypeDef TIMx;
	uint32_t TIM_Period;			/*!< Specifies the period value to be loaded into the active
									   Auto-Reload Register at the next update event.
									   This parameter must be a number between 0x0000 and 0xFFFFFFFF.  */ 
}TIM_InitTypeDef;

typedef struct 
{
	TIM_NumTypeDef TIMx;
	uint32_t TIM_LowLevelPeriod;
	uint32_t TIM_HighLevelPeriod;
}TIM_PWMInitTypeDef;

typedef enum
{
	TIM_Mode_General	= 0,
	TIM_Mode_PWM		= 1
}TIM_ModeTypeDef;
#define IS_TIM_MODE(MODE)							(MODE == TIM_Mode_General || MODE == TIM_Mode_PWM)


void TIM_DeInit(TIM_Module_TypeDef * TIMMx);
void TIM_Init(TIM_Module_TypeDef* TIMMx, TIM_InitTypeDef* TIM_InitStruct);
void TIM_PWMInit(TIM_Module_TypeDef* TIMMx, TIM_PWMInitTypeDef* TIM_PWMInitStruct);
void TIM_Cmd(TIM_Module_TypeDef* TIMMx, TIM_NumTypeDef TIMx, FunctionalState NewState);
void TIM_ModeConfig(TIM_Module_TypeDef* TIMMx, TIM_NumTypeDef TIMx, TIM_ModeTypeDef TIM_Mode);
void TIM_SetPeriod(TIM_Module_TypeDef* TIMMx, TIM_NumTypeDef TIMx, uint32_t Period);
void TIM_SetPWMPeriod(TIM_Module_TypeDef* TIMMx, TIM_NumTypeDef TIMx, uint32_t PWM_LowLevelPeriod, uint32_t PWM_HighLevelPeriod);
void TIM_ITConfig(TIM_Module_TypeDef* TIMMx, TIM_NumTypeDef TIMx, FunctionalState NewState);
void TIM_ClearITPendingBit(TIM_Module_TypeDef* TIMMx, TIM_NumTypeDef TIMx);
ITStatus TIM_GetITStatus(TIM_Module_TypeDef* TIMMx, TIM_NumTypeDef TIMx);
uint32_t TIM_GetAllITStatus(TIM_Module_TypeDef* TIMMx);
ITStatus TIM_GetRawITStatus(TIM_Module_TypeDef* TIMMx, TIM_NumTypeDef TIMx);
uint32_t TIM_GetAllRawITStatus(TIM_Module_TypeDef* TIMMx);

#ifdef __cplusplus
}
#endif

#endif

