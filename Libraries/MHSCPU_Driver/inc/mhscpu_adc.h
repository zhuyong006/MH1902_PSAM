/************************ (C) COPYRIGHT Megahuntmicro *************************
 * File Name            : mhscpu_adc.h
 * Author               : Megahuntmicro
 * Version              : V1.0.0
 * Date                 : 09/12/2016
 * Description          : ADC headfile.
 *****************************************************************************/
 
 
#ifndef __MHSCPU_ADC_H__
#define __MHSCPU_ADC_H__
 
 
#ifdef __cplusplus
extern "C" {
#endif
    
/* Include ------------------------------------------------------------------*/
#include "mhscpu.h" 
    
    
    
    
    #define ADC_REF_mV  (1215)

    #define ADC_START_BIT       BIT(8)
    #define ADC_IRQ_EN_BIT      BIT(5)
    #define ADC_RATE_MASK       (0x03 << 3)
    #define ADC_CHAN_MASK       (0x07 << 0)
     
    /* ADC Channel select */  
    typedef enum
    {
        ADC_CHANNEL_0 = 0,
        ADC_CHANNEL_1,
        ADC_CHANNEL_2,
        ADC_CHANNEL_3,
        ADC_CHANNEL_4,
        ADC_CHANNEL_5,
        ADC_CHANNEL_VDD
    }ADC_ChxTypeDef;
    #define IS_ADC_CHANNEL(CHANNEL_NUM) (((CHANNEL_NUM) == ADC_CHANNEL_0) || \
                                         ((CHANNEL_NUM) == ADC_CHANNEL_1) || \
                                         ((CHANNEL_NUM) == ADC_CHANNEL_2) || \
                                         ((CHANNEL_NUM) == ADC_CHANNEL_3) || \
                                         ((CHANNEL_NUM) == ADC_CHANNEL_4) || \
                                         ((CHANNEL_NUM) == ADC_CHANNEL_5) || \
                                         ((CHANNEL_NUM) == ADC_CHANNEL_VDD))
    /* ADC Samp Select */
    typedef enum
    {
        ADC_SAMP_SEL_600K = 0,
        ADC_SAMP_SEL_300K,
        ADC_SAMP_SEL_150K,
        ADC_SAMP_SEL_50K,
    }ADC_SampTypeDef;
    #define IS_ADC_SAMP(SAMP)           (((SAMP) == ADC_SAMP_SEL_50K)  || \
                                         ((SAMP) == ADC_SAMP_SEL_150K) || \
                                         ((SAMP) == ADC_SAMP_SEL_300K) || \
                                         ((SAMP) == ADC_SAMP_SEL_600K))
    /* ADC Struct Define*/  
    typedef struct _ADC_InitTypeDef
    {
        ADC_ChxTypeDef      ADC_Channel;            /* ADC Channel select */
        ADC_SampTypeDef     ADC_SampSpeed;          /* ADC Channel select */
        FunctionalState     ADC_IRQ_EN;             /* ADC Samp Select */
    } ADC_InitTypeDef;  
        


    /* Exported constants -------------------------------------------------------*/ 
    /* Exported macro -----------------------------------------------------------*/ 
    /* Exported functions -------------------------------------------------------*/ 
    extern void     ADC_Init(ADC_InitTypeDef *ADC_InitStruct);
    extern int32_t  ADC_GetResult(ADC_ChxTypeDef ADC_Channel);
    extern uint32_t ADC_GetResult_Wait(ADC_ChxTypeDef ADC_Channel);
    extern int32_t  ADC_GetVoltage(ADC_ChxTypeDef ADC_Channel);
    extern uint32_t ADC_CalVoltage(uint32_t u32ADC_Value);
    extern uint32_t ADC_Filt(ADC_ChxTypeDef ADC_Channel);
    /* Exported variables -------------------------------------------------------*/
    /* Exported types -----------------------------------------------------------*/

#ifdef __cplusplus
}
#endif   

#endif  /* __MHSCPU_ADC_H__ */
/************************ (C) COPYRIGHT 2016 Megahuntmicro ****END OF FILE****/
