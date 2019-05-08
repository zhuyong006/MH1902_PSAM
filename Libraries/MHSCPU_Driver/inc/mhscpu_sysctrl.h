/**************************************************************************//**
 * @file     mhscpu_sysctrl.h
 * @brief    
 * @version  V1.00
 * @date     11. April 2015
 *
 * @note
 *
 ******************************************************************************/
/* Copyright (c) 2012 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/
#ifndef __MHSCPU_SYSCTRL_H
#define __MHSCPU_SYSCTRL_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "mhscpu.h"
#include "mhscpu_dma.h"

#define SYSCTRL_FREQ_SEL_POWERMODE_Pos                  (16)
#define SYSCTRL_FREQ_SEL_POWERMODE_Mask                 (0x07 << SYSCTRL_FREQ_SEL_POWERMODE_Pos)     
#define SYSCTRL_FREQ_SEL_POWERMODE_CLOSE_CPU            (0x00 << SYSCTRL_FREQ_SEL_POWERMODE_Pos) 
#define SYSCTRL_FREQ_SEL_POWERMODE_CLOSE_CPU_MEM        (0x01 << SYSCTRL_FREQ_SEL_POWERMODE_Pos) 
#define SYSCTRL_FREQ_SEL_POWERMODE_CLOSE_ALL_BUT_IO     (0x02 << SYSCTRL_FREQ_SEL_POWERMODE_Pos) 

/* AHB 总线外设 */
#define SYSCTRL_AHBPeriph_DMA                           ((uint32_t)0x20000000)
#define SYSCTRL_AHBPeriph_USB                           ((uint32_t)0x10000000)
#define SYSCTRL_AHBPeriph_DCMI                          ((uint32_t)0x00000004)
#define SYSCTRL_AHBPeriph_LCD                           ((uint32_t)0x00000002)
#define SYSCTRL_AHBPeriph_CRYPT                         ((uint32_t)0x00000001)
#define IS_SYSCTRL_AHB_PERIPH(PERIPH)                   ((((PERIPH) & 0x30000007) != 0x00) && ((PERIPH) != 0x00))
 
/* APB 总线外设 */
#define SYSCTRL_APBPeriph_TRNG                          ((uint32_t)0x80000000)
#define SYSCTRL_APBPeriph_ADC                           ((uint32_t)0x40000000)
#define SYSCTRL_APBPeriph_CRC                           ((uint32_t)0x20000000)
#define SYSCTRL_APBPeriph_IMG_COP						((uint32_t)0x10000000)
#define SYSCTRL_APBPeriph_BPU                           ((uint32_t)0x04000000)
#define SYSCTRL_APBPeriph_TIMM0                         ((uint32_t)0x00200000)
#define SYSCTRL_APBPeriph_GPIO                          ((uint32_t)0x00100000)
#define SYSCTRL_APBPeriph_SCI0                          ((uint32_t)0x00004000)
#define SYSCTRL_APBPeriph_SPI2							((uint32_t)0x00000400)
#define SYSCTRL_APBPeriph_SPI1							((uint32_t)0x00000200)
#define SYSCTRL_APBPeriph_SPI0                          ((uint32_t)0x00000100)
#define SYSCTRL_APBPeriph_UART1                         ((uint32_t)0x00000002)
#define SYSCTRL_APBPeriph_UART0                         ((uint32_t)0x00000001)
#define IS_SYSCTRL_APB_PERIPH(PERIPH)                   ((((PERIPH) & 0xF4304703) != 0x00) && ((PERIPH) != 0x00))

/* SOFT_RST2*/
#define SYSCTRL_GLB_RESET                               ((uint32_t)0x80000000)
#define SYSCTRL_CM3_RESET                               ((uint32_t)0x40000000)
#define IS_SYSCTRL_AHB_PERIPH_RESET(PERIPH)             ((((PERIPH) & 0xF000000F) != 0x00) && ((PERIPH) != 0x00))
                                                                                                                                                               
/* Wake_FLAG1 */
#define SYSCTRL_WakeInt1_DCMI                       	(BIT25)
#define SYSCTRL_WakeInt1_MSR                       	    (BIT24)
#define SYSCTRL_WakeInt1_EXTI_INT3                  	(BIT23)
#define SYSCTRL_WakeInt1_EXTI_INT2                  	(BIT22)
#define SYSCTRL_WakeInt1_EXTI_INT1                  	(BIT21)
#define SYSCTRL_WakeInt1_EXTI_INT0                  	(BIT20)
#define SYSCTRL_WakeInt1_SCI0                       	(BIT14)
#define SYSCTRL_WakeInt1_SPI0                           (BIT8)
#define SYSCTRL_WakeInt1_UART1                          (BIT1)
#define SYSCTRL_WakeInt1_UART0                          (BIT0)
/* Wake_FLAG2 */
#define SYSCTRL_WakeInt2_TIM0_5                         (BIT13)
#define SYSCTRL_WakeInt2_TIM0_4                         (BIT12)
#define SYSCTRL_WakeInt2_TIM0_3                         (BIT11)
#define SYSCTRL_WakeInt2_TIM0_2                         (BIT10)
#define SYSCTRL_WakeInt2_TIM0_1                         (BIT9)
#define SYSCTRL_WakeInt2_TIM0_0                         (BIT8)
#define SYSCTRL_WakeInt2_USB                            (BIT3)
#define SYSCTRL_WakeInt2_RTC                            (BIT2)
/* 仅存在于Wake_Flag2寄存器 */
#define SYSCTRL_WakeInt2_SENSOR                         (BIT0)


/** @defgroup SYSCTRL_Exported_Types
  * @{
  */

typedef struct
{
    uint32_t PLL_Frequency;     /*!< returns PLL frequency expressed in Hz */
    uint32_t HCLK_Frequency;    /*!< returns HCLK frequency expressed in Hz */
    uint32_t PCLK_Frequency;    /*!< returns PCLK frequency expressed in Hz */
}SYSCTRL_ClocksTypeDef;

/**
  * @}
  */


typedef enum
{
    ///关闭CPU时钟
    SleepMode_CpuOff = 0,
    ///关闭CPU时钟,ROM可选断电，SRAM进入retain
    SleepMode_CpuMemOff,
    SleepMode_Invalid
}SleepMode_TypeDef;
#define IS_ALL_SLEEP_MODE(MODE)                     ((MODE) < SleepMode_Invalid) 

typedef enum 
{
    SELECT_EXT12M,
    SELECT_INC12M
} SYSCLK_SOURCE_TypeDef;
#define IS_SYSCLK_SOURCE(FREQ)                      (((FREQ) == SELECT_EXT12M) || \
                                                    ((FREQ) == SELECT_INC12M))
typedef enum
{
    SYSCTRL_PLL_72MHz      = (uint32_t)0x01,
	SYSCTRL_PLL_96MHz      = (uint32_t)0x02,
}SYSCTRL_PLL_TypeDef;
#define IS_PLL_FREQ(FREQ)                           (((FREQ) == SYSCTRL_PLL_96MHz) || \
                                                     ((FREQ) == SYSCTRL_PLL_72MHz))

#define SYSCTRL_HCLK_Div_None                       ((uint32_t)0x00)
#define SYSCTRL_HCLK_Div2                           ((uint32_t)0x01)
#define SYSCTRL_HCLK_Div4                           ((uint32_t)0x02)
#define IS_GET_SYSCTRL_HCLK_DIV(DIV)                (((DIV) == SYSCTRL_HCLK_Div_None) || \
                                                    ((DIV) == SYSCTRL_HCLK_Div2) || \
                                                    ((DIV) == SYSCTRL_HCLK_Div4)) 

#define SYSCTRL_PCLK_Div2                           ((uint32_t)0x00)
#define SYSCTRL_PCLK_Div4                           ((uint32_t)0x01)
#define IS_GET_SYSCTRL_PCLK_DIV(DIV)                (((DIV) == SYSCTRL_PCLK_Div2) || \
                                                    ((DIV) == SYSCTRL_PCLK_Div4)) 
                                                    
/********************* System control APIs ***************************************/
void SYSCTRL_AHBPeriphClockCmd(uint32_t SYSCTRL_AHBPeriph, FunctionalState NewState);
void SYSCTRL_AHBPeriphResetCmd(uint32_t SYSCTRL_AHBPeriph, FunctionalState NewState);
void SYSCTRL_APBPeriphClockCmd(uint32_t SYSCTRL_APBPeriph, FunctionalState NewState);
void SYSCTRL_APBPeriphResetCmd(uint32_t SYSCTRL_APBPeriph, FunctionalState NewState);

void SYSCTRL_EnterSleep(SleepMode_TypeDef SleepMode);
uint32_t SYSCTRL_GetWakeupFlag1Status(void);
uint32_t SYSCTRL_GetWakeupFlag2Status(void);

void SYSCTRL_SYSCLKSourceSelect(SYSCLK_SOURCE_TypeDef source_select);
void SYSCTRL_PLLConfig(SYSCTRL_PLL_TypeDef PLL_Freq);
void SYSCTRL_HCLKConfig(uint32_t HCLK_Div);
void SYSCTRL_PCLKConfig(uint32_t PCLK_Div);

void SYSCTRL_GetClocksFreq(SYSCTRL_ClocksTypeDef* SYSCTRL_Clocks);

#ifdef __cplusplus
}
#endif


#endif 

/**************************      (C) COPYRIGHT Megahunt    *****END OF FILE****/
