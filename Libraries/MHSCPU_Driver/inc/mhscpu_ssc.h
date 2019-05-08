/**************************************************************************//**
 * @file     mhscpu_ssc.h
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
#ifndef __MHSCPU_SSC_H
#define __MHSCPU_SSC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "mhscpu.h"

typedef enum
{ 
	SSC_RandFreq_1				= 0x3F,	//随机源频率不分频
	SSC_RandFreq_1_2 			= 0xA0,	//随机源频率2分频
	SSC_RandFreq_1_4 			= 0xFF,	//随机源频率4分频
	SSC_RandFreqOff 			= 0xA5	//功能关闭
}SSC_RandFreqTypeDef; 
#define IS_SSC_RAND_FREQ(FREQ) (((FREQ) == SSC_RandFreq_1) || ((FREQ) == SSC_RandFreq_1_2) || \
								((FREQ) == SSC_RandFreq_1_4) || ((FREQ) == SSC_RandFreqOff) )


typedef enum
{
	SSC_ITDisable				= 0x00,	//安全异常响应中断关闭
	SSC_ITEnable				= 0x01,	//安全异常响应中断打开
	SSC_ITEnableWithKeyClean 	= 0x02,	//安全异常响应中断打开，在产生中断同时伴随密钥擦除操作
}SSC_ITModeTypeDef;
#define IS_SSC_IT_MODE(MODE) (((MODE) == SSC_ITDisable) || ((MODE) == SSC_ITEnable) || \
								((MODE) == SSC_ITEnableWithKeyClean))


#define	SSC_ITSysXTAL12M					BIT(18)	//系统主12M时钟标志
#define	SSC_ITSysGlitch						BIT(17)	//主电源毛刺标志
#define	SSC_ITSysVolHigh					BIT(16)	//主电源过压标志
#define	SSC_ITSysVolLow						BIT(15)	//主电源欠压标志
#define	SSC_ITIntRetValueNotExpect			BIT(12)	//中断服务程序返回非匹配值
#define	SSC_ITExeInstructionWhenTCleaned	BIT(11)	//在PSR中T比特位清0时，尝试执行指令
#define	SSC_ITExeUndefinedInstruction		BIT(10)	//执行非定义指令
#define	SSC_ITExeCoprocessorInstruction		BIT(9)	//执行协处理器指令
#define	SSC_ITMPUStackAccessException 		BIT(8)	//MPU下访问栈异常
#define	SSC_ITStackAccessException			BIT(7)	//访问栈异常
#define	SSC_ITIBUSAccessException			BIT(6)	//IBUS访问异常
#define	SSC_ITMPUDataAccessException		BIT(5)	//MPU下指令或数据访问异常
#define	SSC_ITDivZero						BIT(4)	//除0错误异常
#define	SSC_ITParityError					(BIT(3)|BIT(2)|BIT(1)|BIT(0))		//奇偶校验错误异常
#define	SSC_ITParityErrorRegBank			BIT(3)	//奇偶校验寄存器BANK错误
#define	SSC_ITParityErrorMPU				BIT(2)	//奇偶校验MPU下错误
#define	SSC_ITParityErrorIntCtrl			BIT(1)	//奇偶校验中断控制错误
#define	SSC_ITParityErrorInstructionFlow	BIT(0)	//奇偶校验指令流错误


typedef struct
{
	SSC_RandFreqTypeDef IBUS_DBUS_ReadPolarityReversalFreq;		//IBUS_DBUS读数据极性翻转控制
	SSC_RandFreqTypeDef SBUS_ReadPolarityReversalFreq;			//SBUS读数据极性翻转控制
	SSC_RandFreqTypeDef DBUS_SBUS_WritePolarityReversalFreq;	//DBUS_SBUS写数据极性翻转控制
	SSC_RandFreqTypeDef RandBranchInsertFreq;					//随机分支插入使能
	SSC_RandFreqTypeDef IgnoreClockGateFreq;					//忽略时钟门控使能
	FunctionalState RegisterHeapClean;							//寄存器堆清除使能
	FunctionalState ParityCheck;								//奇偶校验使能
}SSC_InitTypeDef;


/*
 *	将BPK分为4块每块256比特为单位设置读写权限
 *	SSC_BPKAccessCtrBlock_0为起始0地址块
 */
#define SSC_BPKAccessCtrBlock_0				(0x01)
#define SSC_BPKAccessCtrBlock_1				(0x02)
#define SSC_BPKAccessCtrBlock_2				(0x04)
#define SSC_BPKAccessCtrBlock_3				(0x08)

typedef enum
{
	SSC_BPKReadOnly		= 0x01,		//BPK块只读
	SSC_BPKWriteOnly	= 0x02,		//BPK块只写
	SSC_BPKReadWrite	= 0x03		//BPK块读写
}SSC_BPKAccessCtrlTypeDef;

#define IS_BPK_ACCESS_CTRL(CTRL) (((CTRL) == SSC_BPKReadOnly) || ((CTRL) == SSC_BPKWriteOnly) || \
								((CTRL) == SSC_BPKReadWrite))

#define SSC_SENSOR_XTAL12M							((uint32_t)0x00000001)
#define SSC_SENSOR_VOL_LOW							((uint32_t)0x00000002)
#define SSC_SENSOR_VOL_HIGH							((uint32_t)0x00000004)
#define SSC_SENSOR_VOLGLITCH						((uint32_t)0x00000008)
#define IS_SSC_SENSOR(SENSOR)						((((SENSOR) & (uint32_t)0xFFFFFFF0) == 0x00) && ((SENSOR) != (uint32_t)0x00))

typedef enum
{
	SSC_SENSOR_CPUReset  = 0,
	SSC_SENSOR_Interrupt = 1
}SSC_SENSOR_RespModeTypeDef;
#define IS_SSC_SENSOR_RESP_MODE(Mode)				((Mode) == SSC_SENSOR_CPUReset ||\
													(Mode) == SSC_SENSOR_Interrupt)

/**
  * @method	SSC_Init
  * @brief	SSC安全特性初始化
  * @param	SSC_InitTypeDef SSC_InitStruct
  * @retval void
  */
void SSC_Init(SSC_InitTypeDef SSC_InitStruct);
/**
  * @method	SSC_ITConfig
  * @brief	SSC安全中断响应配置
  * @param	uint32_t SSC_IT
  * @param	SSC_ITModeTypeDef SSC_ITMode
  * @retval void
  */
void SSC_ITConfig(uint32_t SSC_IT, SSC_ITModeTypeDef SSC_ITMode);
/**
  * @method	SSC_GetITStatus
  * @brief	SSC安全中断状态
  * @param	uint32_t SSC_IT
  * @retval ITStatus
  */
ITStatus SSC_GetITStatus(uint32_t SSC_IT);
/**
  * @method	SSC_ClearITPendingBit
  * @brief	SSC安全中断清楚
  * @param	uint32_t SSC_IT
  * @retval void
  */
void SSC_ClearITPendingBit( uint32_t SSC_IT);

/**
  * @method	SSC_SetDataRAMScrambler
  * @brief	设置数据RAM扰码
  * @param	uint32_t Scrambler
  * @retval void
  */
void SSC_SetDataRAMScrambler(uint32_t Scrambler);
/**
  * @method	SSC_BPKAccessCtrConfig
  * @brief	设置BPK访问权限
  * @param	uint32_t SSC_BPKAccessCtrBlock
  * @param	SSC_BPKAccessCtrlTypeDef SSC_BPKAccessCtr
  * @retval void
  */
void SSC_BPKAccessCtrlConfig(uint32_t SSC_BPKAccessCtrBlock, SSC_BPKAccessCtrlTypeDef SSC_BPKAccessCtr);

/**
  * @method	SSC_SENSOR_Enable
  * @brief	开启系统Sensor
  * @param	SSC_SENSOR
  * @retval 
  */
uint32_t SSC_SENSORCmd(uint32_t SSC_SENSOR, FunctionalState NewState);

/**
  * @method	SSC_SENSORLock
  * @brief	锁定系统Sensor开启状态
  * @param	SSC_SENSOR
  * @retval 
  */
void SSC_SENSORLock(uint32_t SSC_SENSOR);


/**
  * @method	SSC_SENSOR_AttackRespMode
  * @brief	系统Sensor响应方式
  * @param	SSC_SENSOR_RespMode
  * @retval 
  */
void SSC_SENSORAttackRespMode(SSC_SENSOR_RespModeTypeDef SSC_SENSOR_RespMode);

#ifdef __cplusplus
}
#endif

#endif 

/**************************      (C) COPYRIGHT Megahunt    *****END OF FILE****/
