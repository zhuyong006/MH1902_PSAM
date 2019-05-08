/**************************************************************************//**
 * @file     mhscpu_ssc.c
 * @brief    This file provides all the system firmware function.
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
#include "mhscpu_ssc.h"


#define SSC_SEN_ENABLE									((uint32_t)0xAA)
#define SSC_SEN_DISABLE									((uint32_t)0x55)

void SSC_Init(SSC_InitTypeDef SSC_InitStruct)
{
	assert_param(IS_SSC_RAND_FREQ(SSC_InitStruct.IBUS_DBUS_ReadPolarityReversalFreq));
	assert_param(IS_SSC_RAND_FREQ(SSC_InitStruct.SBUS_ReadPolarityReversalFreq));
	assert_param(IS_SSC_RAND_FREQ(SSC_InitStruct.DBUS_SBUS_WritePolarityReversalFreq));
	assert_param(IS_SSC_RAND_FREQ(SSC_InitStruct.RandBranchInsertFreq));
	assert_param(IS_SSC_RAND_FREQ(SSC_InitStruct.IgnoreClockGateFreq));
	assert_param(IS_FUNCTIONAL_STATE(SSC_InitStruct.RegisterHeapClean));
	assert_param(IS_FUNCTIONAL_STATE(SSC_InitStruct.ParityCheck));
	
	SSC->SSC_CR1 = (SSC_InitStruct.IBUS_DBUS_ReadPolarityReversalFreq << 24 |
					SSC_InitStruct.SBUS_ReadPolarityReversalFreq << 16 |
					SSC_InitStruct.DBUS_SBUS_WritePolarityReversalFreq << 8 |
					SSC_InitStruct.RandBranchInsertFreq);

	SSC->SSC_CR2 = (SSC_InitStruct.IgnoreClockGateFreq << 8 |
					(SSC_InitStruct.RegisterHeapClean == ENABLE ? 0xFF : 0xA5));
	
	SSC->SSC_CR3 = (SSC->SSC_CR3 & ~0x3FUL) | (SSC_InitStruct.ParityCheck == ENABLE ? 0x00 : 0x10);
	
	SSC->SSC_SR_CLR = 0x7FFF;
}

void SSC_ITConfig(uint32_t SSC_IT, SSC_ITModeTypeDef SSC_ITMode)
{
	assert_param(IS_SSC_IT_MODE(SSC_ITMode));
	
	if (SSC_IT & SSC_ITIntRetValueNotExpect)
	{
		SSC->SSC_ACK = (SSC->SSC_ACK & ~(0x03 << 18)) | SSC_ITMode << 18;
	}
	if (SSC_IT & SSC_ITExeInstructionWhenTCleaned)
	{
		SSC->SSC_ACK = (SSC->SSC_ACK & ~(0x03 << 16)) | SSC_ITMode << 16;
	}
	if (SSC_IT & SSC_ITExeUndefinedInstruction)
	{
		SSC->SSC_ACK = (SSC->SSC_ACK & ~(0x03 << 14)) | SSC_ITMode << 14;
	}
	if (SSC_IT & SSC_ITExeCoprocessorInstruction)
	{
		SSC->SSC_ACK = (SSC->SSC_ACK & ~(0x03 << 12)) | SSC_ITMode << 12;
	}
	if (SSC_IT & SSC_ITMPUStackAccessException)
	{
		SSC->SSC_ACK = (SSC->SSC_ACK & ~(0x03 << 10)) | SSC_ITMode << 10;
	}
	if (SSC_IT & SSC_ITStackAccessException)
	{
		SSC->SSC_ACK = (SSC->SSC_ACK & ~(0x03 << 8)) | SSC_ITMode << 8;
	}
	if (SSC_IT & SSC_ITIBUSAccessException)
	{
		SSC->SSC_ACK = (SSC->SSC_ACK & ~(0x03 << 6)) | SSC_ITMode << 6;
	}
	if (SSC_IT & SSC_ITMPUDataAccessException)
	{
		SSC->SSC_ACK = (SSC->SSC_ACK & ~(0x03 << 4)) | SSC_ITMode << 4;
	}
	if (SSC_IT & SSC_ITDivZero)
	{
		SSC->SSC_ACK = (SSC->SSC_ACK &= ~(0x03 << 2)) | SSC_ITMode << 2;
	}
	if (SSC_IT & SSC_ITParityError)
	{
		SSC->SSC_ACK = (SSC->SSC_ACK &= ~0x03) | SSC_ITMode;
	}
}

ITStatus SSC_GetITStatus(uint32_t SSC_IT)
{
	if (SSC->SSC_SR && SSC_IT)
	{
		return SET;
	}
	return RESET;
}

void SSC_ClearITPendingBit( uint32_t SSC_IT)
{
	if (SSC_IT & SSC_ITSysXTAL12M)
	{
		SSC->SSC_SR |= SSC_ITSysXTAL12M;
	}
	if (SSC_IT & SSC_ITSysGlitch)
	{
		SSC->SSC_SR |= SSC_ITSysGlitch;
	}
	if (SSC_IT & SSC_ITSysVolHigh)
	{
		SSC->SSC_SR |= SSC_ITSysVolHigh;
	}
	if (SSC_IT & SSC_ITSysVolLow)
	{
		SSC->SSC_SR |= SSC_ITSysVolLow;
	}
	
	SSC_IT &= ~(SSC_ITSysXTAL12M | SSC_ITSysGlitch | SSC_ITSysVolHigh | SSC_ITSysVolLow);
	SSC->SSC_SR_CLR = SSC_IT;
}

void SSC_SetDataRAMScrambler(uint32_t Scrambler)
{
	SSC->DATARAM_SCR = Scrambler;
}

void SSC_BPKAccessCtrlConfig(uint32_t SSC_BPKAccessCtrBlock, SSC_BPKAccessCtrlTypeDef SSC_BPKAccessCtr)
{
	assert_param(IS_BPK_ACCESS_CTRL(SSC_BPKAccessCtr));

	if (SSC_BPKAccessCtr == SSC_BPKReadOnly)
	{
		SSC->BPU_RWC |= SSC_BPKAccessCtrBlock << 4;
		SSC->BPU_RWC &= ~(SSC_BPKAccessCtrBlock);
	}
	else if (SSC_BPKAccessCtr == SSC_BPKWriteOnly)
	{
		SSC->BPU_RWC &= ~(SSC_BPKAccessCtrBlock << 4);
		SSC->BPU_RWC |= SSC_BPKAccessCtrBlock;
	}
	else if (SSC_BPKAccessCtr == SSC_BPKReadWrite)
	{
		SSC->BPU_RWC |= (SSC_BPKAccessCtrBlock << 4 | SSC_BPKAccessCtrBlock);
	}
}

uint32_t SSC_SENSORCmd(uint32_t SSC_SENSOR, FunctionalState NewState)
{
	uint32_t sensor_en = SSC_SEN_ENABLE;
	assert_param(IS_SSC_SENSOR(SSC_SENSOR));

	if (DISABLE != NewState)
	{
		sensor_en = SSC_SEN_ENABLE;
	}
	else
	{
		sensor_en = SSC_SEN_DISABLE;
	}

	if (SSC_SENSOR & SSC_SENSOR_XTAL12M)
	{
		SSC->MAIN_SEN_EN = (SSC->MAIN_SEN_EN & ~0xFFUL) | sensor_en;
	}
	if (SSC_SENSOR & SSC_SENSOR_VOL_LOW)
	{
		SSC->MAIN_SEN_EN = (SSC->MAIN_SEN_EN & ~(0xFFUL << 8)) | (sensor_en << 8);
	}
	if (SSC_SENSOR & SSC_SENSOR_VOL_HIGH)
	{
		SSC->MAIN_SEN_EN = (SSC->MAIN_SEN_EN & ~(0xFFUL << 16)) | (sensor_en << 16);
	}
	if (SSC_SENSOR & SSC_SENSOR_VOLGLITCH)
	{
		SSC->MAIN_SEN_EN = (SSC->MAIN_SEN_EN & ~(0xFFUL << 24)) | (sensor_en << 24);
	}
	
	return 0;
}

void SSC_SENSORLock(uint32_t SSC_SENSOR)
{
	assert_param(IS_SSC_SENSOR(SSC_SENSOR));
	SSC->MAIN_SEN_LOCK = SSC_SENSOR;
}

void SSC_SENSORAttackRespMode(SSC_SENSOR_RespModeTypeDef SSC_SENSOR_RespMode)
{
	assert_param(IS_SSC_SENSOR_RESP_MODE(SSC_SENSOR_RespMode));

	if (SSC_SENSOR_CPUReset == SSC_SENSOR_RespMode)
	{
		SSC->SSC_CR3 &= ~BIT(28);
	} 
	else if (SSC_SENSOR_Interrupt == SSC_SENSOR_RespMode)
	{
		SSC->SSC_CR3 |= BIT(28);
	}
}


/**************************      (C) COPYRIGHT Megahunt    *****END OF FILE****/
