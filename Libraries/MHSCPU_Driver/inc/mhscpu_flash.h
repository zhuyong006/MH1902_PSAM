/**
  ******************************************************************************
  * @file    stm32f10x_flash.h
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file contains all the functions prototypes for the FLASH 
  *          firmware library.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MHSCPU_FLASH_H
#define __MHSCPU_FLASH_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <mhscpu.h>
/** 
  * @brief  FLASH Status  
  */

typedef enum
{ 
  FLASH_BUSY = 1,
  FLASH_ERROR_PG,
  FLASH_ERROR_WRP,
  FLASH_COMPLETE,
  FLASH_TIMEOUT
}FLASH_Status;



#define IS_FLASH_ADDRESS(ADDRESS) (((ADDRESS) > FLASH_BASE - 1) && ((ADDRESS) < FLASH_BASE + FLASH_SIZE))

#define IS_OTP_ADDRESS(ADDRESS) (((ADDRESS) > OTP_BASE - 1) && ((ADDRESS) < OTP_BASE + OTP_SIZE))


void FLASH_Unlock(void);
void FLASH_Lock(void);

void FLASH_Protect(uint32_t u32Addr);
void FLASH_ProtectLock(uint32_t u32Addr);

void FLASH_SetProtect(uint8_t u8Type, uint32_t u32RO);
void FLASH_SetProtectLock(uint8_t u8Type, uint32_t u32Lock);

void FLASH_UnProtect(uint32_t u32Addr);
void FLASH_SetUnProtect(uint8_t u8Type, uint32_t u32RO);

uint32_t FLASH_GetProtect(uint8_t u8Type);
uint32_t FLASH_GetProtectLock(uint8_t u8Type);


uint32_t FLASH_IsProtect(uint32_t u32Addr);
uint32_t FLASH_IsProtectLock(uint32_t u32Addr);

void FLASH_SetLatency(uint32_t u32UsClk);

FLASH_Status FLASH_EraseAllPages(void);
FLASH_Status FLASH_ErasePage(uint32_t u32Addr);
FLASH_Status FLASH_ProgramWord(uint32_t u32Addr, uint32_t u32Data);
FLASH_Status FLASH_WriteBlock(uint32_t u32Addr, void *pvBuf, uint32_t len);


#ifdef __cplusplus
}
#endif

#endif /* __STM32F10x_FLASH_H */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
