/**
  ******************************************************************************
  * @file    stm32f10x_flash.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    11-March-2011
  * @brief   This file provides all the FLASH firmware functions.
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

/* Includes ------------------------------------------------------------------*/
#include "mhscpu_conf.h"
#include "mhscpu_flash.h"


/** @addtogroup STM32F10x_StdPeriph_Driver
  * @{
  */

/** @defgroup FLASH 
  * @brief FLASH driver modules
  * @{
  */ 

/** @defgroup FLASH_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */ 

/** @defgroup FLASH_Private_Defines
  * @{
  */ 

/*Flash operate*/
#define OP_SEL_PROGRAM          (0)
#define OP_SEL_ERASE_ALL   		(1)
#define OP_SEL_PAGE_ERASE       (2)

/*Flash unlock key*/
#define FLASH_KEY1 				(0xABCD00A5)
#define FLASH_KEY2 				(0x1234005A)

static uint32_t gu32FLASH_Key1 = 0;
static uint32_t gu32FLASH_Key2 = 0;

#define pfFlash_Done()     ((uint32_t (*)(void))(*(uint32_t *)0x0000001C))()

/**
  * @}
  */ 

/** @defgroup FLASH_Private_Macros
  * @{
  */

/**
  * @}
  */ 

/** @defgroup FLASH_Private_Variables
  * @{
  */

/**
  * @}
  */ 

/** @defgroup FLASH_Private_FunctionPrototypes
  * @{
  */
  
/**
  * @}
  */

/** @defgroup FLASH_Private_Functions
  * @{
  */
static void FLASH_Operate(void)
{
	uint32_t u32Addr = (uint32_t)FLASH_Operate;

	__disable_irq();
	__disable_fault_irq();	
	
	if (IS_FLASH_ADDRESS(u32Addr) || IS_OTP_ADDRESS(u32Addr))
	{
		pfFlash_Done();
	}
	else
	{	
		FCU->CS = 1;
	    while (FCU->CS & 1);	
	}
	
	__enable_fault_irq();
	__enable_irq();	
}

/**
  * @brief  Sets the code latency value.
  * @note   This function can be used for all STM32F10x devices.
  * @param  FLASH_Latency: specifies the FLASH Latency value.
  *   This parameter can be one of the following values:
  *     @arg FLASH_Latency_0: FLASH Zero Latency cycle
  *     @arg FLASH_Latency_1: FLASH One Latency cycle
  *     @arg FLASH_Latency_2: FLASH Two Latency cycles
  * @retval None
  */
void FLASH_SetLatency(uint32_t u32UsClk)
{
	if (0 == u32UsClk)
	{
		FCU->TIM = (SYSCTRL->HCLK_1MS_VAL + 1000 - 1) / 1000;
	}
	else
	{
		FCU->TIM = u32UsClk;
	}
}


/**
  * @brief  Unlocks the FLASH Program Erase Controller.
  * @note   This function can be used for all STM32F10x devices.
  *         - For STM32F10X_XL devices this function unlocks Bank1 and Bank2.
  *         - For all other devices it unlocks Bank1 and it is equivalent 
  *           to FLASH_UnlockBank1 function.. 
  * @param  None
  * @retval None
  */
void FLASH_Unlock(void)
{
	gu32FLASH_Key1 = FLASH_KEY1;
	gu32FLASH_Key2 = FLASH_KEY2;
}

/**
  * @brief  Locks the FLASH Program Erase Controller.
  * @note   This function can be used for all STM32F10x devices.
  *         - For STM32F10X_XL devices this function Locks Bank1 and Bank2.
  *         - For all other devices it Locks Bank1 and it is equivalent 
  *           to FLASH_LockBank1 function.
  * @param  None
  * @retval None
  */
void FLASH_Lock(void)
{
	gu32FLASH_Key1 = ~FLASH_KEY1;
	gu32FLASH_Key2 = ~FLASH_KEY2;
	FCU->PROT = FLASH_KEY2;
}

/**
  * @brief  Erases a specified FLASH page.
  * @note   This function can be used for all STM32F10x devices.
  * @param  Page_Address: The page address to be erased.
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_ErasePage(uint32_t u32Addr)
{
	uint32_t u32RO;
	FLASH_Status status = FLASH_COMPLETE;
	
	/* Check the parameters */
	assert_param(IS_FLASH_ADDRESS(u32Addr));

	u32RO = 1UL << (((u32Addr - FLASH_BASE) >> 12) >> 3);
	if (0 != (FCU->RO & u32RO))
	{
		return FLASH_ERROR_WRP;
	}
	
    FCU->ADDR = (uint32_t *)u32Addr;
    FCU->CFG = OP_SEL_PAGE_ERASE;

	FCU->PROT = gu32FLASH_Key1;
	FCU->PROT = gu32FLASH_Key2;

    FLASH_Operate();   /* Start operate */

	if (0 != (FCU->CS & 0x02))
	{
		status = FLASH_ERROR_PG;
	}

	return status;
}


/**
  * @brief  Erases all FLASH pages.
  * @note   This function can be used for all STM32F10x devices.
  * @param  None
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_EraseAllPages(void)
{
	FLASH_Status status = FLASH_COMPLETE;

	if (0 != FCU->RO)
	{
		return FLASH_ERROR_WRP;
	}

	FCU->ADDR = (__IO uint32_t *)FLASH_BASE;
    FCU->CFG = OP_SEL_ERASE_ALL;

	FCU->PROT = gu32FLASH_Key1;
	FCU->PROT = gu32FLASH_Key2;

    FLASH_Operate();  /* Start operate */

	if (0 != (FCU->CS & 0x02))
	{
		status = FLASH_ERROR_PG;
	}

	return status;
}


/**
  * @brief  Programs a word at a specified address.
  * @note   This function can be used for all STM32F10x devices.
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT. 
  */
FLASH_Status FLASH_ProgramWord(uint32_t u32Addr, uint32_t u32Data)
{
	uint8_t u8Size;
	uint32_t u32Tmp;
	__IO uint32_t *pu32RO;
	FLASH_Status status = FLASH_COMPLETE;

	assert_param(IS_FLASH_ADDRESS(u32Addr) || IS_OTP_ADDRESS(u32Addr));  /* Check the parameters */
	assert_param(0 == (u32Addr & 0x03));                                 /* Check Word align */

    if (IS_FLASH_ADDRESS(u32Addr))
    {
		pu32RO = &FCU->RO;
		u32Tmp = FLASH_BASE;
		u8Size = 3;
    }
    else
    {
        pu32RO = &FCU->RO_INFO;
		u32Tmp = OTP_BASE;
		u8Size = 0;
    }

	u32Tmp = 1UL << (((u32Addr - u32Tmp) >> 12) >> u8Size);

	if (0 != (*pu32RO & u32Tmp))
	{
		return FLASH_ERROR_WRP;
	}

    FCU->CFG = OP_SEL_PROGRAM;
    FCU->PDATA = u32Data;
    FCU->ADDR = (__IO uint32_t *)u32Addr;

    FCU->PROT = gu32FLASH_Key1;
    FCU->PROT = gu32FLASH_Key2;

    FLASH_Operate();

    if (0 != (FCU->CS & 2) || *(uint32_t *)u32Addr != u32Data)
    {
		status = FLASH_ERROR_PG;
    }

	return status;
}

/**
  * @brief  Write protects the desired pages
  * @note   This function can be used for all STM32F10x devices.
  * @param  FLASH_Pages: specifies the address of the pages to be write protected.
  *   This parameter can be:
  *     @arg For @b STM32_Low-density_devices: value between FLASH_WRProt_Pages0to3 and FLASH_WRProt_Pages28to31  
  *     @arg For @b STM32_Medium-density_devices: value between FLASH_WRProt_Pages0to3
  *       and FLASH_WRProt_Pages124to127
  *     @arg For @b STM32_High-density_devices: value between FLASH_WRProt_Pages0to1 and
  *       FLASH_WRProt_Pages60to61 or FLASH_WRProt_Pages62to255
  *     @arg For @b STM32_Connectivity_line_devices: value between FLASH_WRProt_Pages0to1 and
  *       FLASH_WRProt_Pages60to61 or FLASH_WRProt_Pages62to127    
  *     @arg For @b STM32_XL-density_devices: value between FLASH_WRProt_Pages0to1 and
  *       FLASH_WRProt_Pages60to61 or FLASH_WRProt_Pages62to511
  *     @arg FLASH_WRProt_AllPages
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *         FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
void FLASH_Protect(uint32_t u32Addr)
{
	uint8_t u8Size;
	uint32_t u32Tmp;
	__IO uint32_t *pu32RO;
	
    if (IS_FLASH_ADDRESS(u32Addr))
    {
		pu32RO = &FCU->RO;
		u32Tmp = FLASH_BASE;
		u8Size = 3;
    }
	else if (IS_OTP_ADDRESS(u32Addr))
    {
        pu32RO = &FCU->RO_INFO;
		u32Tmp = OTP_BASE;
		u8Size = 0;
    }
	else
	{
		FCU->RO = ~0UL;
		FCU->RO_INFO = ~0UL;
		return;
	}

	u32Tmp = 1UL << (((u32Addr - u32Tmp) >> 12) >> u8Size);;
	*pu32RO |= u32Tmp;
}

uint32_t FLASH_IsProtect(uint32_t u32Addr)
{
	uint8_t u8Size;
	uint32_t u32Tmp;
	__IO uint32_t *pu32RO;

    if (IS_FLASH_ADDRESS(u32Addr))
    {
		pu32RO = &FCU->RO;
		u32Tmp = FLASH_BASE;
		u8Size = 3;
    }
	else if (IS_OTP_ADDRESS(u32Addr))
    {
        pu32RO = &FCU->RO_INFO;
		u32Tmp = OTP_BASE;
		u8Size = 0;
    }
	else
	{
		return 0;
	}

	u32Tmp = 1UL << (((u32Addr - u32Tmp) >> 12) >> u8Size);

	return (0 != (*pu32RO & u32Tmp));
}

void FLASH_SetProtect(uint8_t u8Type, uint32_t u32RO)
{
	__IO uint32_t *pu32RO;
	
    if (0 == u8Type)
    {
		pu32RO = &FCU->RO;
    }
	else
	{
		pu32RO = &FCU->RO_INFO;
	}

	*pu32RO |= u32RO;
}

uint32_t FLASH_GetProtect(uint8_t u8Type)
{
    if (0 == u8Type)
    {
		return FCU->RO;
    }

	return FCU->RO_INFO;
}

void FLASH_SetProtectLock(uint8_t u8Type, uint32_t u32Lock)
{
	__IO uint32_t *pu32RO;
	__IO uint32_t *pu32ROL;
	
    if (0 == u8Type)
    {
		pu32RO = &FCU->RO;
		pu32ROL = &FCU->ROL;
    }
	else
	{
		pu32RO = &FCU->RO_INFO;
		pu32ROL = &FCU->ROL_INFO;
	}

	*pu32RO |= u32Lock;
	*pu32ROL |= u32Lock;
}


uint32_t FLASH_GetProtectLock(uint8_t u8Type)
{
	if (0 == u8Type)
    {
		return FCU->ROL;
    }

	return FCU->ROL_INFO;
}

void FLASH_UnProtect(uint32_t u32Addr)
{
	uint8_t u8Size;
	uint32_t u32Tmp;
	__IO uint32_t *pu32RO;

    if (IS_FLASH_ADDRESS(u32Addr))
    {
		pu32RO = &FCU->RO;
		u32Tmp = FLASH_BASE;
		u8Size = 3;
    }
	else if (IS_OTP_ADDRESS(u32Addr))
    {
        pu32RO = &FCU->RO_INFO;
		u32Tmp = OTP_BASE;
		u8Size = 0;
    }
	else
	{
		return;
	}

	u32Tmp = 1UL << (((u32Addr - u32Tmp) >> 12) >> u8Size);
	*pu32RO &= ~u32Tmp;
}

void FLASH_SetUnProtect(uint8_t u8Type, uint32_t u32RO)
{
	__IO uint32_t *pu32RO;
	
    if (0 == u8Type)
    {
		pu32RO = &FCU->RO;
    }
	else
	{
		pu32RO = &FCU->RO_INFO;
	}

	*pu32RO &= ~u32RO;
}

void FLASH_ProtectLock(uint32_t u32Addr)
{
	uint8_t u8Size;
	uint32_t u32Tmp;
	__IO uint32_t *pu32RO;
	__IO uint32_t *pu32ROL;
	
    if (IS_FLASH_ADDRESS(u32Addr))
    {
		pu32RO = &FCU->RO;
		pu32ROL = &FCU->ROL;
		u32Tmp = FLASH_BASE;
		u8Size = 3;
    }
	else if (IS_OTP_ADDRESS(u32Addr))
    {
        pu32RO = &FCU->RO_INFO;
		pu32ROL = &FCU->ROL_INFO;
		u32Tmp = OTP_BASE;
		u8Size = 0;
    }
	else
	{
		FCU->RO = ~0UL;
		FCU->ROL = ~0UL;
		FCU->RO_INFO = ~0UL;
		FCU->ROL_INFO = ~0UL;
		return;
	}

	u32Tmp = 1UL << (((u32Addr - u32Tmp) >> 12) >> u8Size);
	*pu32RO |= u32Tmp;
	*pu32ROL |= u32Tmp;
}

uint32_t FLASH_IsProtectLock(uint32_t u32Addr)
{
	uint8_t u8Size;
	uint32_t u32Tmp;
	__IO uint32_t *pu32RO;
	__IO uint32_t *pu32ROL;
	
    if (IS_FLASH_ADDRESS(u32Addr))
    {
		pu32RO = &FCU->RO;
		pu32ROL = &FCU->ROL;
		u32Tmp = FLASH_BASE;
		u8Size = 3;
    }
	else if (IS_OTP_ADDRESS(u32Addr))
    {
        pu32RO = &FCU->RO_INFO;
		pu32ROL = &FCU->ROL_INFO;
		u32Tmp = OTP_BASE;
		u8Size = 0;
    }
	else
	{
		return 0;
	}

	u32Tmp = 1UL << (((u32Addr - u32Tmp) >> 12) >> u8Size);;
	*pu32RO |= u32Tmp;
	*pu32ROL |= u32Tmp;

	return (0 != (*pu32ROL & u32Tmp));
}

FLASH_Status FLASH_WriteBlock(uint32_t u32Addr, void *pvBuf, uint32_t len)
{
	uint8_t j, u8Size;
	uint32_t i, u32Tmp, u32Base;
    uint8_t *pu8Start = pvBuf, *pu8End = (uint8_t *)pvBuf + len;
	uint32_t volatile *pu32Addr = (uint32_t volatile *)u32Addr;
    __IO uint32_t *pRO;

	if (0 == len)
	{
		return FLASH_COMPLETE;
	}
    
	assert_param((IS_FLASH_ADDRESS(u32Addr) && IS_FLASH_ADDRESS(u32Addr + len - 1)) || 
				 (IS_OTP_ADDRESS(u32Addr) && IS_OTP_ADDRESS(u32Addr + len - 1)));
	assert_param(0 == (u32Addr & 0x03));
	
    if (IS_OTP_ADDRESS(u32Addr))
    {
        pRO = &FCU->RO_INFO;
		u32Base = OTP_BASE;
		u8Size = 0;
    }
    else
    {
		pRO = &FCU->RO;
		u32Base = FLASH_BASE;
		u8Size = 3;
    }

	for (i = u32Addr; i < u32Addr + len; i += (0x1000 << u8Size))
	{
		u32Tmp = 1UL << (((i - u32Base) >> 12) >> u8Size);
		if (0 != (*pRO & u32Tmp))
		{
			return FLASH_ERROR_WRP;
		}
	}

    //clear erro flag.
    FCU->CS = 0;
	FCU->ADDR = pu32Addr;
	FCU->PDATA = *pu32Addr;
	
    while (pu8Start < pu8End)
    {
		i = pu8End - pu8Start;
		i = i > 4 ? 4 : i;
		u32Tmp = 0;
		for ( j = 0; j < i; j++)
		{
			u32Tmp |= (pu8Start[j] << (j << 3));
		}
		pu8Start += 4;

		while (FCU->CS & 1);
		if (FCU->PDATA != *FCU->ADDR || 0 != (FCU->CS & 0x02))
		{
			return FLASH_ERROR_PG;
		}
        FCU->CFG = OP_SEL_PROGRAM;
        FCU->PDATA = u32Tmp;
        FCU->ADDR = pu32Addr++;

        FCU->PROT = gu32FLASH_Key1;
        FCU->PROT = gu32FLASH_Key2;
        FLASH_Operate();
    }

	if (FCU->PDATA != *FCU->ADDR || 0 != (FCU->CS & 0x02))
	{
		return FLASH_ERROR_PG;
	}

    return FLASH_COMPLETE;
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
