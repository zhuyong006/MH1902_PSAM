/**
  ******************************************************************************
  * @file    mhscpu_img_cop.c
  * @author  Megahunt Software Team
  * @version V1.0.0
  * @date    25-August-2016
  * @brief   
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "mhscpu_img_cop.h"
#include "mhscpu_sysctrl.h"
/** @addtogroup app_Lib
  * @{
  */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Set IMG_COP registers as default value.
  * @param  img_cop: IMG_COP model address. 
  *   For @b this parameter can be any combination
  *   of the following values:        
  *     @arg IMG_COP
  * @retval None
  */
void IMG_COP_DeInit(IMG_COP_TypeDef *img_cop)
{
	/* set register to default value */
	img_cop->IMG_COP_CONFIG = 0x00;
	img_cop->INTC = 0x00;
	img_cop->IMG_ADDR = 0x00;
	img_cop->IMG_SIZE = 0x00;
	img_cop->INTGIMG_ADDR = 0x00;
	img_cop->BINIMG_ADDR = 0x00;
	img_cop->WINDOW_ADDR = 0x00;
	img_cop->SEARCH_PIXEL = 0x00;
	img_cop->SEARCH_MAX = 0x00;
}

/**
  * @brief  Initialization IMG_COP_Structure to default value.
  * @param  IMG_COP_Structure: Initialization structure start address.
  * @retval None
  */
void IMG_COP_StructDeInit(IMG_COP_Structure_TypeDef *IMG_COP_Structure)
{
	IMG_COP_Structure->original_image_addr = 0x00;
	IMG_COP_Structure->original_row_width = 0x00;
	IMG_COP_Structure->original_col_width = 0x00;
	IMG_COP_Structure->integration_image_addr = 0x00;
	IMG_COP_Structure->binary_image_addr = 0x00;
	IMG_COP_Structure->window_row_width = 0x00;
	IMG_COP_Structure->window_col_width = 0x00;
	IMG_COP_Structure->search_start_x = 0x00;
	IMG_COP_Structure->search_start_y = 0x00;
	/* Connected domain */
	IMG_COP_Structure->max_pixel_cnt = 0x00;
	IMG_COP_Structure->sum_gray_x = 0x00;
	IMG_COP_Structure->sum_gray_y = 0x00;
	IMG_COP_Structure->sum_gray = 0x00;
	IMG_COP_Structure->search_count = 0x00;
}

/**
  * @brief  IMG_COP Initialization 
  * @param  img_cop: IMG_COP model address. 
  *   For @b this parameter can be any combination
  *   of the following values:        
  *     @arg IMG_COP
  * @param  IMG_COP_Structure: Initialization structure start address.
  * @retval None
  */
void IMG_COP_Init(IMG_COP_TypeDef *img_cop, 
				  IMG_COP_Structure_TypeDef *IMG_COP_Structure)
{
	IMG_COP_CONFIG_TypeDef img_cop_config;
	/* Reset IMG_COP */
	SYSCTRL_APBPeriphResetCmd(SYSCTRL_APBPeriph_IMG_COP, ENABLE);
	img_cop_config.d32 = 0x00;
//	img_cop_config.b.kn_factor = (uint16_t)(((MULBASE * K) / N) + 0.5);
	img_cop_config.b.kn_factor = (uint16_t)(((MULBASE * K) / N));
	img_cop->IMG_COP_CONFIG = img_cop_config.d32;
	img_cop->IMG_ADDR = IMG_COP_Structure->original_image_addr;
	img_cop->IMG_SIZE = IMG_COP_Structure->original_row_width;
	img_cop->IMG_SIZE |= ((uint32_t)IMG_COP_Structure->original_col_width << 16);
	
	img_cop->INTGIMG_ADDR = IMG_COP_Structure->integration_image_addr;
	
	img_cop->BINIMG_ADDR = IMG_COP_Structure->binary_image_addr;
	img_cop->WINDOW_ADDR = IMG_COP_Structure->window_col_width;
	img_cop->WINDOW_ADDR |= ((uint16_t)IMG_COP_Structure->window_row_width << 8);
	
	img_cop->SEARCH_PIXEL = IMG_COP_Structure->search_start_x;
	img_cop->SEARCH_PIXEL |= ((uint32_t)IMG_COP_Structure->search_start_y << 16);
	img_cop->SEARCH_MAX = IMG_COP_Structure->max_pixel_cnt;
}

/**
  * @brief  IMG_COP Initialization 
  * @param  img_cop: IMG_COP model address. 
  *   For @b this parameter can be any combination
  *   of the following values:        
  *     @arg IMG_COP
  * @param  IMG_COP_Structure: Initialization structure start address.
  * @retval None
  */
void IMG_COP_SearchInit(IMG_COP_TypeDef *img_cop, 
						IMG_COP_Structure_TypeDef *IMG_COP_Structure)
{
	/* Reset IMG_COP */
//	SYSCTRL_APBPeriphResetCmd(SYSCTRL_APBPeriph_IMG_COP, ENABLE);
//	img_cop->IMG_SIZE = IMG_COP_Structure->original_row_width;
//	img_cop->BINIMG_ADDR = IMG_COP_Structure->binary_image_addr;
	img_cop->SEARCH_PIXEL = IMG_COP_Structure->search_start_x;
	img_cop->SEARCH_PIXEL |= ((uint32_t)IMG_COP_Structure->search_start_y << 16);
//	img_cop->SEARCH_MAX = IMG_COP_Structure->max_pixel_cnt;
}

/**
  * @brief  Get the IMG_COP calculation result and saves it to the specified location.
  * @param  img_cop: IMG_COP model address. 
  *   For @b this parameter can be any combination
  *   of the following values:        
  *     @arg IMG_COP
  * @param  IMG_COP_Structure: Initialization structure start address.
  * @retval None
  */
void IMG_COP_ReadRegionGenResultToStruct(IMG_COP_TypeDef *img_cop, 
										 IMG_COP_Structure_TypeDef *IMG_COP_Structure)
{
	IMG_COP_Structure->sum_gray_x = img_cop->SUM_X;
	IMG_COP_Structure->sum_gray_y = img_cop->SUM_Y;
	IMG_COP_Structure->sum_gray = img_cop->SUM_TOTAL;
	IMG_COP_Structure->search_count = img_cop->SEARCH_COUNT;
}

/**
  * @brief  Enable or Disable IMG_COP Interrupt.
  * @param  img_cop: IMG_COP model address. 
  *   For @b this parameter can be any combination
  *   of the following values:        
  *     @arg IMG_COP
  * @param  int_flag: interrupt flag
  *   For @b this parameter can be any combination
  *   of the following values:        
  *     @arg INTEGRATION_INT
  *     @arg BINARY_INT
  *     @arg SEARCH_INT
  * @param  NewState: new state of the specified peripheral clock.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void IMG_COP_INT_Cmd(IMG_COP_TypeDef *img_cop, uint8_t int_falg, FunctionalState NewState)
{
	assert_param(IS_IMG_COP_INT(int_falg));
	if(DISABLE != NewState)
	{
		img_cop->INTC &= ~int_falg;
	}
	else
	{
		img_cop->INTC |= int_falg ;
	}
}

/**
  * @brief  Enable or Disable IMG_COP calculation.
  * @param  img_cop: IMG_COP model address. 
  *   For @b this parameter can be any combination
  *   of the following values:        
  *     @arg IMG_COP
  * @param  cac_fun: IMG_COP calculation function number
  *   For @b this parameter can be any combination
  *   of the following values:        
  *     @arg BINARY_FORCE_STOP
  *     @arg SEARCH_AUTOSTART
  *     @arg SEARCH_FORCE_STOP
  * @param  NewState: new state of the specified peripheral clock.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void IMG_COP_CacCmd(IMG_COP_TypeDef *img_cop, uint8_t cac_fun, FunctionalState NewState)
{
	assert_param(IS_IMG_COP_CONFIG_PARAM(cac_fun));
	if(DISABLE != NewState)
	{
		img_cop->IMG_COP_CONFIG |= (cac_fun);
	}
	else
	{
		img_cop->INTC &= ~cac_fun;
	}
}

/**
  * @brief  Start Binary calculation.
  * @param  img_cop: IMG_COP model address. 
  *   For @b this parameter can be any combination
  *   of the following values:        
  *     @arg IMG_COP
  * @retval None
  */
void IMG_COP_BinaryStart(IMG_COP_TypeDef *img_cop)
{
	img_cop->IMG_COP_CONFIG |= BINARY_START;
}

/**
  * @brief  Start srarch calculation.
  * @param  img_cop: IMG_COP model address. 
  *   For @b this parameter can be any combination
  *   of the following values:        
  *     @arg IMG_COP
  * @retval None
  */
void IMG_COP_SearchStart(IMG_COP_TypeDef *img_cop)
{
	img_cop->IMG_COP_CONFIG |= SEARCH_START;
}

/**
  * @brief  Get IMG_COP Interrupt status value.
  * @param  img_cop: IMG_COP model address. 
  *   For @b this parameter can be any combination
  *   of the following values:        
  *     @arg IMG_COP
  * @param  int_flag: interrupt flag
  *   For @b this parameter can be any combination
  *   of the following values:        
  *     @arg INTEGRATION_INT
  *     @arg BINARY_INT
  *     @arg SEARCH_INT
  * @retval None
  */
uint8_t IMG_COP_GetIntFlag(IMG_COP_TypeDef *img_cop, uint8_t int_falg)
{
	uint8_t bitstatus = 0x00;
	assert_param(IS_IMG_COP_INT(int_falg));
	if (img_cop->STATUS & int_falg)
	{
		bitstatus = (uint8_t)Bit_SET;
	}else{
		bitstatus = (uint8_t)Bit_RESET;
	}
	return bitstatus;
}

/**
  * @brief  Clear all IMG_COP Interrupt status.
  * @param  img_cop: IMG_COP model address. 
  *   For @b this parameter can be any combination
  *   of the following values:        
  *     @arg IMG_COP
  * @retval None
  */
void IMG_COP_ClearIntFlag(IMG_COP_TypeDef *img_cop)
{
	img_cop->INTC = INT_CLEAR;
}

/**
  * @brief  Get the IMG_COP calculation SUN_X result and return it.
  * @param  img_cop: IMG_COP model address. 
  *   For @b this parameter can be any combination
  *   of the following values:        
  *     @arg IMG_COP
  * @retval None
  */
uint32_t IMG_COP_GetSUM_X(IMG_COP_TypeDef *img_cop)
{
	return (img_cop->SUM_X);
}

/**
  * @brief  Get the IMG_COP calculation SUN_Y result and return it.
  * @param  img_cop: IMG_COP model address. 
  *   For @b this parameter can be any combination
  *   of the following values:        
  *     @arg IMG_COP
  * @retval None
  */
uint32_t IMG_COP_GetSUM_Y(IMG_COP_TypeDef *img_cop)
{
	return (img_cop->SUM_Y);
}

/**
  * @brief  Get the IMG_COP calculation SUM_TOTAL result and return it.
  * @param  img_cop: IMG_COP model address. 
  *   For @b this parameter can be any combination
  *   of the following values:        
  *     @arg IMG_COP
  * @retval None
  */
uint32_t IMG_COP_GetSUM_TOTAL(IMG_COP_TypeDef *img_cop)
{
	return (img_cop->SUM_TOTAL);
}

/**
  * @brief  Get the IMG_COP calculation SEARCH_COUNT result and return it.
  * @param  img_cop: IMG_COP model address. 
  *   For @b this parameter can be any combination
  *   of the following values:        
  *     @arg IMG_COP
  * @retval None
  */
uint32_t IMG_COP_GetSEARCH_COUNT(IMG_COP_TypeDef *img_cop)
{
	return (img_cop->SEARCH_COUNT);
}
/************************ (C) COPYRIGHT LW EmbeddedTeam ******END OF FILE******/
