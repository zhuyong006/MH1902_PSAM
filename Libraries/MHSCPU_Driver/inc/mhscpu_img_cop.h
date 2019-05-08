/************************ (C) COPYRIGHT Megahuntmicro *************************
 * File Name            : mhscpu_img_cop.h
 * Author               : Megahuntmicro
 * Version              : V1.0.0
 * Date                 : 07/29/2014
 * Description          : Header file for mhscpu_img_cop.c file
 *****************************************************************************/
 
 
#ifndef __MHSCPU_IMG_COP_H__
#define __MHSCPU_IMG_COP_H__
 
 
#ifdef __cplusplus
extern "C" {
#endif
	
/* Include ------------------------------------------------------------------*/
#include "mhscpu.h"	
/* Exported types -----------------------------------------------------------*/
/* Exported constants -------------------------------------------------------*/	
/* Exported macro -----------------------------------------------------------*/	
/* Exported functions -------------------------------------------------------*/	
/* Exported variables -------------------------------------------------------*/	
#define K									0.95
#define N									81			// 9*9	
#define MULBASE								0x10000		// 65536
	
#define BINARY_START	   					(0x01)	
#define BINARY_FORCE_STOP  					(0x02)
#define SEARCH_START 						(0x04)
#define SEARCH_AUTOSTART 					(0x08)
#define SEARCH_FORCE_STOP 					(0x10)
#define IS_IMG_COP_CONFIG_PARAM(para)		(((para) | BINARY_FORCE_STOP) || \
											 ((para) | SEARCH_AUTOSTART) || \
											 ((para) | SEARCH_FORCE_STOP))
#define INT_STATUS							(0x01)
#define INT_CLEAR							(0x01)	
#define INTEGRATION_INT 					(0x02)
#define BINARY_INT 							(0x04)
#define SEARCH_INT 							(0x08)
#define IS_IMG_COP_INT(intNum)				(((intNum) | INTEGRATION_INT) || \
											 ((intNum) | BINARY_INT) || \
											 ((intNum) | SEARCH_INT))
	
typedef struct _IMG_COP_Structure_TypeDef
{
	uint32_t  original_image_addr;
	uint16_t  original_row_width;
	uint16_t  original_col_width;
	uint32_t  integration_image_addr;
	uint32_t  binary_image_addr;
	uint8_t   window_row_width;
	uint8_t   window_col_width;
	uint16_t  search_start_x;
	uint16_t  search_start_y;
	/* Connected domain */
	uint16_t  max_pixel_cnt;
	
	uint32_t  sum_gray_x;
	uint32_t  sum_gray_y;
	uint32_t  sum_gray;
	uint32_t  search_count;
} IMG_COP_Structure_TypeDef; 	
	
	
/**
  * IMG_COP_CONFIG register bitend definition
  */
typedef union _IMG_COP_CONFIG_TypeDef
{
	uint32_t d32;
	struct{
		uint32_t binary_start			: 1;
		uint32_t binary_force_stop		: 1;
		uint32_t search_start			: 1;
		uint32_t search_autostart		: 1;
		uint32_t search_force_stop		: 1;
		uint32_t reserved5_15			: 11;
		uint32_t kn_factor				: 16;
	} b;
} IMG_COP_CONFIG_TypeDef;

/**
  * STATUS register bitend definition
  */
typedef union _STATUS_TypeDef
{
	uint32_t d32;
	struct{
		uint32_t int_status				: 1;
		uint32_t integration_done		: 1;
		uint32_t binary_done			: 1;
		uint32_t search_done			: 1;
		uint32_t reserved4_15			: 12;
		uint32_t core_status			: 16;
	} b;
} STATUS_TypeDef;
	
/**
  * INTC register bitend definition
  */
typedef union _INTC_TypeDef
{
	uint32_t d32;
	struct{
		uint32_t int_clear				: 1;
		uint32_t integration_int_mask	: 1;
		uint32_t binary_int_mask		: 1;
		uint32_t search_int_mask		: 1;
		uint32_t reserved4_31			: 28;
	} b;
} INTC_TypeDef;

/**
  * IMG_SIZE register bitend definition
  */
typedef union _IMG_SIZE_TypeDef
{
	uint32_t d32;
	struct{
		uint32_t h_size					: 16;
		uint32_t v_size					: 16;
	} b;
} IMG_SIZE_TypeDef;

/**
  * WINDOW_ADDR register bitend definition
  */
typedef union _WINDOW_ADDR_TypeDef
{
	uint32_t d32;
	struct{
		uint32_t window_y				: 8;
		uint32_t window_x				: 8;
		uint32_t reserved16_31			: 16;
	} b;
} WINDOW_ADDR_TypeDef;


/**
  * SEARCH_PIXEL register bitend definition
  */
typedef union _SEARCH_PIXEL_TypeDef
{
	uint32_t d32;
	struct{
		uint32_t search_start_x			: 16;
		uint32_t search_start_y			: 16;
	} b;
} SEARCH_PIXEL_TypeDef;

/**
  * SEARCH_MAX register bitend definition
  */
typedef union _SEARCH_MAX_TypeDef
{
	uint32_t d32;
	struct{
		uint32_t search_max				: 16;
		uint32_t reserved16_31			: 16;
	} b;
} SEARCH_MAX_TypeDef;

/********************* System control APIs ***************************************/
void IMG_COP_DeInit(IMG_COP_TypeDef *img_cop);
void IMG_COP_StructDeInit(IMG_COP_Structure_TypeDef *IMG_COP_Structure);
void IMG_COP_Init(IMG_COP_TypeDef *img_cop, 
				  IMG_COP_Structure_TypeDef *IMG_COP_Structure);
void IMG_COP_SearchInit(IMG_COP_TypeDef *img_cop, 
						IMG_COP_Structure_TypeDef *IMG_COP_Structure);
void IMG_COP_ReadRegionGenResultToStruct(IMG_COP_TypeDef *img_cop, 
										 IMG_COP_Structure_TypeDef *IMG_COP_Structure);
void IMG_COP_INT_Cmd(IMG_COP_TypeDef *img_cop, uint8_t int_falg, FunctionalState NewState);
void IMG_COP_CacCmd(IMG_COP_TypeDef *img_cop, uint8_t cac_fun, FunctionalState NewState);
void IMG_COP_BinaryStart(IMG_COP_TypeDef *img_cop);
void IMG_COP_SearchStart(IMG_COP_TypeDef *img_cop);
uint8_t IMG_COP_GetIntFlag(IMG_COP_TypeDef *img_cop, uint8_t int_falg);
void IMG_COP_ClearIntFlag(IMG_COP_TypeDef *img_cop);
uint32_t IMG_COP_GetSUM_X(IMG_COP_TypeDef *img_cop);
uint32_t IMG_COP_GetSUM_Y(IMG_COP_TypeDef *img_cop);
uint32_t IMG_COP_GetSUM_TOTAL(IMG_COP_TypeDef *img_cop);
uint32_t IMG_COP_GetSEARCH_COUNT(IMG_COP_TypeDef *img_cop);
#ifdef __cplusplus
}
#endif	 

#endif	/* __MHSCPU_IMG_COP_H__ */
/*********************** (C) COPYRIGHT 2014 Megahuntmicro ****END OF FILE****/
