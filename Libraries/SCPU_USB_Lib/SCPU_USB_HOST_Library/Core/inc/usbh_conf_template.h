/************************ (C) COPYRIGHT Megahuntmicro *************************
 * File Name            : usbh_conf_template.h
 * Author               : Megahuntmicro
 * Version              : V1.0.0
 * Date                 : 21-October-2014
 * Description          : General USB Host library configuration.
 *****************************************************************************/
 
 
#ifndef __USBH_CONF__H__
#define __USBH_CONF__H__
 
#ifdef __cplusplus
extern "C" {
#endif
	
/* Include ------------------------------------------------------------------*/	
/* Exported types -----------------------------------------------------------*/
/* Exported constants -------------------------------------------------------*/	
/* Exported macro -----------------------------------------------------------*/	
/* Exported functions -------------------------------------------------------*/	
/* Exported variables -------------------------------------------------------*/	
/* Define to prevent recursive inclusion -------------------------------------*/


/* Includes ------------------------------------------------------------------*/

/** @addtogroup USBH_OTG_DRIVER
  * @{
  */
  
/** @defgroup USBH_CONF
  * @brief usb otg low level driver configuration file
  * @{
  */ 

/** @defgroup USBH_CONF_Exported_Defines
  * @{
  */ 

#define USBH_MAX_NUM_ENDPOINTS                2
#define USBH_MAX_NUM_INTERFACES               2
#ifdef USE_USB_OTG_FS 
#define USBH_MSC_MPS_SIZE                 0x40
#else
#define USBH_MSC_MPS_SIZE                 0x200
#endif

/**
  * @}
  */ 


/** @defgroup USBH_CONF_Exported_Types
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USBH_CONF_Exported_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USBH_CONF_Exported_Variables
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USBH_CONF_Exported_FunctionsPrototype
  * @{
  */ 
/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif	
#endif /* __USBH_CONF__H__ */

/************************ (C) COPYRIGHT 2014 Megahuntmicro ****END OF FILE****/
