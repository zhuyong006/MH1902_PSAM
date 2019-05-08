/************************ (C) COPYRIGHT Megahuntmicro *************************
 * File Name            : usbd_conf_template.h
 * Author               : Megahuntmicro
 * Version              : V1.0.0
 * Date                 : 21-October-2014
 * Description          : usb device configuration template file.
 *****************************************************************************/
 
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_CONF__H__
#define __USBD_CONF__H__
 
#ifdef __cplusplus
extern "C" {
#endif


/* Includes ------------------------------------------------------------------*/
#include "usb_conf.h"

/** @defgroup USB_CONF_Exported_Defines
  * @{
  */ 
#define USE_USB_OTG_HS  

#define USBD_CFG_MAX_NUM           1
#define USB_MAX_STR_DESC_SIZ       64 
#define USBD_EP0_MAX_PACKET_SIZE   64

/**
  * @}
  */ 


/** @defgroup USB_CONF_Exported_Types
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup USB_CONF_Exported_Macros
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USB_CONF_Exported_Variables
  * @{
  */ 
/**
  * @}
  */ 

/** @defgroup USB_CONF_Exported_FunctionsPrototype
  * @{
  */ 
/**
  * @}
  */ 


#endif /* __USBD_CONF__H__ */

/************************ (C) COPYRIGHT 2014 Megahuntmicro ****END OF FILE****/
