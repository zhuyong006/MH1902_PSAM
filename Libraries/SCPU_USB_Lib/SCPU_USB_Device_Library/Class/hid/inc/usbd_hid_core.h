/************************ (C) COPYRIGHT Megahuntmicro *************************
 * File Name            : usbd_hid_core.h
 * Author               : Megahuntmicro
 * Version              : V1.0.0
 * Date                 : 21-October-2014
 * Description          : header file for the usbd_hid_core.c file.
 *****************************************************************************/

#ifndef __USB_HID_CORE_H_
#define __USB_HID_CORE_H_

#include  "usbd_ioreq.h"

  
/** @defgroup USBD_HID_Exported_Defines
  * @{
  */
#ifdef HID_SUPPORT_IN_OUT    
#define USB_HID_CONFIG_DESC_SIZ       41
#define HID_MOUSE_REPORT_DESC_SIZE    27
#else
#define USB_HID_CONFIG_DESC_SIZ       34
#define HID_MOUSE_REPORT_DESC_SIZE    74
#endif
#define USB_HID_DESC_SIZ              9


#define HID_DESCRIPTOR_TYPE           0x21
#define HID_REPORT_DESC               0x22

#define HID_REQ_SET_PROTOCOL          0x0B
#define HID_REQ_GET_PROTOCOL          0x03

#define HID_REQ_SET_IDLE              0x0A
#define HID_REQ_GET_IDLE              0x02

#define HID_REQ_SET_REPORT            0x09
#define HID_REQ_GET_REPORT            0x01
/**
  * @}
  */ 

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */ 
extern USBD_Class_cb_TypeDef  USBD_HID_cb;
/**
  * @}
  */ 

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */ 
uint8_t USBD_HID_SendReport (USB_OTG_CORE_HANDLE  *pdev, 
                                 uint8_t *report,
                                 uint16_t len);
/**
  * @}
  */ 

#endif  // __USB_HID_CORE_H_


/************************ (C) COPYRIGHT 2014 Megahuntmicro ****END OF FILE****/
