/************************ (C) COPYRIGHT Megahuntmicro *************************
 * File Name            : usb_hcd_int.c
 * Author               : Megahuntmicro
 * Version              : V1.0.0
 * Date                 : 21-October-2014
 * Description          : Host driver interrupt subroutines.
 *****************************************************************************/
 
/* Include ------------------------------------------------------------------*/
#include "usb_core.h"
#include "usb_defines.h"
#include "usb_hcd_int.h"

/* Private typedef ----------------------------------------------------------*/
/* Private define -----------------------------------------------------------*/	
/* Private macro ------------------------------------------------------------*/	
/* Private variables --------------------------------------------------------*/	
/* Ptivate function prototypes ----------------------------------------------*/	

/******************************************************************************
* Function Name  :
* Description    :
* Input          :
* Output         :
* Return         :
******************************************************************************/
#if defined   (__CC_ARM) /*!< ARM Compiler */
#pragma O0
#elif defined (__GNUC__) /*!< GNU Compiler */
#pragma GCC optimize ("O0")
#elif defined  (__TASKING__) /*!< TASKING Compiler */ 
#pragma optimize=0                          

#endif /* __CC_ARM */

/** @addtogroup USB_OTG_DRIVER
* @{
*/

/** @defgroup USB_HCD_INT 
* @brief This file contains the interrupt subroutines for the Host mode.
* @{
*/


/** @defgroup USB_HCD_INT_Private_Defines
* @{
*/ 
/**
* @}
*/ 


/** @defgroup USB_HCD_INT_Private_TypesDefinitions
* @{
*/ 
/**
* @}
*/ 



/** @defgroup USB_HCD_INT_Private_Macros
* @{
*/ 
/**
* @}
*/ 


/** @defgroup USB_HCD_INT_Private_Variables
* @{
*/ 
/**
* @}
*/ 


/** @defgroup USB_HCD_INT_Private_FunctionPrototypes
* @{
*/ 

static uint32_t USB_OTG_USBH_handle_sof_ISR(USB_OTG_CORE_HANDLE *pdev);
static uint32_t USB_OTG_USBH_handle_Suspend_ISR(USB_OTG_CORE_HANDLE *pdev);
static uint32_t USB_OTG_USBH_handle_hc_ISR (USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost);
static uint32_t USB_OTG_USBH_handle_hc_n_In_ISR (USB_OTG_CORE_HANDLE *pdev ,
                                                 uint32_t ep_num);
static uint32_t USB_OTG_USBH_handle_hc_n_Out_ISR (USB_OTG_CORE_HANDLE *pdev , 
                                                  uint32_t ep_num);
static uint32_t USB_OTG_USBH_handle_SessReq_ISR (USB_OTG_CORE_HANDLE *pdev);
static uint32_t USB_OTG_USBH_Resume_ISR (USB_OTG_CORE_HANDLE *pdev);
static uint32_t USB_OTG_USBH_handle_Babble_ISR (USB_OTG_CORE_HANDLE *pdev);
static uint32_t USB_OTG_USBH_handle_Connect_ISR (USB_OTG_CORE_HANDLE *pdev);
static uint32_t USB_OTG_USBH_handle_Disconnect_ISR (USB_OTG_CORE_HANDLE *pdev);
static uint32_t USB_OTG_USBH_handle_VBusError_ISR (USB_OTG_CORE_HANDLE *pdev);

/**
* @}
*/ 


/** @defgroup USB_HCD_INT_Private_Functions
* @{
*/ 

/**
* @brief  HOST_Handle_ISR 
*         This function handles all USB Host Interrupts
* @param  pdev: Selected device
* @retval status 
*/

uint32_t USBH_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost)
{
    uint32_t retval = 0;
    USB_OTG_INTRUSB_TypeDef intr_usb;
    USB_OTG_INTRRX_TypeDef  intr_rx;
    USB_OTG_INTRTX_TypeDef  intr_tx;
    USB_OTG_DEVCTL_TypeDef devctl;
    
    devctl.d8 = USB_OTG_READ_REG8(&pdev->regs.DYNFIFOREGS->DEVCTL);
    intr_usb.d8 = USB_OTG_READ_REG8(&pdev->regs.COMMREGS->INTRUSB);
    /* Disconnect interrupt */
    if (intr_usb.b.discon)
    {
        devctl.b.session = 0;
        USB_OTG_WRITE_REG8(&pdev->regs.DYNFIFOREGS->DEVCTL, devctl.d8);
        /* Reset USB */
        USB_OTG_WRITE_REG8(&pdev->regs.COMMREGS->FADDR, 0x00);
        retval |= USB_OTG_USBH_handle_Disconnect_ISR (pdev);  
        devctl.b.session = 1;
        USB_OTG_WRITE_REG8(&pdev->regs.DYNFIFOREGS->DEVCTL, devctl.d8); 
    }
	/* VBus Error interrupt */
    if (intr_usb.b.VBus_error)
    {
		devctl.b.session = 1;
        USB_OTG_WRITE_REG8(&pdev->regs.DYNFIFOREGS->DEVCTL, devctl.d8); 
        USB_OTG_USBH_handle_VBusError_ISR (pdev);
    }
    /* Check if HOST Mode */
    if (USB_OTG_IsHostMode(pdev))
    {
        intr_rx.d16 = USB_OTG_READ_REG16(&pdev->regs.COMMREGS->INTRRX);
        intr_tx.d16 = USB_OTG_READ_REG16(&pdev->regs.COMMREGS->INTRTX);
        /**
          * Gloable Interrupt 
          */
        /* Suspend Interrupt. Only valid in Peripheral mode. */
        if (intr_usb.b.suspend)
        {
            USB_OTG_USBH_handle_Suspend_ISR (pdev);
        }
        /* Resume interrupt */
        if (intr_usb.b.resume)
        {
            USB_OTG_USBH_Resume_ISR (pdev);
        }
        /* Reset/Babble interrupt */
        if (intr_usb.b.reset_babble)
        {
            USB_OTG_USBH_handle_Babble_ISR (pdev);
        }
        /* SOF interrupt */
        if (intr_usb.b.sof)
        {
            retval |= USB_OTG_USBH_handle_sof_ISR (pdev);
        }
        /* Connect interrupt */
        if (intr_usb.b.conn)
        {
			devctl.b.session = 1;
            USB_OTG_WRITE_REG8(&pdev->regs.DYNFIFOREGS->DEVCTL, devctl.d8);
            retval |= USB_OTG_USBH_handle_Connect_ISR (pdev);  
        }
        /* Session Request interrupt */
        if (intr_usb.b.sess_req)
        {
            USB_OTG_USBH_handle_SessReq_ISR (pdev);
        }

        /**
          * Tx/Rx Endpoint Interrupt 
          */
        if (intr_tx.b.EP0_intp)
        {
            USB_OTG_USBH_handle_hc_ISR(pdev, phost);
        }
        if (intr_tx.d16 & 0xFFFE)
        {
            if (intr_tx.b.EP1_tx_intp)
                USB_OTG_USBH_handle_hc_n_Out_ISR(pdev, 1);
            if (intr_tx.b.EP2_tx_intp)
                USB_OTG_USBH_handle_hc_n_Out_ISR(pdev, 2);
            if (intr_tx.b.EP3_tx_intp)
                USB_OTG_USBH_handle_hc_n_Out_ISR(pdev, 3);
            if (intr_tx.b.EP4_tx_intp)
                USB_OTG_USBH_handle_hc_n_Out_ISR(pdev, 4);
        }
        if (intr_rx.d16 & 0xFFFE)
        {
            if (intr_rx.b.EP1_rx_intp)
                USB_OTG_USBH_handle_hc_n_In_ISR(pdev, 1);
            if (intr_rx.b.EP2_rx_intp)
                USB_OTG_USBH_handle_hc_n_In_ISR(pdev, 2);
            if (intr_rx.b.EP3_rx_intp)
                USB_OTG_USBH_handle_hc_n_In_ISR(pdev, 3);
            if (intr_rx.b.EP4_rx_intp)
                USB_OTG_USBH_handle_hc_n_In_ISR(pdev, 4);
        }
    }
    return retval;
}

/**
* @brief  USB_OTG_USBH_handle_hc_ISR 
*         This function indicates that one or more host channels has a pending
* @param  pdev: Selected device
* @retval status 
*/
static uint32_t USB_OTG_USBH_handle_hc_ISR (USB_OTG_CORE_HANDLE *pdev, USBH_HOST *phost)
{
    USB_OTG_CSR0L_IN_HOST_TypeDef csr0l;
    USB_OTG_CSR0H_IN_HOST_TypeDef csr0h;
    uint32_t retval = 0;
    uint8_t count_len;
    
    csr0l.d8 = USB_OTG_READ_REG8(&pdev->regs.INDEXREGS->CSRL.CSR0L); 
    csr0h.d8 = USB_OTG_READ_REG8(&pdev->regs.INDEXREGS->CSRH.CSR0H); 
    
    if (csr0l.b.rx_stall)
    {
        pdev->host.URB_State[0] = URB_STALL;  
        csr0l.b.rx_stall = 0;
        USB_OTG_WRITE_REG8(&pdev->regs.INDEXREGS->CSRL.CSR0L, csr0l.d8);
    }
    if (csr0l.b.error)
    {
        pdev->host.URB_State[0] = URB_ERROR;
        csr0l.b.error = 0;
        USB_OTG_WRITE_REG8(&pdev->regs.INDEXREGS->CSRL.CSR0L, csr0l.d8);
    }
    if (csr0l.b.nak_timeout)
    {
        pdev->host.URB_State[0] = URB_ERROR;
        csr0l.b.nak_timeout = 0;
        USB_OTG_WRITE_REG8(&pdev->regs.INDEXREGS->CSRL.CSR0L, csr0l.d8);
    }
    /* Setup Stage */
    if (phost->Control.state == CTRL_SETUP_WAIT)
    {
        pdev->host.URB_State[0] = URB_DONE;
        csr0l.b.setup_pkt = 0;
        USB_OTG_WRITE_REG8(&pdev->regs.INDEXREGS->CSRL.CSR0L, csr0l.d8);
        
        return 1;
    }
    /* Setup Data Stage */
    if (phost->Control.state == CTRL_DATA_IN_WAIT)
    {
        if (csr0l.b.rx_pkt_rdy)
        {
            count_len = USB_OTG_READ_REG8(&pdev->regs.INDEXREGS->COUNT.COUNT0);
            if (phost->Control.length >= count_len)
            {
                phost->Control.length -= count_len;
            }
            else
            {
                phost->Control.length = 0;
            }
            if ((count_len < phost->Control.ep0size) || (!phost->Control.length))
            {
                pdev->host.URB_State[1] = URB_DONE;
                USB_OTG_ReadPacket(pdev, phost->Control.buff, 0, count_len);
                csr0l.b.rx_pkt_rdy = 0;
                USB_OTG_WRITE_REG8(&pdev->regs.INDEXREGS->CSRL.CSR0L, csr0l.d8);
            }
            else 
            {
                USB_OTG_ReadPacket(pdev, phost->Control.buff, 0, count_len);
                phost->Control.buff += count_len;
                csr0l.b.rx_pkt_rdy = 0;
                csr0l.b.req_pkt = 1;
                USB_OTG_WRITE_REG8(&pdev->regs.INDEXREGS->CSRL.CSR0L, csr0l.d8);
            }
        }
    }
    if (phost->Control.state == CTRL_DATA_OUT_WAIT)
    {
        if (csr0l.b.tx_pkt_rdy)
        {
            pdev->host.URB_State[0] = URB_DONE;
        }
    }
    /* Setup Status Stage */
    if (phost->Control.state == CTRL_STATUS_IN_WAIT)
    {
        pdev->host.URB_State[1] = URB_DONE;
    }
    if (phost->Control.state == CTRL_STATUS_OUT_WAIT)
    {
        pdev->host.URB_State[0] = URB_DONE;
    }
    
    return retval;
}

/**
* @brief  USB_OTG_otg_hcd_handle_sof_intr 
*         Handles sof Interrupt.
* @param  pdev: Selected device
* @retval status 
*/
static uint32_t USB_OTG_USBH_handle_sof_ISR (USB_OTG_CORE_HANDLE *pdev)
{
    USB_OTG_INTRUSB_TypeDef intr_usb;
    
    USBH_HCD_INT_fops->SOF(pdev);
    intr_usb.d8 = USB_OTG_READ_REG8(&pdev->regs.COMMREGS->INTRUSB);  
    
    /* Clear interrupt */
    intr_usb.b.sof = 0;
    USB_OTG_WRITE_REG8(&pdev->regs.COMMREGS->INTRUSB, intr_usb.d8);
    
    return 1;
}
/**
* @brief  USB_OTG_USBH_handle_Connect_ISR 
*         Handles connect event.
* @param  pdev: Selected device
* @retval status 
*/
static uint32_t USB_OTG_USBH_handle_Connect_ISR (USB_OTG_CORE_HANDLE *pdev)
{
//    USB_OTG_INTRUSB_TypeDef intr_usb;
    USB_OTG_DEVCTL_TypeDef devctl;
    USB_OTG_POWER_TypeDef  power;
    
    USBH_HCD_INT_fops->DevConnected(pdev);
//    intr_usb.d8 = USB_OTG_READ_REG8(&pdev->regs.COMMREGS->INTRUSB); 
    devctl.d8 = USB_OTG_READ_REG8(&pdev->regs.DYNFIFOREGS->DEVCTL);
    power.d8 = USB_OTG_READ_REG8(&pdev->regs.COMMREGS->POWER);
//    /* Clear interrupt */
//    intr_usb.b.conn = 0;
//    USB_OTG_WRITE_REG8(&pdev->regs.COMMREGS->INTRUSB, intr_usb.d8);
    if (devctl.b.LSDev)
    {
        /* Low-speed Device */
        
    }
    if ((devctl.b.FSDev) && (!power.b.HS_mode))
    {
        /* Full-speed Device */
    }
    /* High-speed Devices are not supported */
    else if ((devctl.b.FSDev) && (power.b.HS_mode))
    {
    
    }
    
    return 1;
}
/**
* @brief  USB_OTG_USBH_handle_Disconnect_ISR 
*         Handles disconnect event.
* @param  pdev: Selected device
* @retval status 
*/
static uint32_t USB_OTG_USBH_handle_Disconnect_ISR (USB_OTG_CORE_HANDLE *pdev)
{
//    USB_OTG_INTRUSB_TypeDef intr_usb;
    USBH_HCD_INT_fops->DevDisconnected(pdev);
//    intr_usb.d8 = USB_OTG_READ_REG8(&pdev->regs.COMMREGS->INTRUSB); 
//    /* Clear interrupt */
//    intr_usb.b.discon = 0;
//    USB_OTG_WRITE_REG8(&pdev->regs.COMMREGS->INTRUSB, intr_usb.d8);
    return 1;
}
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma optimize = none
#endif /* __CC_ARM */
/**
* @brief  USB_OTG_USBH_handle_Suspend_ISR 
*         Handles suspend Interrupt.
* @param  pdev: Selected device
* @retval status 
*/
static uint32_t USB_OTG_USBH_handle_Suspend_ISR (USB_OTG_CORE_HANDLE *pdev)
{
    USB_OTG_INTRUSB_TypeDef intr_usb;
    intr_usb.d8 = USB_OTG_READ_REG8(&pdev->regs.COMMREGS->INTRUSB); 
    /* Clear interrupt */
    intr_usb.b.suspend = 0;
    USB_OTG_WRITE_REG8(&pdev->regs.COMMREGS->INTRUSB, intr_usb.d8);
    return 1;
}
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma optimize = none
#endif /* __CC_ARM */
/**
* @brief  USB_OTG_USBH_handle_Babble_ISR 
*         Handles Babble Interrupt
* @param  pdev: Selected device
* @retval status 
*/
static uint32_t USB_OTG_USBH_handle_Babble_ISR (USB_OTG_CORE_HANDLE *pdev)
{
    USB_OTG_INTRUSB_TypeDef intr_usb;
    USBH_HCD_INT_fops->USBH_Babble(pdev);
    intr_usb.d8 = USB_OTG_READ_REG8(&pdev->regs.COMMREGS->INTRUSB); 
    /* Clear interrupt */
    intr_usb.b.reset_babble = 0;
    USB_OTG_WRITE_REG8(&pdev->regs.COMMREGS->INTRUSB, intr_usb.d8);
    return 1;
}

/**
* @brief  USB_OTG_USBH_Resume_ISR 
*         Handles Resume Interrupt
* @param  pdev: Selected device
* @retval status 
*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma optimize = none
#endif /* __CC_ARM */
static uint32_t USB_OTG_USBH_Resume_ISR (USB_OTG_CORE_HANDLE *pdev)
{
    USB_OTG_INTRUSB_TypeDef intr_usb;
    USBH_HCD_INT_fops->USBH_Resume(pdev);
    intr_usb.d8 = USB_OTG_READ_REG8(&pdev->regs.COMMREGS->INTRUSB); 
    /* Clear interrupt */
    intr_usb.b.resume = 0;
    USB_OTG_WRITE_REG8(&pdev->regs.COMMREGS->INTRUSB, intr_usb.d8);
    return 1;
}
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma optimize = none
#endif /* __CC_ARM */
/**
* @brief  USB_OTG_USBH_handle_hc_n_Out_ISR 
*         Handles interrupt for a specific Host Channel
* @param  pdev: Selected device
* @param  hc_num: Channel number
* @retval status 
*/
uint32_t USB_OTG_USBH_handle_hc_n_Out_ISR (USB_OTG_CORE_HANDLE *pdev , uint32_t ep_num)
{
    USB_OTG_TXCSRL_IN_HOST_TypeDef tx_csrl;
    USB_OTG_TXCSRH_IN_HOST_TypeDef tx_csrh;
    uint8_t hc_num = 0;
    uint8_t Serch_hc_num = 0;
    uint32_t count_len = 0;
    
    while ((hc_num < 8) && (!Serch_hc_num))
    {
        if ((pdev->host.channel[hc_num] & 0xFF) == ep_num)
        {
            Serch_hc_num = 1;
        }
        else
        {
            hc_num++;
        }
    }
    
    if ((hc_num < 8) && (Serch_hc_num))
    {
        tx_csrl.d8 = USB_OTG_READ_REG8(&pdev->regs.CSRREGS[ep_num]->TXCSRL); 
        tx_csrh.d8 = USB_OTG_READ_REG8(&pdev->regs.CSRREGS[ep_num]->TXCSRH); 
        
        if (tx_csrl.b.Rx_stall)
        {
            pdev->host.URB_State[hc_num] = URB_STALL;  
            tx_csrl.b.Rx_stall = 0;
            USB_OTG_WRITE_REG8(&pdev->regs.CSRREGS[ep_num]->RXCSRL, tx_csrl.d8);
        }
        if (tx_csrl.b.error)
        {
            pdev->host.URB_State[hc_num] = URB_ERROR;
            tx_csrl.b.error = 0;
            USB_OTG_WRITE_REG8(&pdev->regs.CSRREGS[ep_num]->RXCSRL, tx_csrl.d8);
        }
        if (tx_csrl.b.NAK_timeout_incompTx)
        {
            pdev->host.URB_State[hc_num] = URB_ERROR;
            tx_csrl.b.NAK_timeout_incompTx = 0;
            USB_OTG_WRITE_REG8(&pdev->regs.CSRREGS[ep_num]->RXCSRL, tx_csrl.d8);
        }
        if (!tx_csrl.b.tx_pkt_rdy)
        {
            count_len = pdev->host.hc[hc_num].xfer_len - pdev->host.hc[hc_num].xfer_count;
            if (count_len > 0)
            {
                if (count_len < pdev->host.hc[hc_num].max_packet)
                {
                    pdev->host.URB_State[hc_num] = URB_DONE;
                    USB_OTG_WritePacket(pdev, \
                                       pdev->host.hc[hc_num].xfer_buff + pdev->host.hc[hc_num].xfer_count, \
                                       ep_num, \
                                       count_len);
                    pdev->host.hc[hc_num].xfer_count += count_len;
                    pdev->host.XferCnt[hc_num] = pdev->host.hc[hc_num].xfer_count;
                    tx_csrl.b.tx_pkt_rdy = 1;
                    USB_OTG_WRITE_REG8(&pdev->regs.CSRREGS[ep_num]->RXCSRL, tx_csrl.d8);
                }
                else 
                {
                    USB_OTG_WritePacket(pdev, \
                                       pdev->host.hc[hc_num].xfer_buff + pdev->host.hc[hc_num].xfer_count, \
                                       ep_num, \
                                       pdev->host.hc[hc_num].max_packet);
                    pdev->host.hc[hc_num].xfer_count += pdev->host.hc[hc_num].max_packet;
                    pdev->host.XferCnt[hc_num] = pdev->host.hc[hc_num].xfer_count;
                    tx_csrl.b.tx_pkt_rdy = 1;
                    USB_OTG_WRITE_REG8(&pdev->regs.CSRREGS[ep_num]->RXCSRL, tx_csrl.d8);
                }
            }
            else
            {
                if ((!(pdev->host.hc[hc_num].xfer_len % pdev->host.hc[hc_num].max_packet)) && \
                    (pdev->host.URB_State[hc_num] != URB_DONE))
                {
                    USB_OTG_WritePacket(pdev, \
                                       pdev->host.hc[hc_num].xfer_buff + pdev->host.hc[hc_num].xfer_count, \
                                       ep_num, \
                                       0);
                    pdev->host.XferCnt[hc_num] = pdev->host.hc[hc_num].xfer_count;
                    tx_csrl.b.tx_pkt_rdy = 1;
                    USB_OTG_WRITE_REG8(&pdev->regs.CSRREGS[ep_num]->RXCSRL, tx_csrl.d8);
                    pdev->host.URB_State[hc_num] = URB_DONE;
                }
                else
                {
                    pdev->host.URB_State[hc_num] = URB_DONE;
                }
            }
        }
    }
    
    return 1;
}
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma optimize = none
#endif /* __CC_ARM */
/**
* @brief  USB_OTG_USBH_handle_hc_n_In_ISR 
*         Handles interrupt for a specific Host Channel
* @param  pdev: Selected device
* @param  hc_num: Channel number
* @retval status 
*/
static uint32_t USB_OTG_USBH_handle_hc_n_In_ISR (USB_OTG_CORE_HANDLE *pdev , uint32_t ep_num)
{
    USB_OTG_RXCSRL_IN_HOST_TypeDef rx_csrl;
    USB_OTG_RXCSRH_IN_HOST_TypeDef rx_csrh;
    uint8_t count_len;
    uint8_t hc_num = 0;
    uint8_t Serch_hc_num = 0;
    
    while ((hc_num < 8) && (!Serch_hc_num))
    {
        if ((pdev->host.channel[hc_num] & 0xFF) == (0x80 | ep_num))
        {
            Serch_hc_num = 1;
        }
        else
        {
            hc_num++;
        }
    }
    
    if ((hc_num < 8) && (Serch_hc_num))
    {
        rx_csrl.d8 = USB_OTG_READ_REG8(&pdev->regs.CSRREGS[ep_num]->RXCSRL); 
        rx_csrh.d8 = USB_OTG_READ_REG8(&pdev->regs.CSRREGS[ep_num]->RXCSRH); 
        
        if (rx_csrl.b.RxStall)
        {
            pdev->host.URB_State[hc_num] = URB_STALL;  
            rx_csrl.b.RxStall = 0;
            USB_OTG_WRITE_REG8(&pdev->regs.CSRREGS[ep_num]->RXCSRL, rx_csrl.d8);
        }
        if (rx_csrl.b.error)
        {
            pdev->host.URB_State[hc_num] = URB_ERROR;
            rx_csrl.b.error = 0;
            USB_OTG_WRITE_REG8(&pdev->regs.CSRREGS[ep_num]->RXCSRL, rx_csrl.d8);
        }
        if (rx_csrl.b.data_error)
        {
            pdev->host.URB_State[hc_num] = URB_ERROR;
            rx_csrl.b.data_error = 0;
            USB_OTG_WRITE_REG8(&pdev->regs.CSRREGS[ep_num]->RXCSRL, rx_csrl.d8);
        }
        if (rx_csrl.b.rx_pkt_rdy)
        {
            count_len = USB_OTG_READ_REG8(&pdev->regs.CSRREGS[ep_num]->RXCOUNT);
            if ((count_len < pdev->host.hc[hc_num].max_packet) || \
                (pdev->host.hc[hc_num].xfer_count + count_len == pdev->host.hc[hc_num].xfer_len))
            {
                pdev->host.URB_State[hc_num] = URB_DONE;
                USB_OTG_ReadPacket(pdev, \
                                   pdev->host.hc[hc_num].xfer_buff + pdev->host.hc[hc_num].xfer_count, \
                                   ep_num, \
                                   count_len);
                pdev->host.hc[hc_num].xfer_count += count_len;
                pdev->host.XferCnt[hc_num] = pdev->host.hc[hc_num].xfer_count;
                rx_csrl.b.rx_pkt_rdy = 0;
                USB_OTG_WRITE_REG8(&pdev->regs.CSRREGS[ep_num]->RXCSRL, rx_csrl.d8);
            }
            else 
            {
                USB_OTG_ReadPacket(pdev, \
                                   pdev->host.hc[hc_num].xfer_buff + pdev->host.hc[hc_num].xfer_count, \
                                   ep_num, \
                                   count_len);
                pdev->host.hc[hc_num].xfer_count += count_len;
                pdev->host.XferCnt[hc_num] = pdev->host.hc[hc_num].xfer_count;
                rx_csrl.b.rx_pkt_rdy = 0;
                rx_csrl.b.ReqPkt = 1;
                USB_OTG_WRITE_REG8(&pdev->regs.CSRREGS[ep_num]->RXCSRL, rx_csrl.d8);
            }
        }
    }
    
    return 1;
}

/**
* @brief  USB_OTG_USBH_handle_SessReq_ISR 
*         Handles SessReq Interrupt
* @param  pdev: Selected device
* @retval status 
*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma optimize = none
#endif /* __CC_ARM */
static uint32_t USB_OTG_USBH_handle_SessReq_ISR (USB_OTG_CORE_HANDLE *pdev)
{
    USB_OTG_INTRUSB_TypeDef intr_usb;
    USBH_HCD_INT_fops->USBH_SessReq(pdev);
    intr_usb.d8 = USB_OTG_READ_REG8(&pdev->regs.COMMREGS->INTRUSB); 
    /* Clear interrupt */
    intr_usb.b.sess_req = 0;
    USB_OTG_WRITE_REG8(&pdev->regs.COMMREGS->INTRUSB, intr_usb.d8);
    return 1;
}

/**
* @brief  USB_OTG_USBH_handle_VBusError_ISR 
*         Handles VBus Error Interrupt
* @param  pdev: Selected device
* @retval status 
*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma optimize = none
#endif /* __CC_ARM */
static uint32_t USB_OTG_USBH_handle_VBusError_ISR (USB_OTG_CORE_HANDLE *pdev)
{
    USB_OTG_INTRUSB_TypeDef intr_usb;
    USBH_HCD_INT_fops->USBH_VbusError(pdev);
    intr_usb.d8 = USB_OTG_READ_REG8(&pdev->regs.COMMREGS->INTRUSB); 
    /* Clear interrupt */
    intr_usb.b.VBus_error = 0;
    USB_OTG_WRITE_REG8(&pdev->regs.COMMREGS->INTRUSB, intr_usb.d8);
    return 1;
}

/**
* @}
*/ 

/**
* @}
*/ 

/**
* @}
*/
/************************ (C) COPYRIGHT 2014 Megahuntmicro ****END OF FILE****/
