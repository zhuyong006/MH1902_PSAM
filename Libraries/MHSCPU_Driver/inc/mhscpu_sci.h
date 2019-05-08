/**
\file       megahunt_scu_uart.h
\brief      This is a brief description.
\details    This is the detail description.
\author     jf
\date       2013-06-27
\version    1.0
\par Copyright (c):
    2013, MegaHunt.
\par History:
    Version | Author    | Date      | Descript
    ------- | :----:    | ----      | --------
    1.0     | jf    | 2012-06-27    | Create file. 
\note
    This is note consists.
*/

#ifndef __MEGAHUNT_SCU_SCI_H
#define __MEGAHUNT_SCU_SCI_H

#ifdef __cplusplus
extern "C" {
#endif 
     
    #include "mhscpu.h"
#if   defined ( __CC_ARM )
    #include <arm_math.h>
#endif


    #define SCI_UNCONFIG                (-1)
    #define SCI_ICC_CLOCK_ERR           (-2)
    #define SCI_REF_CLOCK_ERR           (-3)
    #define SCI_IMPRECISION_CLK         (-4)
    #define SCI_EMV_F_D_ERR             (-5)
    #define SCI_EMV_TS_ERR              (-6)
    #define SCI_EMV_ATR_ERR             (-7)
    #define SCI_CARD_OUT_ERR            (-8)

    
    int32_t SCI_ConfigEMV(uint8_t SCI_Bitmap, uint32_t SCIx_Clk);

#ifdef __cplusplus
}
#endif

#endif // __MEGAHUNT_SCU_UART_H
