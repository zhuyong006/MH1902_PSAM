/**************************************************************************//**
 * @file     <Device>.h
 * @brief    CMSIS Cortex-M# Core Peripheral Access Layer Header File for
 *           Device <Device>
 * @version  V3.10
 * @date     23. November 2012
 *
 * @note
 *
 ******************************************************************************/
/* Copyright (c) 2012 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list ofC conditions and the following disclaimer.
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


#ifndef MHSCPU_H      /* ToDo: replace '<Device>' with your device name */
#define MHSCPU_H

#ifdef __cplusplus
 extern "C" {
#endif

/* ToDo: replace '<Device>' with your device name; add your doxyGen comment   */
/** @addtogroup <Device>_Definitions <Device> Definitions
  This file defines all structures and symbols for <Device>:
    - registers and bitfields
    - peripheral base address
    - peripheral ID
    - Peripheral definitions
  @{
*/


/******************************************************************************/
/*                Processor and Core Peripherals                              */
/******************************************************************************/
/** @addtogroup <Device>_CMSIS Device CMSIS Definitions
  Configuration of the Cortex-M# Processor and Core Peripherals
  @{
*/
/*
 * ==========================================================================
 * ---------- Interrupt Number Definition -----------------------------------
 * ==========================================================================
 */
typedef enum IRQn
{
/******  Cortex-M# Processor Exceptions Numbers ***************************************************/



/* ToDo: use this Cortex interrupt numbers if your device is a CORTEX-M3 / Cortex-M4 device       */
  NonMaskableInt_IRQn           = -14,      /*!<  2 Non Maskable Interrupt                        */
  MemoryManagement_IRQn         = -12,      /*!<  4 Memory Management Interrupt                   */
  BusFault_IRQn                 = -11,      /*!<  5 Bus Fault Interrupt                           */
  UsageFault_IRQn               = -10,      /*!<  6 Usage Fault Interrupt                         */
  SVCall_IRQn                   = -5,       /*!< 11 SV Call Interrupt                             */
  DebugMonitor_IRQn             = -4,       /*!< 12 Debug Monitor Interrupt                       */
  PendSV_IRQn                   = -2,       /*!< 14 Pend SV Interrupt                             */
  SysTick_IRQn                  = -1,       /*!< 15 System Tick Interrupt                         */

/******  Device Specific Interrupt Numbers ********************************************************/
/* ToDo: add here your device specific external interrupt numbers
         according the interrupt handlers defined in startup_Device.s
         eg.: Interrupt for Timer#1       TIM1_IRQHandler   ->   TIM1_IRQn                        */
    DMA_IRQn                                        = 0,
	USB_IRQn										= 1,
	USBDMA_IRQn										= 2,
    LCD_IRQn                                        = 3,
    SCI0_IRQn                                       = 4,
    UART0_IRQn                                      = 5,
    UART1_IRQn                                      = 6,
    SPI0_IRQn                                       = 7,
    CRYPT0_IRQn                                     = 8,
    TIM0_0_IRQn                                     = 9,
    TIM0_1_IRQn                                     = 10,
    TIM0_2_IRQn                                     = 11,
    TIM0_3_IRQn                                     = 12,
    EXTI0_IRQn                                      = 13,
    EXTI1_IRQn                                      = 14,
    EXTI2_IRQn                                      = 15,
    RTC_IRQn                                        = 16,
    SENSOR_IRQn                                     = 17,
    TRNG_IRQn                                       = 18,
    ADC0_IRQn                                       = 19,
    SSC_IRQn                                        = 20,
    TIM0_4_IRQn                                     = 21,
    TIM0_5_IRQn                                     = 22,
    DCMI_IRQn                                       = 23,
    MSR_IRQn                                        = 24,
    EXTI3_IRQn                                      = 25,
    SPI1_IRQn										= 26,
    SPI2_IRQn										= 27,
    IMG_COP_IRQn									= 28
} IRQn_Type;


/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of the Cortex-M# Processor and Core Peripherals */
/* ToDo: set the defines according your Device                                                    */
/* ToDo: define the correct core revision
         __CM0_REV if your device is a CORTEX-M0 device
         __CM3_REV if your device is a CORTEX-M3 device
         __CM4_REV if your device is a CORTEX-M4 device                                           */
//#define __CM3_REV                 0x0201    /*!< Core Revision r2p1                               */
#define __CM3_REV                 0x0200    /*!< Core Revision r2p0                               */
#define __NVIC_PRIO_BITS          4         /*!< Number of Bits used for Priority Levels          */
#define __Vendor_SysTickConfig    0         /*!< Set to 1 if different SysTick Config is used     */
#define __MPU_PRESENT             1         /*!< MPU present or not                               */
/* ToDo: define __FPU_PRESENT if your devise is a CORTEX-M4                                       */
#define __FPU_PRESENT             0        /*!< FPU present or not                                */

/*@}*/ /* end of group <Device>_CMSIS */


/* ToDo: include the correct core_cm#.h file
         core_cm0.h if your device is a CORTEX-M0 device
         core_cm3.h if your device is a CORTEX-M3 device
         core_cm4.h if your device is a CORTEX-M4 device                                          */
#include "core_cm3.h"                       /* Cortex-M# processor and core peripherals           */
/* ToDo: include your system_<Device>.h file
         replace '<Device>' with your device name                                                 */
#include "system_mhscpu.h"                /* <Device> System  include file                      */


/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/
/** @addtogroup <Device>_Peripherals <Device> Peripherals
  <Device> Device Specific Peripheral registers structures
  @{
*/

#if defined ( __CC_ARM   )
#pragma anon_unions
#endif

#include <stdint.h>

/** @addtogroup Exported_types
  * @{
  */  

/*!<Standard Peripheral Library old types (maintained for legacy purpose) */
// typedef int32_t      s32;
// typedef int16_t  s16;
// typedef int8_t   s8;

// typedef const int32_t sc32;  /*!< Read Only */
// typedef const int16_t sc16;  /*!< Read Only */
// typedef const int8_t     sc8;   /*!< Read Only */

// typedef __IO int32_t  vs32;
// typedef __IO int16_t  vs16;
// typedef __IO int8_t   vs8;

// typedef __I int32_t vsc32;  /*!< Read Only */
// typedef __I int16_t vsc16;  /*!< Read Only */
// typedef __I int8_t   vsc8;   /*!< Read Only */

// typedef uint32_t  u32;
// typedef uint16_t     u16;
// typedef uint8_t      u8;

// typedef const uint32_t   uc32;  /*!< Read Only */
// typedef const uint16_t   uc16;  /*!< Read Only */
// typedef const uint8_t    uc8;   /*!< Read Only */

// typedef __IO uint32_t    vu32;
// typedef __IO uint16_t    vu16;
// typedef __IO uint8_t     vu8;

// typedef __I uint32_t     vuc32;  /*!< Read Only */
// typedef __I uint16_t     vuc16;  /*!< Read Only */
// typedef __I uint8_t  vuc8;   /*!< Read Only */

typedef enum {RESET = 0, SET = !RESET} FlagStatus, ITStatus;

typedef enum {DISABLE = 0, ENABLE = !DISABLE} FunctionalState;
#define IS_FUNCTIONAL_STATE(STATE) (((STATE) == DISABLE) || ((STATE) == ENABLE))

typedef enum {FALSE = 0, TRUE = !FALSE} Boolean;

typedef enum {ERROR = 0, SUCCESS = !ERROR} ErrorStatus;

/* ToDo: add here your device specific peripheral access structure typedefs
         following is an example for a timer                                  */
#define BIT0        (0x00000001U)
#define BIT1        (0x00000002U)
#define BIT2        (0x00000004U)
#define BIT3        (0x00000008U)
#define BIT4        (0x00000010U)
#define BIT5        (0x00000020U)
#define BIT6        (0x00000040U)
#define BIT7        (0x00000080U)
#define BIT8        (0x00000100U)
#define BIT9        (0x00000200U)
#define BIT10       (0x00000400U)
#define BIT11       (0x00000800U)
#define BIT12       (0x00001000U)
#define BIT13       (0x00002000U)
#define BIT14       (0x00004000U)
#define BIT15       (0x00008000U)
#define BIT16       (0x00010000U)
#define BIT17       (0x00020000U)
#define BIT18       (0x00040000U)
#define BIT19       (0x00080000U)
#define BIT20       (0x00100000U)
#define BIT21       (0x00200000U)
#define BIT22       (0x00400000U)
#define BIT23       (0x00800000U)
#define BIT24       (0x01000000U)
#define BIT25       (0x02000000U)
#define BIT26       (0x04000000U)
#define BIT27       (0x08000000U)
#define BIT28       (0x10000000U)
#define BIT29       (0x20000000U)
#define BIT30       (0x40000000U)
#define BIT31       (0x80000000U)

#define BIT(n)      (1UL << (n))

typedef struct
{
    __IO uint32_t FREQ_SEL;
    __IO uint32_t CG_CTRL1;
    __IO uint32_t CG_CTRL2;
    __O uint32_t  SOFT_RST1;
    __O uint32_t  SOFT_RST2;
    __IO uint32_t LOCK_R;
    __IO uint32_t PHER_CTRL;
	__IO uint32_t WAKE_EN1;
    __IO uint32_t WAKE_EN2;
    __I uint32_t  WAKE_FLAG1;
    __I uint32_t  WAKE_FLAG2;
    __I uint32_t  HCLK_1MS_VAL;
    __I uint32_t  PCLK_1MS_VAL;
    __IO uint32_t ANA_CTRL;
    __IO uint32_t DMA_CHAN;
    __IO uint32_t SCI_GLF;
    __IO uint32_t SW_RSV1;
    __IO uint32_t SW_RSV2;
    __IO uint32_t CARD_RSVD;
    __I uint32_t  SYS_RSCD[(0x100 - 0x4C)>>2];
    __IO uint32_t  MSR_CR1;
    __IO uint32_t  MSR_CR2;
	__IO uint32_t USBPHY_CR1;
	__IO uint32_t USBPHY_CR2;
	__IO uint32_t USBPHY_CR3;
    __I uint32_t  SYS_RSVD1[(0x3EC - 0x114)>>2];
    __IO uint32_t PM2_WK_FLAG;
    __IO uint32_t CALIB_CSR;        /* CALIB_CSRÔ­¼Ä´æÆ÷ÃûÎªCALIB_SR */
    __IO uint32_t DBG_CR;
    __IO uint32_t CHIP_ID;
} SYSCTRL_TypeDef;

typedef struct
{
    union
    {
        __I  uint32_t RBR;
        __O  uint32_t THR;
        __IO uint32_t DLL;
    } OFFSET_0;
    union
    {
        __IO uint32_t DLH;
        __IO uint32_t IER;
    } OFFSET_4;
    union
    {
        __I uint32_t IIR;
        __O uint32_t FCR;
    } OFFSET_8;
    __IO uint32_t LCR;
    __IO uint32_t MCR;
    __I  uint32_t LSR;
    __I  uint32_t MSR;
    __IO uint32_t SCR;
    __IO uint32_t LPDLL;
    __IO uint32_t LPDLH;
    __I  uint32_t RES0[2];
    union
    {
        __I  uint32_t SRBR[16];
        __O  uint32_t STHR[16];
    } OFFSET_48;
    __IO uint32_t FAR;
    __I  uint32_t TFR;
    __O  uint32_t RFW;
    __I  uint32_t USR;
    __I  uint32_t TFL;
    __I  uint32_t RFL;
    __O  uint32_t SRR;
    __IO uint32_t SRTS;
    __IO uint32_t SBCR;
    __IO uint32_t SDMAM;
    __IO uint32_t SFE;
    __IO uint32_t SRT;
    __IO uint32_t STET;
    __IO uint32_t HTX;
    __O uint32_t DMASA;
    __I  uint32_t RES1[18];
    __I  uint32_t CPR;
    __I  uint32_t UCV;
    __I  uint32_t CTR;
    
} UART_TypeDef;

typedef struct
{
    __IO uint16_t CTRLR0;
    uint16_t RESERVED0;
    __IO uint16_t CTRLR1;
    uint16_t RESERVED1;
    __IO uint32_t SSIENR;
    __IO uint32_t MWCR;
    __IO uint32_t SER;
    __IO uint32_t BAUDR;
    __IO uint32_t TXFTLR;
    __IO uint32_t RXFTLR;
    __IO uint32_t TXFLR;
    __I  uint32_t RXFLR;
    __I  uint32_t SR;
    __IO uint32_t IMR;
    __I  uint32_t ISR;
    __I  uint32_t RISR;
    __I  uint32_t TXOICR;
    __I  uint32_t RXOICR;
    __I  uint32_t RXUICR;
    __I  uint32_t MSTICR;
    __IO uint32_t ICR;
    __IO uint32_t DMACR;
    __IO uint32_t DMATDLR;
    __IO uint32_t DMARDLR;
    __I  uint32_t IDR;
    __I  uint32_t SSI_COMP_VERSION;
    __IO uint32_t DR;
    __IO uint32_t DR_Array[35];
    __IO uint32_t RX_SAMPLE_DLY;    
} SPI_TypeDef;

typedef struct
{
    __IO uint32_t WDT_CR;
    __IO uint32_t RESERVED0;
    __I  uint32_t WDT_CCVR;
    __O  uint32_t WDT_CRR;
    __I  uint32_t WDT_STAT;
    __I  uint32_t WDT_EOI;  
    __I  uint32_t RESERVED1;
    __IO uint32_t WDT_RLD;
    __I  uint32_t RESERVED[53];
    __I  uint32_t WDT_COMP_PARAMS_1;
    __I  uint32_t WDT_COMP_VERSION;
    __I  uint32_t WDT_COMP_TYPE;
} WDT_TypeDef;

typedef struct
{
    __IO uint32_t CRC_CSR;
    __O  uint32_t CRC_INI;
    union
    {
        __I uint32_t DOUT;
        __O uint8_t  DIN;
    } CRC_DATA;
} CRC_TypeDef;

typedef struct
{
    __IO uint32_t LoadCount;
    __I  uint32_t CurrentValue;
    __IO uint32_t ControlReg;
    __IO  uint32_t EOI;
    __I  uint32_t IntStatus;
} TIM_TypeDef;

typedef struct
{
    #define TIM_NUM 6
    TIM_TypeDef TIM[TIM_NUM];
    __I uint8_t RESERVED[0xA0 - TIM_NUM * sizeof(TIM_TypeDef)];
    __I  uint32_t TIM_IntStatus;
    __I  uint32_t TIM_EOI;
    __I  uint32_t TIM_RawIntStatus;
    __I  uint32_t TIM_Comp;
    __IO uint32_t TIM_ReloadCount[TIM_NUM];
} TIM_Module_TypeDef;

typedef struct 
{
    __IO uint32_t ADC_CR;
    __I  uint32_t ADC_SR;
} ADC_TypeDef;

typedef struct
{
    __IO uint32_t IODR;
    __IO uint32_t BSRR;
    __IO uint32_t OEN;
    __IO uint32_t PUE;
} GPIO_TypeDef;

typedef struct
{
    __IO uint32_t INTP_TYPE;
    __IO uint32_t INTP_STA;
} GPIO_INTP_TypeDef;

typedef struct
{
    #define GPIO_NUM    4
    GPIO_TypeDef GPIO[GPIO_NUM];
    __I  uint32_t RSVD0[(0x11C - 0x040)>>2];
    __I  uint32_t INTP[GPIO_NUM];
    __I  uint32_t RSVD1[(0x180 - 0x12C)>>2];
    __IO uint32_t ALT[GPIO_NUM];
    __I  uint32_t RSVD2[(0x200 - 0x190)>>2];
    __IO uint32_t SYS_CR1;
    __I  uint32_t RSVD3[(0x220 - 0x204)>>2];
    __IO uint32_t WAKE_EVEN_TYPE_EN;
    __IO uint32_t WAKE_EVEN_SRC_L;
	__IO uint32_t WAKE_EVEN_SRC_H;
    __I  uint32_t RSVD5[(0x800 - 0x22C)>>2];
    GPIO_INTP_TypeDef INTP_TYPE_STA[GPIO_NUM];
} GPIO_MODULE_TypeDef;


typedef struct
{
    __IO uint32_t KEY[32];
    __IO uint32_t BPK_RDY;
    __IO uint32_t BPK_CLR;
    __IO uint32_t BPK_LRA;
    __IO uint32_t BPK_LWA;
    __IO uint32_t BPK_RR;
    __IO uint32_t BPK_LR;
    __O  uint32_t BPK_SCR;
    __I  uint32_t BPK_RSVD0;
    __IO uint32_t RTC_CS;
    __IO uint32_t RTC_REF;
    __IO uint32_t RTC_ARM;
    __I  uint32_t RTC_TIM;
    __O  uint32_t RTC_INTCLR;
    __IO uint32_t OSC32K_CR;
    __IO uint32_t RTC_ATTA_TIM;
    __IO uint32_t RESERVED2;
    __O uint32_t SEN_EXT_TYPE;
    __IO uint32_t SEN_EXT_CFG;
    __IO uint32_t SEN_SOFT_EN;
    __IO uint32_t SEN_STATE;
    __IO uint32_t SEN_BRIDGE;
    __IO uint32_t SEN_SOFT_ATTACK_SOFT_ATTACK;
    __IO uint32_t SEN_SOFT_LOCK;
    __IO uint32_t SEN_ATTACK_CNT;
    __IO uint32_t SEN_REG;
    __IO uint32_t SEN_VG_DETECT;
    __IO uint32_t SEN_RNG_INI;
    __IO uint32_t RESERVED3[(0x0104 - 0x00EC) >> 2];
    __IO uint32_t SEN_EN[13];
    __IO uint32_t RESERVED4[(0x0134 - 0x011C) >> 2];
    __IO uint32_t SEN_EXTS_START;
    __IO uint32_t SEN_EXTS_LOCK;
    __IO uint32_t SEN_ANAO;
    __IO uint32_t SEN_RSVD;
} BPU_MODULE_TypeDef;

typedef struct
{
    __IO uint32_t KEY[32];
    __IO uint32_t BPK_RDY;
    __IO uint32_t BPK_CLR;
    __IO uint32_t BPK_LRA;
    __IO uint32_t BPK_LWA;
    __IO uint32_t BPK_RR;
    __IO uint32_t BPK_LR;
    __O  uint32_t BPK_SCR;
    __I  uint32_t BPK_RSVD0;
} BPK_TypeDef;

typedef struct
{
    __IO uint32_t RTC_CS;
    __IO uint32_t RTC_REF;
    __IO uint32_t RTC_ARM;
    __I  uint32_t RTC_TIM;
    __O  uint32_t RTC_INTCLR;
    __IO uint32_t OSC32K_CR;
    __IO uint32_t RTC_ATTA_TIM;
} RTC_TypeDef;


typedef struct
{
    __O uint32_t SEN_EXT_TYPE;
    __IO uint32_t SEN_EXT_CFG;
    __IO uint32_t SEN_SOFT_EN;
    __IO uint32_t SEN_STATE;
    __IO uint32_t SEN_BRIDGE;
    __IO uint32_t SEN_SOFT_ATTACK;
    __IO uint32_t SEN_SOFT_LOCK;
    __IO uint32_t SEN_ATTACK_CNT;
    __IO uint32_t SEN_ATTACK_TYP;
    __IO uint32_t SEN_VG_DETECT;
    __IO uint32_t SEN_RNG_INI;
    __O uint32_t RESERVED3[(0x0104 - 0x00EC)/4];
    __IO uint32_t SEN_EN[19];
    __IO uint32_t SEN_EXTS_START;
    __IO uint32_t SEN_LOCK;
} SEN_TypeDef;

typedef struct
{
    __IO uint32_t RNG_CSR;
    __IO uint32_t RNG_DATA[1];
    __I  uint32_t RES;
    __IO uint32_t RNG_AMA;
    __IO uint32_t RNG_PN;
} TRNG_TypeDef;

typedef struct
{
    __IO uint32_t SAR_L;
    __IO uint32_t SAR_H;
    __IO uint32_t DAR_L;
    __IO uint32_t DAR_H;
    __IO uint32_t LLP_L;
    __IO uint32_t LLP_H;
    __IO uint32_t CTL_L;
    __IO uint32_t CTL_H;
    __IO uint32_t SSTAT_L;
    __IO uint32_t SSTAT_H;
    __IO uint32_t DSTAT_L;
    __IO uint32_t DSTAT_H;
    __IO uint32_t SSTATAR_L;
    __IO uint32_t SSTATAR_H;
    __IO uint32_t DSTATAR_L;
    __IO uint32_t DSTATAR_H;
    __IO uint32_t CFG_L;
    __IO uint32_t CFG_H;
    __IO uint32_t SGR_L;
    __IO uint32_t SGR_H;
    __IO uint32_t DSR_L;
    __IO uint32_t DSR_H;
} DMA_TypeDef;

typedef struct
{
    DMA_TypeDef DMA_Channel[4];

    __IO uint32_t RESERVED1[88];
    
    __I  uint32_t RawTfr_L;
    __I  uint32_t RawTfr_H;
    __I  uint32_t RawBlock_L;
    __I  uint32_t RawBlock_H;
    __I  uint32_t RawSrcTran_L;
    __I  uint32_t RawSrcTran_H;
    __I  uint32_t RawDstTran_L;
    __I  uint32_t RawDstTran_H;
    __I  uint32_t RawErr_L;
    __I  uint32_t RawErr_H;
    
    __I  uint32_t StatusTfr_L;
    __I  uint32_t StatusTfr_H;
    __I  uint32_t StatusBlock_L;
    __I  uint32_t StatusBlock_H;
    __I  uint32_t StatusSrcTran_L;
    __I  uint32_t StatusSrcTran_H;
    __I  uint32_t StatusDstTran_L;
    __I  uint32_t StatusDstTran_H;
    __I  uint32_t StatusErr_L;
    __I  uint32_t StatusErr_H;

    __IO uint32_t MaskTfr_L;
    __IO uint32_t MaskTfr_H;
    __IO uint32_t MaskBlock_L;
    __IO uint32_t MaskBlock_H;
    __IO uint32_t MaskSrcTran_L;
    __IO uint32_t MaskSrcTran_H;
    __IO uint32_t MaskDstTran_L;
    __IO uint32_t MaskDstTran_H;
    __IO uint32_t MaskErr_L;
    __IO uint32_t MaskErr_H;
    
    __O  uint32_t ClearTfr_L;
    __O  uint32_t ClearTfr_H;
    __O  uint32_t ClearBlock_L;
    __O  uint32_t ClearBlock_H;
    __O  uint32_t ClearSrcTran_L;
    __O  uint32_t ClearSrcTran_H;
    __O  uint32_t ClearDstTran_L;
    __O  uint32_t ClearDstTran_H;
    __O  uint32_t ClearErr_L;
    __O  uint32_t ClearErr_H;
    
    __I  uint32_t StatusInt_L;
    __I  uint32_t StatusInt_H;
    
    __IO uint32_t ReqSrcReg_L;
    __IO uint32_t ReqSrcReg_H;
    __IO uint32_t ReqDstReg_L;
    __IO uint32_t ReqDstReg_H;
    __IO uint32_t SglReqSrcReg_L;
    __IO uint32_t SglReqSrcReg_H;
    __IO uint32_t SglReqDstReg_L;
    __IO uint32_t SglReqDstReg_H;
    __IO uint32_t LstSrcReg_L;
    __IO uint32_t LstSrcReg_H;
    __IO uint32_t LstDstReg_L;
    __IO uint32_t LstDstReg_H;
    
    __IO uint32_t DmaCfgReg_L;
    __IO uint32_t DmaCfgReg_H;
    __IO uint32_t ChEnReg_L;
    __IO uint32_t ChEnReg_H;
    __I  uint32_t DmaIdReg_L;
    __I  uint32_t DmaIdReg_H;
    __IO uint32_t DmaTestReg_L;
    __IO uint32_t DmaTestReg_H;
    
    __IO uint32_t RESERVED2[4];
    
    __I  uint32_t DMA_COMP_PARAMS_6_L;
    __I  uint32_t DMA_COMP_PARAMS_6_H;
    __I  uint32_t DMA_COMP_PARAMS_5_L;
    __I  uint32_t DMA_COMP_PARAMS_5_H;
    __I  uint32_t DMA_COMP_PARAMS_4_L;
    __I  uint32_t DMA_COMP_PARAMS_4_H;
    __I  uint32_t DMA_COMP_PARAMS_3_L;
    __I  uint32_t DMA_COMP_PARAMS_3_H;
    __I  uint32_t DMA_COMP_PARAMS_2_L;
    __I  uint32_t DMA_COMP_PARAMS_2_H;
    __I  uint32_t DMA_COMP_PARAMS_1_L;
    __I  uint32_t DMA_COMP_PARAMS_1_H;
    __I  uint32_t DMA_Component_ID_Register_L;
    __I  uint32_t DMA_Component_ID_Register_H;

} DMA_MODULE_TypeDef;

typedef struct
{
  __IO uint32_t lcdi_ctrl;
  __IO uint32_t lcdi_cycle;
  __IO uint32_t lcdi_status;
  __IO uint32_t lcdi_data;
 __IO uint32_t lcdi_fifolevel;
 __IO uint32_t lcdi_fifothr;
} LCD_TypeDef;

typedef struct
{
  __IO uint32_t SCI_DATA;
  __IO uint32_t SCI_CR0;
  __IO uint32_t SCI_CR1;
  __IO uint32_t SCI_CR2;
  __IO uint32_t SCI_IER;
  __IO uint32_t SCI_RETRY;
  __IO uint32_t SCI_TIDE;
  __IO uint32_t SCI_TXCOUNT;
  __IO uint32_t SCI_RXCOUNT;
  __I  uint32_t SCI_FR;
  __IO uint32_t SCI_RXTIME;
  __IO uint32_t SCI_ISTAT;
  __IO uint32_t SCI_STABLE;
  __IO uint32_t SCI_ATIME;
  __IO uint32_t SCI_DTIME;

  __IO uint32_t SCI_ATRSTIME;
  __IO uint32_t SCI_ATRDTIME;
  __IO uint32_t SCI_BLKTIME;
  __IO uint32_t SCI_CHTIME;
  __IO uint32_t SCI_CLKICC;
  __IO uint32_t SCI_BAUD;
  __IO uint32_t SCI_VALUE;
  __IO uint32_t SCI_CHGUARD;
  __IO uint32_t SCI_BLKGUARD;
  __IO uint32_t SCI_SYNCCR;
  __IO uint32_t SCI_SYNCDATA;
  __IO uint32_t SCI_RAWSTAT;
  __IO uint32_t SCI_IIR;
  __I  uint32_t SCI_RES1[4];
  __I  uint32_t SCI_RES2[32];
} SCI_TypeDef;

/** 
  * @brief DCMI&IMG COP
  */

typedef struct
{
  __IO uint32_t CR;       /*!< DCMI control register 1,                       Address offset: 0x00 */
  __IO uint32_t SR;       /*!< DCMI status register,                          Address offset: 0x04 */
  __IO uint32_t RISR;     /*!< DCMI raw interrupt status register,            Address offset: 0x08 */
  __IO uint32_t IER;      /*!< DCMI interrupt enable register,                Address offset: 0x0C */
  __IO uint32_t MISR;     /*!< DCMI masked interrupt status register,         Address offset: 0x10 */
  __IO uint32_t ICR;      /*!< DCMI interrupt clear register,                 Address offset: 0x14 */
  __IO uint32_t RESERVED1[2];
  __IO uint32_t CWSTRTR;  /*!< DCMI crop window start,                        Address offset: 0x20 */
  __IO uint32_t CWSIZER;  /*!< DCMI crop window size,                         Address offset: 0x24 */
  __IO uint32_t DR;       /*!< DCMI data register,                            Address offset: 0x28 */
} DCMI_TypeDef;

typedef struct{
	__IO uint32_t IMG_COP_CONFIG;
	__I uint32_t STATUS;
	__IO uint32_t INTC;
	__IO uint32_t IMG_SIZE;
	__IO uint32_t IMG_ADDR;
	__IO uint32_t INTGIMG_ADDR;
	__IO uint32_t BINIMG_ADDR;
	__IO uint32_t WINDOW_ADDR;
	__IO uint32_t SEARCH_PIXEL;
	__IO uint32_t SEARCH_MAX;
	__I  uint32_t SUM_X;
	__I  uint32_t SUM_Y;
	__I  uint32_t SUM_TOTAL;
	__I  uint32_t SEARCH_COUNT;
} IMG_COP_TypeDef;


typedef struct
{
    __IO uint32_t  CFG;
    __IO uint32_t  CS;
    __IO uint32_t  PROT;
    __IO uint32_t *ADDR;
    __IO uint32_t  PDATA;
    __IO uint32_t  RO;
    __IO uint32_t  ROL;
    __IO uint32_t  RO_INFO;
    __IO uint32_t  ROL_INFO;
    __IO uint32_t  HIDE_INFO;
    __IO uint32_t  SCR;
    __IO uint32_t  TIM;
    __IO uint32_t  SRC_LOCK;
    __IO uint32_t  RSVD;
    __IO uint32_t  SRC_SHDW;
    __IO uint32_t  SRC_INFO_EN;
    __IO uint32_t  IEL;
    __IO uint32_t  CB;
} FCU_TypeDef;

typedef struct
{
    __IO uint32_t  SSC_CR1;
    __IO uint32_t  SSC_CR2;
    __IO uint32_t  SSC_CR3;
    __O  uint32_t  RESERVED0[(0x0104-0x000C)/4];
    __IO uint32_t  SSC_SR;
    __IO uint32_t  SSC_SR_CLR;
    __IO uint32_t  SSC_ACK;
    __O  uint32_t  RESERVED1[(0x0184-0x0110)/4];
    __IO uint32_t  DATARAM_SCR;
    __O  uint32_t  RESERVED2[(0x01FC-0x0188)/4];
    __IO uint32_t  BPU_RWC;
    __O  uint32_t  RESERVED3[(0x03EC-0x0200)/4];
    __IO uint32_t  MAIN_SEN_LOCK;
    __IO uint32_t  MAIN_SEN_EN;
} SSC_TypeDef;

typedef struct
{
    __IO uint32_t  TST_JTAG;
    __IO uint32_t  TST_ROM;
    __IO uint32_t  TST_FLASH;
} MH_SMCU_TST_TypeDef;

#if defined ( __CC_ARM   )
#pragma no_anon_unions
#endif

/*@}*/ /* end of group <Device>_Peripherals */


/******************************************************************************/
/*                         Peripheral memory map                              */
/******************************************************************************/
/* ToDo: add here your device peripherals base addresses
         following is an example for timer                                    */
/** @addtogroup <Device>_MemoryMap <Device> Memory Mapping
  @{
*/

/* Peripheral and SRAM base address */
#define MHSCPU_FLASH_BASE                       (0x01000000UL)                              /*!< (FLASH     ) Base Address */
#define MHSCPU_SRAM_BASE                        (0x20000000UL)                              /*!< (SRAM      ) Base Address */
#define MHSCPU_PERIPH_BASE                      (0x40000000UL)                              /*!< (Peripheral) Base Address */

#define MHSCPU_EXT_FLASH_BASE                   (0x60000000UL)
#define MHSCPU_EXT_RAM_BASE                     (0x80000000UL)


#define FLASH_SIZE                              (1UL << 19)
#define FLASH_BASE                              (0x01000000UL)

#define OTP_SIZE                                (1UL << 14)
#define OTP_BASE                                (0x02000000UL)

#define SRAM_SIZE                               (1UL << 16)
#define SRAM_BASE                               (0x20000000UL)

/* Peripheral memory map */
#define MHSCPU_AHB_BASE                         (MHSCPU_PERIPH_BASE)
#define MHSCPU_APB0_BASE                        (MHSCPU_PERIPH_BASE + 0x10000)
#define MHSCPU_APB1_BASE                        (MHSCPU_PERIPH_BASE + 0x20000)
#define MHSCPU_APB2_BASE                        (MHSCPU_PERIPH_BASE + 0x30000)

#define SSC_BASE                                (MHSCPU_AHB_BASE + 0x0000)
#define TST_BASE                                (MHSCPU_AHB_BASE + 0x03F4)
#define DMA_BASE                                (MHSCPU_AHB_BASE + 0x0800)
#define USB_BASE                                (MHSCPU_AHB_BASE + 0x0C00)
#define LCD_BASE                                (MHSCPU_AHB_BASE + 0x1000)
#define FCU_BASE                                (MHSCPU_AHB_BASE + 0x1400)
#define DCMI_BASE                               (MHSCPU_AHB_BASE + 0x1800)

#define SCI0_BASE                               (MHSCPU_APB0_BASE)
#define IMG_COP_BASE							(MHSCPU_APB0_BASE + 0x1000)
#define CRC_BASE                                (MHSCPU_APB0_BASE + 0x2000)
#define TIMM0_BASE                              (MHSCPU_APB0_BASE + 0x3000)
#define ADC_BASE                                (MHSCPU_APB0_BASE + 0x4000)
#define UART0_BASE                              (MHSCPU_APB0_BASE + 0x6000)
#define UART1_BASE                              (MHSCPU_APB0_BASE + 0x7000)
#define SPIM1_BASE                              (MHSCPU_APB0_BASE + 0x8000)
#define SPIM2_BASE                              (MHSCPU_APB0_BASE + 0x9000)
#define SPIM0_BASE                              (MHSCPU_APB0_BASE + 0xA000)
#define SPIS0_BASE                              (MHSCPU_APB0_BASE + 0xB000)
#define WDG_BASE                                (MHSCPU_APB0_BASE + 0xC000)
#define GPIO_BASE                               (MHSCPU_APB0_BASE + 0xD000)
#define TRNG_BASE                               (MHSCPU_APB0_BASE + 0xE000)
#define SYSCTRL_BASE                            (MHSCPU_APB0_BASE + 0xF000)

#define MSR_BASE                                (MHSCPU_APB1_BASE)

#define BPU_BASE                                (MHSCPU_APB2_BASE)

/*@}*/ /* end of group <Device>_MemoryMap */


/******************************************************************************/
/*                         Peripheral declaration                             */
/******************************************************************************/
/* ToDo: add here your device peripherals pointer definitions
         following is an example for timer                                    */

/** @addtogroup <Device>_PeripheralDecl <Device> Peripheral Declaration
  @{
*/
#define SYSCTRL                                 ((SYSCTRL_TypeDef *) SYSCTRL_BASE)

#define UART0                                   ((UART_TypeDef *) UART0_BASE)
#define UART1                                   ((UART_TypeDef *) UART1_BASE)

#define SPIM1									((SPI_TypeDef *) SPIM1_BASE)
#define SPIM2									((SPI_TypeDef *) SPIM2_BASE)
#define SPIM0                                   ((SPI_TypeDef *) SPIM0_BASE)
#define SPIS0                                   ((SPI_TypeDef *) SPIS0_BASE)

#define SCI0                                    ((SCI_TypeDef *) SCI0_BASE)

#define TIMM0                                   ((TIM_Module_TypeDef *)TIMM0_BASE)

#define ADC0                                    ((ADC_TypeDef *)ADC_BASE) 

#define TRNG                                    ((TRNG_TypeDef *)TRNG_BASE)
#define LCD                                     ((LCD_TypeDef *)LCD_BASE)  
#define CRC                                     ((CRC_TypeDef *)CRC_BASE)

#define DMA                                     ((DMA_MODULE_TypeDef *)DMA_BASE)
#define DMA_Channel_0                           ((DMA_TypeDef *)DMA_BASE)
#define DMA_Channel_1                           ((DMA_TypeDef *)(DMA_BASE + 0x58))
#define DMA_Channel_2                           ((DMA_TypeDef *)(DMA_BASE + 0x58*2))
#define DMA_Channel_3                           ((DMA_TypeDef *)(DMA_BASE + 0x58*3))

#define GPIO                                    ((GPIO_MODULE_TypeDef *)GPIO_BASE)
#define GPIOA                                   ((GPIO_TypeDef *)GPIO_BASE)
#define GPIOB                                   ((GPIO_TypeDef *)(GPIO_BASE + 0x0010))
#define GPIOC                                   ((GPIO_TypeDef *)(GPIO_BASE + 0x0020))
#define GPIOD                                   ((GPIO_TypeDef *)(GPIO_BASE + 0x0030))
#define GPIO_GROUP                              ((GPIO_TypeDef *)GPIO_BASE)
#define GPIO_ALT_GROUP                          ((__IO uint32_t *)(GPIO_BASE + 0x180))
#define GPIO_WKEN_TYPE_EN						((__IO uint32_t *)(GPIO_BASE + 0x220))
#define GPIO_WKEN_PL_EN							((__IO uint32_t *)(GPIO_BASE + 0x224))
#define GPIO_WKEN_PH_EN							((__IO uint32_t *)(GPIO_BASE + 0x228))

#define WDT                                     ((WDT_TypeDef *)WDG_BASE)
#define FCU                                     ((FCU_TypeDef *)FCU_BASE)
#define SSC                                     ((SSC_TypeDef *)SSC_BASE)   
#define TST                                     ((MH_SMCU_TST_TypeDef *)TST_BASE)

#define DCMI                                    ((DCMI_TypeDef *) DCMI_BASE)
#define IMG_COP 								((IMG_COP_TypeDef *) IMG_COP_BASE)

#define BPU                                     ((BPU_MODULE_TypeDef *)BPU_BASE)
#define BPK                                     ((BPK_TypeDef *)BPU_BASE)
#define RTC                                     ((RTC_TypeDef *)(BPU_BASE + 0xA0))
#define SENSOR                                  ((SEN_TypeDef *)(BPU_BASE + 0xC0))           


/** @addtogroup Exported_constants
  * @{
  */
  
  /** @addtogroup Peripheral_Registers_Bits_Definition
  * @{
  */
    
/******************************************************************************/
/*                         Peripheral Registers_Bits_Definition               */
/******************************************************************************/

/******************************************************************************/
/*                                                                            */
/*                Universal Asynchronous Receiver Transmitter                 */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for UART_RBR register  *******************/
#define UART_RBR_RBR                         ((uint32_t)0x01FF)            /*!< Data value */

/*******************  Bit definition for UART_THR register  *******************/
#define UART_THR_THR                         ((uint32_t)0x01FF)            /*!< Data value */

/*******************  Bit definition for UART_DLH register  *******************/
#define UART_DLH_DLH                         ((uint32_t)0x0FF)

/*******************  Bit definition for UART_DLL register  *******************/
#define UART_DLL_DLL                         ((uint32_t)0x0FF)

/*******************  Bit definition for UART_IER register  *******************/
#define UART_IER_ERBFI                          ((uint32_t)0x0001)
#define UART_IER_ETBEI                          ((uint32_t)0x0002)
#define UART_IER_ELSI                           ((uint32_t)0x0004)
#define UART_IER_EDSSI                          ((uint32_t)0x0008)
#define UART_IER_PTIME                          ((uint32_t)0x0080)

/*******************  Bit definition for UART_IIR register  *******************/
#define UART_IIR_IID                            ((uint32_t)0x0007)
#define UART_IIR_IID_0                          ((uint32_t)0x0001)
#define UART_IIR_IID_1                          ((uint32_t)0x0002)
#define UART_IIR_IID_2                          ((uint32_t)0x0004)
#define UART_IIR_IID_3                          ((uint32_t)0x0008)
#define UART_IIR_FIFOSE                         ((uint32_t)0x0060)
#define UART_IIR_FIFOSE_0                       ((uint32_t)0x0020)
#define UART_IIR_FIFOSE_1                       ((uint32_t)0x0040)

/*******************  Bit definition for UART_FCR register  *******************/
#define UART_FCR_FIFOE                          ((uint32_t)0x0001)
#define UART_FCR_RFIFOR                         ((uint32_t)0x0002)
#define UART_FCR_XFIFOR                         ((uint32_t)0x0004)
#define UART_FCR_DMAM                           ((uint32_t)0x0008)
#define UART_FCR_TET                            ((uint32_t)0x0030)
#define UART_FCR_TET_0                          ((uint32_t)0x0010)
#define UART_FCR_TET_1                          ((uint32_t)0x0020)
#define UART_FCR_RCVER                          ((uint32_t)0x00C0)
#define UART_FCR_RCVER_0                        ((uint32_t)0x0040)
#define UART_FCR_RCVER_1                        ((uint32_t)0x0080)

/*******************  Bit definition for UART_LCR register  *******************/
#define UART_LCR_DLS                            ((uint32_t)0x0003)
#define UART_LCR_DLS_0                          ((uint32_t)0x0001)
#define UART_LCR_DLS_1                          ((uint32_t)0x0002)
#define UART_LCR_STOP                           ((uint32_t)0x0004)
#define UART_LCR_PEN                            ((uint32_t)0x0008)
#define UART_LCR_EPS                            ((uint32_t)0x0010)
#define UART_LCR_SP                             ((uint32_t)0x0020)
#define UART_LCR_BC                             ((uint32_t)0x0040)
#define UART_LCR_DLAB                           ((uint32_t)0x0080)

/*******************  Bit definition for UART_MCR register  *******************/
#define UART_MCR_DTR                            ((uint32_t)0x0001)
#define UART_MCR_RTS                            ((uint32_t)0x0002)
#define UART_MCR_OUT1                           ((uint32_t)0x0004)
#define UART_MCR_OUT2                           ((uint32_t)0x0008)
#define UART_MCR_LB                             ((uint32_t)0x0010)
#define UART_MCR_AFCE                           ((uint32_t)0x0020)
#define UART_MCR_SIRE                           ((uint32_t)0x0040)

/*******************  Bit definition for UART_LSR register  *******************/
#define UART_LSR_DR                             ((uint32_t)0x0001)
#define UART_LSR_OE                             ((uint32_t)0x0002)
#define UART_LSR_PE                             ((uint32_t)0x0004)
#define UART_LSR_FE                             ((uint32_t)0x0008)
#define UART_LSR_BI                             ((uint32_t)0x0010)
#define UART_LSR_THRE                           ((uint32_t)0x0020)
#define UART_LSR_TEMT                           ((uint32_t)0x0040)
#define UART_LSR_PFE                            ((uint32_t)0x0080)

/*******************  Bit definition for UART_MSR register  *******************/
#define UART_MSR_DCTS                               ((uint32_t)0x0001)
#define UART_MSR_DDSR                               ((uint32_t)0x0002)
#define UART_MSR_TERI                               ((uint32_t)0x0004)
#define UART_MSR_DDCD                               ((uint32_t)0x0008)
#define UART_MSR_CTS                                ((uint32_t)0x0010)
#define UART_MSR_DSR                                ((uint32_t)0x0020)
#define UART_MSR_RI                                 ((uint32_t)0x0040)
#define UART_MSR_DCD                                ((uint32_t)0x0080)

/*******************  Bit definition for UART_SRBR register  *******************/
#define UART_SRBR_SRBR                          ((uint32_t)0x01FF)            /*!< Data value */

/*******************  Bit definition for UART_STHR register  *******************/
#define UART_STHR_STHR                          ((uint32_t)0x01FF)            /*!< Data value */

/*******************  Bit definition for UART_FAR register  *******************/
#define UART_FAR_FAR                            ((uint32_t)0x0001)

/*******************  Bit definition for UART_TFR register  *******************/
#define UART_TFR_TFR                            ((uint32_t)0x00FF)

/*******************  Bit definition for UART_RFW register  *******************/
#define UART_RFW_RFWD                           ((uint32_t)0x00FF)
#define UART_RFW_RFPE                           ((uint32_t)0x0100)
#define UART_RFW_RFFE                           ((uint32_t)0x0200)

/*******************  Bit definition for UART_USR register  *******************/
#define UART_USR_BUSY                               ((uint32_t)0x0001)
#define UART_USR_TFNF                               ((uint32_t)0x0002)
#define UART_USR_TFE                                ((uint32_t)0x0004)
#define UART_USR_RFNE                               ((uint32_t)0x0008)
#define UART_USR_RFF                                ((uint32_t)0x0010)

/*******************  Bit definition for UART_TFL register  *******************/
#define UART_TFL_TFL                                ((uint32_t)0x000F)

/*******************  Bit definition for UART_RFL register  *******************/
#define UART_RFL_RFL                                ((uint32_t)0x000F)

/*******************  Bit definition for UART_SRR register  *******************/
#define UART_SRR_UR                                 ((uint32_t)0x0001)
#define UART_SRR_RFR                                ((uint32_t)0x0002)
#define UART_SRR_XFR                                ((uint32_t)0x0004)

/*******************  Bit definition for UART_SRR register  *******************/
#define UART_SRR_UR                                 ((uint32_t)0x0001)

/*******************  Bit definition for UART_SRTS register  *******************/
#define UART_SRTS_SRTS                              ((uint32_t)0x0001)

/*******************  Bit definition for UART_SBCR register  *******************/
#define UART_SBCR_SBCR                              ((uint32_t)0x0001)

/*******************  Bit definition for UART_SDMAM register  *******************/
#define UART_SDMAM_SDMAM                            ((uint32_t)0x0001)

/*******************  Bit definition for UART_SFE register  *******************/
#define UART_SFE_SFE                                ((uint32_t)0x0001)

/*******************  Bit definition for UART_SRT register  *******************/
#define UART_SRT_SRT                                ((uint32_t)0x0003)
#define UART_SRT_SRT_0                              ((uint32_t)0x0001)
#define UART_SRT_SRT_1                              ((uint32_t)0x0002)

/*******************  Bit definition for UART_STET register  *******************/
#define UART_STET_STET                              ((uint32_t)0x0003)
#define UART_STET_STET_0                            ((uint32_t)0x0001)
#define UART_STET_STET_1                            ((uint32_t)0x0002)

/*******************  Bit definition for UART_HTX register  *******************/
#define UART_HTX_HTX                                ((uint32_t)0x0001)

/*******************  Bit definition for UART_DMASA register  *******************/
#define UART_DMASA_DMASA                            ((uint32_t)0x0001)

/******************************************************************************/
/*                                                                            */
/*                General Purpose and Alternate Function I/O                  */
/*                                                                            */
/******************************************************************************/

/*!<******************  Bit definition for GPIO_IODR register  *******************/
#define GPIO_IODR_ODR0                        ((uint16_t)0x00000001)            /*!< Port output data, bit 0 */
#define GPIO_IODR_ODR1                        ((uint16_t)0x00000002)            /*!< Port output data, bit 1 */
#define GPIO_IODR_ODR2                        ((uint16_t)0x00000004)            /*!< Port output data, bit 2 */
#define GPIO_IODR_ODR3                        ((uint16_t)0x00000008)            /*!< Port output data, bit 3 */
#define GPIO_IODR_ODR4                        ((uint16_t)0x00000010)            /*!< Port output data, bit 4 */
#define GPIO_IODR_ODR5                        ((uint16_t)0x00000020)            /*!< Port output data, bit 5 */
#define GPIO_IODR_ODR6                        ((uint16_t)0x00000040)            /*!< Port output data, bit 6 */
#define GPIO_IODR_ODR7                        ((uint16_t)0x00000080)            /*!< Port output data, bit 7 */
#define GPIO_IODR_ODR8                        ((uint16_t)0x00000100)            /*!< Port output data, bit 8 */
#define GPIO_IODR_ODR9                        ((uint16_t)0x00000200)            /*!< Port output data, bit 9 */
#define GPIO_IODR_ODR10                       ((uint16_t)0x00000400)            /*!< Port output data, bit 10 */
#define GPIO_IODR_ODR11                       ((uint16_t)0x00000800)            /*!< Port output data, bit 11 */
#define GPIO_IODR_ODR12                       ((uint16_t)0x00001000)            /*!< Port output data, bit 12 */
#define GPIO_IODR_ODR13                       ((uint16_t)0x00002000)            /*!< Port output data, bit 13 */
#define GPIO_IODR_ODR14                       ((uint16_t)0x00004000)            /*!< Port output data, bit 14 */
#define GPIO_IODR_ODR15                       ((uint16_t)0x00008000)            /*!< Port output data, bit 15 */

#define GPIO_IODR_IDR0                        ((uint16_t)0x00010000)            /*!< Port input data, bit 0 */
#define GPIO_IODR_IDR1                        ((uint16_t)0x00020000)            /*!< Port input data, bit 1 */
#define GPIO_IODR_IDR2                        ((uint16_t)0x00040000)            /*!< Port input data, bit 2 */
#define GPIO_IODR_IDR3                        ((uint16_t)0x00080000)            /*!< Port input data, bit 3 */
#define GPIO_IODR_IDR4                        ((uint16_t)0x00100000)            /*!< Port input data, bit 4 */
#define GPIO_IODR_IDR5                        ((uint16_t)0x00200000)            /*!< Port input data, bit 5 */
#define GPIO_IODR_IDR6                        ((uint16_t)0x00400000)            /*!< Port input data, bit 6 */
#define GPIO_IODR_IDR7                        ((uint16_t)0x00800000)            /*!< Port input data, bit 7 */
#define GPIO_IODR_IDR8                        ((uint16_t)0x01000000)            /*!< Port input data, bit 8 */
#define GPIO_IODR_IDR9                        ((uint16_t)0x02000000)            /*!< Port input data, bit 9 */
#define GPIO_IODR_IDR10                       ((uint16_t)0x04000000)            /*!< Port input data, bit 10 */
#define GPIO_IODR_IDR11                       ((uint16_t)0x08000000)            /*!< Port input data, bit 11 */
#define GPIO_IODR_IDR12                       ((uint16_t)0x10000000)            /*!< Port input data, bit 12 */
#define GPIO_IODR_IDR13                       ((uint16_t)0x20000000)            /*!< Port input data, bit 13 */
#define GPIO_IODR_IDR14                       ((uint16_t)0x40000000)            /*!< Port input data, bit 14 */
#define GPIO_IODR_IDR15                       ((uint16_t)0x80000000)            /*!< Port input data, bit 15 */

/******************  Bit definition for GPIO_BSRR register  *******************/
#define GPIO_BSRR_BS0                        ((uint32_t)0x00000001)        /*!< Port x Set bit 0 */
#define GPIO_BSRR_BS1                        ((uint32_t)0x00000002)        /*!< Port x Set bit 1 */
#define GPIO_BSRR_BS2                        ((uint32_t)0x00000004)        /*!< Port x Set bit 2 */
#define GPIO_BSRR_BS3                        ((uint32_t)0x00000008)        /*!< Port x Set bit 3 */
#define GPIO_BSRR_BS4                        ((uint32_t)0x00000010)        /*!< Port x Set bit 4 */
#define GPIO_BSRR_BS5                        ((uint32_t)0x00000020)        /*!< Port x Set bit 5 */
#define GPIO_BSRR_BS6                        ((uint32_t)0x00000040)        /*!< Port x Set bit 6 */
#define GPIO_BSRR_BS7                        ((uint32_t)0x00000080)        /*!< Port x Set bit 7 */
#define GPIO_BSRR_BS8                        ((uint32_t)0x00000100)        /*!< Port x Set bit 8 */
#define GPIO_BSRR_BS9                        ((uint32_t)0x00000200)        /*!< Port x Set bit 9 */
#define GPIO_BSRR_BS10                       ((uint32_t)0x00000400)        /*!< Port x Set bit 10 */
#define GPIO_BSRR_BS11                       ((uint32_t)0x00000800)        /*!< Port x Set bit 11 */
#define GPIO_BSRR_BS12                       ((uint32_t)0x00001000)        /*!< Port x Set bit 12 */
#define GPIO_BSRR_BS13                       ((uint32_t)0x00002000)        /*!< Port x Set bit 13 */
#define GPIO_BSRR_BS14                       ((uint32_t)0x00004000)        /*!< Port x Set bit 14 */
#define GPIO_BSRR_BS15                       ((uint32_t)0x00008000)        /*!< Port x Set bit 15 */

#define GPIO_BSRR_BR0                        ((uint32_t)0x00010000)        /*!< Port x Reset bit 0 */
#define GPIO_BSRR_BR1                        ((uint32_t)0x00020000)        /*!< Port x Reset bit 1 */
#define GPIO_BSRR_BR2                        ((uint32_t)0x00040000)        /*!< Port x Reset bit 2 */
#define GPIO_BSRR_BR3                        ((uint32_t)0x00080000)        /*!< Port x Reset bit 3 */
#define GPIO_BSRR_BR4                        ((uint32_t)0x00100000)        /*!< Port x Reset bit 4 */
#define GPIO_BSRR_BR5                        ((uint32_t)0x00200000)        /*!< Port x Reset bit 5 */
#define GPIO_BSRR_BR6                        ((uint32_t)0x00400000)        /*!< Port x Reset bit 6 */
#define GPIO_BSRR_BR7                        ((uint32_t)0x00800000)        /*!< Port x Reset bit 7 */
#define GPIO_BSRR_BR8                        ((uint32_t)0x01000000)        /*!< Port x Reset bit 8 */
#define GPIO_BSRR_BR9                        ((uint32_t)0x02000000)        /*!< Port x Reset bit 9 */
#define GPIO_BSRR_BR10                       ((uint32_t)0x04000000)        /*!< Port x Reset bit 10 */
#define GPIO_BSRR_BR11                       ((uint32_t)0x08000000)        /*!< Port x Reset bit 11 */
#define GPIO_BSRR_BR12                       ((uint32_t)0x10000000)        /*!< Port x Reset bit 12 */
#define GPIO_BSRR_BR13                       ((uint32_t)0x20000000)        /*!< Port x Reset bit 13 */
#define GPIO_BSRR_BR14                       ((uint32_t)0x40000000)        /*!< Port x Reset bit 14 */
#define GPIO_BSRR_BR15                       ((uint32_t)0x80000000)        /*!< Port x Reset bit 15 */

/******************  Bit definition for GPIO_OEN register  *******************/
#define GPIO_OEN_0                          ((uint32_t)0x00000001)        /*!< 1:Port Input 0:Port Output */
#define GPIO_OEN_1                          ((uint32_t)0x00000002)        /*!< 1:Port Input 0:Port Output */
#define GPIO_OEN_2                          ((uint32_t)0x00000004)        /*!< 1:Port Input 0:Port Output */
#define GPIO_OEN_3                          ((uint32_t)0x00000008)        /*!< 1:Port Input 0:Port Output */
#define GPIO_OEN_4                          ((uint32_t)0x00000010)        /*!< 1:Port Input 0:Port Output */
#define GPIO_OEN_5                          ((uint32_t)0x00000020)        /*!< 1:Port Input 0:Port Output */
#define GPIO_OEN_6                          ((uint32_t)0x00000040)        /*!< 1:Port Input 0:Port Output */
#define GPIO_OEN_7                          ((uint32_t)0x00000080)        /*!< 1:Port Input 0:Port Output */
#define GPIO_OEN_8                          ((uint32_t)0x00000100)        /*!< 1:Port Input 0:Port Output */
#define GPIO_OEN_9                          ((uint32_t)0x00000200)        /*!< 1:Port Input 0:Port Output */
#define GPIO_OEN_10                         ((uint32_t)0x00000400)        /*!< 1:Port Input 0:Port Output */
#define GPIO_OEN_11                         ((uint32_t)0x00000800)        /*!< 1:Port Input 0:Port Output */
#define GPIO_OEN_12                         ((uint32_t)0x00001000)        /*!< 1:Port Input 0:Port Output */
#define GPIO_OEN_13                         ((uint32_t)0x00002000)        /*!< 1:Port Input 0:Port Output */
#define GPIO_OEN_14                         ((uint32_t)0x00004000)        /*!< 1:Port Input 0:Port Output */
#define GPIO_OEN_15                         ((uint32_t)0x00008000)        /*!< 1:Port Input 0:Port Output */

/******************  Bit definition for GPIO_PUE register  *******************/
#define GPIO_PUE_0                          ((uint32_t)0x00000001)        /*!< 1:PullUp Enable 0:PullUp Disable */
#define GPIO_PUE_1                          ((uint32_t)0x00000002)        /*!< 1:PullUp Enable 0:PullUp Disable */
#define GPIO_PUE_2                          ((uint32_t)0x00000004)        /*!< 1:PullUp Enable 0:PullUp Disable */
#define GPIO_PUE_3                          ((uint32_t)0x00000008)        /*!< 1:PullUp Enable 0:PullUp Disable */
#define GPIO_PUE_4                          ((uint32_t)0x00000010)        /*!< 1:PullUp Enable 0:PullUp Disable */
#define GPIO_PUE_5                          ((uint32_t)0x00000020)        /*!< 1:PullUp Enable 0:PullUp Disable */
#define GPIO_PUE_6                          ((uint32_t)0x00000040)        /*!< 1:PullUp Enable 0:PullUp Disable */
#define GPIO_PUE_7                          ((uint32_t)0x00000080)        /*!< 1:PullUp Enable 0:PullUp Disable */
#define GPIO_PUE_8                          ((uint32_t)0x00000100)        /*!< 1:PullUp Enable 0:PullUp Disable */
#define GPIO_PUE_9                          ((uint32_t)0x00000200)        /*!< 1:PullUp Enable 0:PullUp Disable */
#define GPIO_PUE_10                         ((uint32_t)0x00000400)        /*!< 1:PullUp Enable 0:PullUp Disable */
#define GPIO_PUE_11                         ((uint32_t)0x00000800)        /*!< 1:PullUp Enable 0:PullUp Disable */
#define GPIO_PUE_12                         ((uint32_t)0x00001000)        /*!< 1:PullUp Enable 0:PullUp Disable */
#define GPIO_PUE_13                         ((uint32_t)0x00002000)        /*!< 1:PullUp Enable 0:PullUp Disable */
#define GPIO_PUE_14                         ((uint32_t)0x00004000)        /*!< 1:PullUp Enable 0:PullUp Disable */
#define GPIO_PUE_15                         ((uint32_t)0x00008000)        /*!< 1:PullUp Enable 0:PullUp Disable */

/******************  Bit definition for GPIO_ALT register  *******************/
#define GPIO_ALT_PORT0                      (BIT0 | BIT1)
#define GPIO_ALT_PORT0_0                    (BIT0)
#define GPIO_ALT_PORT0_1                    (BIT1)
#define GPIO_ALT_PORT1                      (BIT2 | BIT3)
#define GPIO_ALT_PORT1_0                    (BIT2)
#define GPIO_ALT_PORT1_1                    (BIT3)
#define GPIO_ALT_PORT2                      (BIT4 | BIT5)
#define GPIO_ALT_PORT2_0                    (BIT4)
#define GPIO_ALT_PORT2_1                    (BIT5)
#define GPIO_ALT_PORT3                      (BIT6 | BIT7)
#define GPIO_ALT_PORT3_0                    (BIT6)
#define GPIO_ALT_PORT3_1                    (BIT7)
#define GPIO_ALT_PORT4                      (BIT8 | BIT9)
#define GPIO_ALT_PORT4_0                    (BIT8)
#define GPIO_ALT_PORT4_1                    (BIT9)
#define GPIO_ALT_PORT5                      (BIT10 | BIT11)
#define GPIO_ALT_PORT5_0                    (BIT10)
#define GPIO_ALT_PORT5_1                    (BIT11)
#define GPIO_ALT_PORT6                      (BIT12 | BIT13)
#define GPIO_ALT_PORT6_0                    (BIT12)
#define GPIO_ALT_PORT6_1                    (BIT13)
#define GPIO_ALT_PORT7                      (BIT14 | BIT15)
#define GPIO_ALT_PORT7_0                    (BIT14)
#define GPIO_ALT_PORT7_1                    (BIT15)
#define GPIO_ALT_PORT8                      (BIT16 | BIT17)
#define GPIO_ALT_PORT8_0                    (BIT16)
#define GPIO_ALT_PORT8_1                    (BIT17)
#define GPIO_ALT_PORT9                      (BIT18 | BIT19)
#define GPIO_ALT_PORT9_0                    (BIT18)
#define GPIO_ALT_PORT9_1                    (BIT19)
#define GPIO_ALT_PORT10                     (BIT20 | BIT21)
#define GPIO_ALT_PORT10_0                   (BIT20)
#define GPIO_ALT_PORT10_1                   (BIT21)
#define GPIO_ALT_PORT11                     (BIT22 | BIT23)
#define GPIO_ALT_PORT11_0                   (BIT22)
#define GPIO_ALT_PORT11_1                   (BIT23)
#define GPIO_ALT_PORT12                     (BIT24 | BIT25)
#define GPIO_ALT_PORT12_0                   (BIT24)
#define GPIO_ALT_PORT12_1                   (BIT25)
#define GPIO_ALT_PORT13                     (BIT26 | BIT27)
#define GPIO_ALT_PORT13_0                   (BIT26)
#define GPIO_ALT_PORT13_1                   (BIT27)
#define GPIO_ALT_PORT14                     (BIT28 | BIT29)
#define GPIO_ALT_PORT14_0                   (BIT28)
#define GPIO_ALT_PORT14_1                   (BIT29)
#define GPIO_ALT_PORT15                     (BIT30 | BIT31)
#define GPIO_ALT_PORT15_0                   (BIT30)
#define GPIO_ALT_PORT15_1                   (BIT31)

/******************************************************************************/
/*                                                                            */
/*                        Serial Peripheral Interface                         */
/*                                                                            */
/******************************************************************************/

/*****************  Bit definition for SPI_CTRLR0 register  *******************/
#define SPI_CTRLR0_DFS                      ((uint32_t)0x000F)
#define SPI_CTRLR0_DFS_0                    ((uint32_t)0x0001)
#define SPI_CTRLR0_DFS_1                    ((uint32_t)0x0002)
#define SPI_CTRLR0_DFS_2                    ((uint32_t)0x0004)
#define SPI_CTRLR0_DFS_3                    ((uint32_t)0x0008)
#define SPI_CTRLR0_FRF                      ((uint32_t)0x0030)
#define SPI_CTRLR0_FRF_0                    ((uint32_t)0x0010)
#define SPI_CTRLR0_FRF_1                    ((uint32_t)0x0020)
#define SPI_CTRLR0_SCPH                     ((uint32_t)0x0040)
#define SPI_CTRLR0_SCPOL                    ((uint32_t)0x0080)
#define SPI_CTRLR0_TMOD                     ((uint32_t)0x0300)
#define SPI_CTRLR0_TMOD_0                   ((uint32_t)0x0100)
#define SPI_CTRLR0_TMOD_1                   ((uint32_t)0x0200)
#define SPI_CTRLR0_SLV_OE                   ((uint32_t)0x0400)
#define SPI_CTRLR0_SRL                      ((uint32_t)0x0800)
#define SPI_CTRLR0_CFS                      ((uint32_t)0xF000)

/*****************  Bit definition for SPI_CTRLR1 register  *******************/
#define SPI_CTRLR0_NDF                      ((uint32_t)0xFFFF)

/*****************  Bit definition for SPI_SSIENR register  *******************/
#define SPI_SSIENR_SSIENR                   ((uint32_t)0x0001)

/*****************  Bit definition for SPI_MWCR register  *********************/
#define SPI_MWCR_MWMOD                      ((uint32_t)0x0001)
#define SPI_MWCR_MDD                        ((uint32_t)0x0002)
#define SPI_MWCR_MHS                        ((uint32_t)0x0004)

/*****************  Bit definition for SPI_SER register  **********************/
#define SPI_SER_SER                         ((uint32_t)0x000F)
#define SPI_SER_0                           ((uint32_t)0x0001)
#define SPI_SER_1                           ((uint32_t)0x0002)
#define SPI_SER_2                           ((uint32_t)0x0004)
#define SPI_SER_3                           ((uint32_t)0x0008)

/*****************  Bit definition for SPI_BAUDR register  ********************/
#define SPI_BAUDR_BAUDR                     ((uint32_t)0xFFFF)

/*****************  Bit definition for SPI_TXFTLR register  *******************/
#define SPI_TXFTLR_TFT                      ((uint32_t)0x000F)
/*****************  Bit definition for SPI_RXFTLR register  *******************/
#define SPI_RXFTLR_RFT                      ((uint32_t)0x000F)
/*****************  Bit definition for SPI_TXFLR register  ********************/
#define SPI_TXFLR_TXTFL                     ((uint32_t)0x001F)
/*****************  Bit definition for SPI_RXFLR register  ********************/
#define SPI_RXFLR_RXTFL                     ((uint32_t)0x001F)

/*****************  Bit definition for SPI_SR register  ***********************/
#define SPI_SR_BUSY                         ((uint32_t)0x0001)
#define SPI_SR_TFNF                         ((uint32_t)0x0002)
#define SPI_SR_TFE                          ((uint32_t)0x0004)
#define SPI_SR_RFNE                         ((uint32_t)0x0008)
#define SPI_SR_RFF                          ((uint32_t)0x0010)
#define SPI_SR_TXE                          ((uint32_t)0x0020)
#define SPI_SR_DCOL                         ((uint32_t)0x0040)
/*****************  Bit definition for SPI_IMR register  **********************/
#define SPI_IMR_TXEIM                       ((uint32_t)0x0001)
#define SPI_IMR_TXOIM                       ((uint32_t)0x0002)
#define SPI_IMR_RXUIM                       ((uint32_t)0x0004)
#define SPI_IMR_RXOIM                       ((uint32_t)0x0008)
#define SPI_IMR_RXFIM                       ((uint32_t)0x0010)
#define SPI_IMR_MSTIM                       ((uint32_t)0x0020)
/*****************  Bit definition for SPI_ISR register  **********************/
#define SPI_ISR_TXEIS                       ((uint32_t)0x0001)
#define SPI_ISR_TXOIS                       ((uint32_t)0x0002)
#define SPI_ISR_RXUIS                       ((uint32_t)0x0004)
#define SPI_ISR_RXOIS                       ((uint32_t)0x0008)
#define SPI_ISR_RXFIS                       ((uint32_t)0x0010)
#define SPI_ISR_MSTIS                       ((uint32_t)0x0020)
/*****************  Bit definition for SPI_RISR register  *********************/
#define SPI_RISR_TXEIR                      ((uint32_t)0x0001)
#define SPI_RISR_TXOIR                      ((uint32_t)0x0002)
#define SPI_RISR_RXUIR                      ((uint32_t)0x0004)
#define SPI_RISR_RXOIR                      ((uint32_t)0x0008)
#define SPI_RISR_RXFIR                      ((uint32_t)0x0010)
#define SPI_RISR_MSTIR                      ((uint32_t)0x0020)
/*****************  Bit definition for SPI_TXOICR register  *******************/
#define SPI_TXOICR_TXOICR                   ((uint32_t)0x0001)
/*****************  Bit definition for SPI_RXOICR register  *******************/
#define SPI_RXOICR_RXOICR                   ((uint32_t)0x0001)
/*****************  Bit definition for SPI_RXUICR register  *******************/
#define SPI_RXUICR_RXUICR                   ((uint32_t)0x0001)
/*****************  Bit definition for SPI_MSTICR register  *******************/
#define SPI_MSTICR_MSTICR                   ((uint32_t)0x0001)

/*****************  Bit definition for SPI_DMACR register  ********************/
#define SPI_DMACR_RDMAE                     ((uint32_t)0x0001)
#define SPI_DMACR_TDMAE                     ((uint32_t)0x0002)
/*****************  Bit definition for SPI_DMATDLR register  ******************/
#define SPI_DMATDLR_DMATDLR                 ((uint32_t)0x000F)
/*****************  Bit definition for SPI_DMARDLR register  ******************/
#define SPI_DMATDLR_DMARDLR                 ((uint32_t)0x000F)
/*****************  Bit definition for SPI_DMARDLR register  ******************/
#define SPI_DMATDLR_DMARDLR                 ((uint32_t)0x000F)

/*****************  Bit definition for SPI_DR register  ***********************/
#define SPI_DR_DR                           ((uint32_t)0xFFFF)
/**************  Bit definition for SPI_RX_SAMPLE_DLY register  ***************/
#define SPI_RX_SAMPLE_DLY                   ((uint32_t)0xFFFF)


/******************************************************************************/
/*                                                                            */
/*                      Timer Control Unit Block                              */
/*                                                                            */
/******************************************************************************/
/**********  Bit definition for TIMER_CONTROL_REG register  *******************/
#define TIMER_CONTROL_REG_TIMER_ENABLE              (0x0001U)
#define TIMER_CONTROL_REG_TIMER_MODE                (0x0002U) 
#define TIMER_CONTROL_REG_TIMER_INTERRUPT           (0x0004U)
#define TIMER_CONTROL_REG_TIMER_PWM                 (0x0008U)
/*****************  Bit definition for IntStatus register  ********************/
#define TIMER_INT_STATUS_INTERRUPT                  (0x0001U)


/******************************************************************************/
/*                                                                            */
/*                          WDT Control Unit Block                            */
/*                                                                            */
/******************************************************************************/
/*****************  Bit definition for WDT_CR register  ***********************/
#define WDT_CR_WDT_EN                               ((uint32_t)0x0001)
#define WDT_CR_RMOD                                 ((uint32_t)0x0002)
/*****************  Bit definition for WDT_CCVR register  *********************/
#define WDT_CCVR_CCVR                               ((uint32_t)0xFFFFFFFF)
/*****************  Bit definition for WDT_CRR register  **********************/
#define WDT_CRR_CRR                                 ((uint32_t)0x00FF)
/*****************  Bit definition for WDT_STAT register  *********************/
#define WDT_STAT_INT                                ((uint32_t)0x0001)
/*****************  Bit definition for WDT_EOI register  **********************/
#define WDT_EOI_EOI                                 ((uint32_t)0x0001)
/*****************  Bit definition for WDT_RLD register  **********************/
#define WDT_RLD_RLD                                 ((uint32_t)0xFFFFFFFF)


/******************************************************************************/
/*                                                                            */
/*                          RNG Control Unit Block                            */
/*                                                                            */
/******************************************************************************/

/************ bit definition for TRNG RNG_CSR REGISTER ************/
#define TRNG_RNG_CSR_INTP_EN_Mask                   BIT(4)
#define TRNG_RNG_CSR_ATTACK_TRNG0_Mask              BIT(2)
#define TRNG_RNG_CSR_S128_TRNG0_Mask                BIT(0)

/************ bit definition for TRNG RNG_AMA REGISTER ************/
#define TRNG_RNG_AMA_PD_TRNG0_Mask                  BIT(30)
#define TRNG_RNG_AMA_ANA_OUT_TRNG0_Mask             BIT(28)

#define MHSCPU_READ_REG8(reg)                       (*(__IO uint8_t *) reg)
#define MHSCPU_READ_REG16(reg)                      (*(__IO uint16_t *) reg)
#define MHSCPU_READ_REG32(reg)                      (*(__IO uint32_t *) reg)
#define MHSCPU_WRITE_REG8(reg, value)               (*(__IO uint8_t *) reg = value)
#define MHSCPU_WRITE_REG16(reg, value)              (*(__IO uint16_t *) reg = value)
#define MHSCPU_WRITE_REG32(reg, value)              (*(__IO uint32_t *) reg = value)
#define MHSCPU_MODIFY_REG8(reg, clear_mask, set_mask)   \
        MHSCPU_WRITE_REG8(reg, (((MHSCPU_READ_REG8(reg)) & ~clear_mask) | set_mask))
#define MHSCPU_MODIFY_REG16(reg, clear_mask, set_mask)   \
        MHSCPU_WRITE_REG16(reg, (((MHSCPU_READ_REG16(reg)) & ~clear_mask) | set_mask))
#define MHSCPU_MODIFY_REG32(reg, clear_mask, set_mask)   \
        MHSCPU_WRITE_REG32(reg, (((MHSCPU_READ_REG32(reg)) & ~clear_mask) | set_mask))
        
/******************************************************************************/
/*                                                                            */
/*                                    DCMI                                    */
/*                                                                            */
/******************************************************************************/
/********************  Bits definition for DCMI_CR register  ******************/
#define DCMI_CR_CAPTURE                      ((uint32_t)0x00000001)
#define DCMI_CR_CM                           ((uint32_t)0x00000002)
#define DCMI_CR_CROP                         ((uint32_t)0x00000004)
#define DCMI_CR_JPEG                         ((uint32_t)0x00000008)
#define DCMI_CR_ESS                          ((uint32_t)0x00000010)
#define DCMI_CR_PCKPOL                       ((uint32_t)0x00000020)
#define DCMI_CR_HSPOL                        ((uint32_t)0x00000040)
#define DCMI_CR_VSPOL                        ((uint32_t)0x00000080)
#define DCMI_CR_FCRC_0                       ((uint32_t)0x00000100)
#define DCMI_CR_FCRC_1                       ((uint32_t)0x00000200)
#define DCMI_CR_EDM_0                        ((uint32_t)0x00000400)
#define DCMI_CR_EDM_1                        ((uint32_t)0x00000800)
#define DCMI_CR_CRE                          ((uint32_t)0x00001000)
#define DCMI_CR_ENABLE                       ((uint32_t)0x00004000)

/********************  Bits definition for DCMI_SR register  ******************/
#define DCMI_SR_HSYNC                        ((uint32_t)0x00000001)
#define DCMI_SR_VSYNC                        ((uint32_t)0x00000002)
#define DCMI_SR_FNE                          ((uint32_t)0x00000004)

/********************  Bits definition for DCMI_RISR register  ****************/
#define DCMI_RISR_FRAME_RIS                  ((uint32_t)0x00000001)
#define DCMI_RISR_OVF_RIS                    ((uint32_t)0x00000002)
#define DCMI_RISR_ERR_RIS                    ((uint32_t)0x00000004)
#define DCMI_RISR_VSYNC_RIS                  ((uint32_t)0x00000008)
#define DCMI_RISR_LINE_RIS                   ((uint32_t)0x00000010)

/********************  Bits definition for DCMI_IER register  *****************/
#define DCMI_IER_FRAME_IE                    ((uint32_t)0x00000001)
#define DCMI_IER_OVF_IE                      ((uint32_t)0x00000002)
#define DCMI_IER_ERR_IE                      ((uint32_t)0x00000004)
#define DCMI_IER_VSYNC_IE                    ((uint32_t)0x00000008)
#define DCMI_IER_LINE_IE                     ((uint32_t)0x00000010)

/********************  Bits definition for DCMI_MISR register  ****************/
#define DCMI_MISR_FRAME_MIS                  ((uint32_t)0x00000001)
#define DCMI_MISR_OVF_MIS                    ((uint32_t)0x00000002)
#define DCMI_MISR_ERR_MIS                    ((uint32_t)0x00000004)
#define DCMI_MISR_VSYNC_MIS                  ((uint32_t)0x00000008)
#define DCMI_MISR_LINE_MIS                   ((uint32_t)0x00000010)

/********************  Bits definition for DCMI_ICR register  *****************/
#define DCMI_ICR_FRAME_ISC                   ((uint32_t)0x00000001)
#define DCMI_ICR_OVF_ISC                     ((uint32_t)0x00000002)
#define DCMI_ICR_ERR_ISC                     ((uint32_t)0x00000004)
#define DCMI_ICR_VSYNC_ISC                   ((uint32_t)0x00000008)
#define DCMI_ICR_LINE_ISC                    ((uint32_t)0x00000010)
/*@}*/ /* end of group <Device>_PeripheralDecl */

/*@}*/ /* end of group <Device>_Definitions */

#ifdef USE_STDPERIPH_DRIVER
  #include "mhscpu_conf.h"
#endif

#ifdef __cplusplus
}
#endif

#endif  /* <Device>_H */
