#ifndef __MHSCPU_REC_H
#define __MHSCPU_REC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "mhscpu.h"


typedef struct
{
	uint32_t LoadCount[TIM_NUM];
	uint32_t ReloadCount[TIM_NUM];
	uint32_t ControlReg[TIM_NUM];
}TIM_ConfigTypeDef;

/**
  * @method	TIM_ConfigBackup
  * @brief	
  * @param	
  * @retval 
  */
__attribute__( ( always_inline ) ) __STATIC_INLINE void TIM_ConfigBackup(TIM_ConfigTypeDef *Config, TIM_Module_TypeDef* TIMMx)
{
	uint32_t i;
	for (i = 0; i < TIM_NUM; i++)
	{
		Config->LoadCount[i] = TIMMx->TIM[i].LoadCount;
		Config->ReloadCount[i] = TIMMx->TIM_ReloadCount[i];
		Config->ControlReg[i] = TIMMx->TIM[i].ControlReg;
	}
}

/**
  * @method	TIM_ConfigRecover
  * @brief	
  * @param	
  * @retval 
  */
__attribute__( ( always_inline ) ) __STATIC_INLINE void TIM_ConfigRecover(TIM_Module_TypeDef* TIMMx, TIM_ConfigTypeDef *Config)
{
	uint32_t i;
	for (i = 0; i < TIM_NUM; i++)
	{
		TIMMx->TIM[i].LoadCount = Config->LoadCount[i];
		TIMMx->TIM_ReloadCount[i] = Config->ReloadCount[i];
		TIMMx->TIM[i].ControlReg = Config->ControlReg[i];
	}
}


typedef struct
{
	uint16_t CTRLR0;
	uint16_t CTRLR1;
	uint32_t SSIENR;
	uint32_t MWCR;
	uint32_t SER;
	uint32_t BAUDR;
	uint32_t TXFTLR;
	uint32_t RXFTLR;
	uint32_t IMR;
	uint32_t DMACR;
	uint32_t DMATDLR;
	uint32_t DMARDLR;
}SPI_ConfigTypeDef;

/**
  * @method	SPI_ConfigBackup
  * @brief	
  * @param	
  * @retval 
  */
__attribute__( ( always_inline ) ) __STATIC_INLINE void SPI_ConfigBackup(SPI_ConfigTypeDef *Config, SPI_TypeDef* SPIx)
{
	Config->SSIENR = SPIx->SSIENR;
	Config->CTRLR0 = SPIx->CTRLR0;
	Config->CTRLR1 = SPIx->CTRLR1;
	Config->MWCR = SPIx->MWCR;
	Config->SER = SPIx->SER;
	Config->BAUDR = SPIx->BAUDR;
	Config->TXFTLR = SPIx->TXFTLR;
	Config->RXFTLR = SPIx->RXFTLR;
	Config->IMR = SPIx->IMR;
	Config->DMACR = SPIx->DMACR;
	Config->DMATDLR = SPIx->DMATDLR;
	Config->DMARDLR = SPIx->DMARDLR;
}

/**
  * @method	SPI_ConfigRecover
  * @brief	
  * @param	
  * @retval 
  */
__attribute__( ( always_inline ) ) __STATIC_INLINE void SPI_ConfigRecover(SPI_TypeDef* SPIx, SPI_ConfigTypeDef *Config)
{
	SPIx->SSIENR = 0;		//DISABLE current SPI before configure CONTROL registers
	SPIx->CTRLR0 = Config->CTRLR0;
	SPIx->CTRLR1 = Config->CTRLR1;
	SPIx->MWCR = Config->MWCR;
	SPIx->SER = Config->SER;
	SPIx->BAUDR = Config->BAUDR;
	SPIx->TXFTLR = Config->TXFTLR;
	SPIx->RXFTLR = Config->RXFTLR;
	SPIx->IMR = Config->IMR;
	SPIx->DMACR = Config->DMACR;
	SPIx->DMATDLR = Config->DMATDLR;
	SPIx->DMARDLR = Config->DMARDLR;
	SPIx->SSIENR = Config->SSIENR;
}


typedef struct
{
	uint32_t DLL;
	uint32_t DLH;
	uint32_t IER;
	uint32_t LCR;
	uint32_t MCR;

	/*uint32_t FCR;*/
	uint32_t SDMAM;
	uint32_t SFE;
	uint32_t SRT;
	uint32_t STET;

}UART_ConfigTypeDef;

/**
  * @method	UART_ConfigBackup
  * @brief	
  * @param	
  * @retval 
  */
__attribute__( ( always_inline ) ) __STATIC_INLINE void UART_ConfigBackup(UART_ConfigTypeDef *Config, UART_TypeDef *UARTx, uint32_t BaudRate)
{	
	uint32_t tmpBaudRateDiv;
	SYSCTRL_ClocksTypeDef  clocks;
	
	// get Clock Frequence
	SYSCTRL_GetClocksFreq(&clocks);
	// baud rate = (serial clock freq) / (16 * divisor).
	tmpBaudRateDiv = ((clocks.PCLK_Frequency / 16) + BaudRate / 2) / BaudRate;
	Config->DLL = (tmpBaudRateDiv & 0x00FF);
	Config->DLH = ((tmpBaudRateDiv >> 8) & 0x00FF);

	Config->IER = UARTx->OFFSET_4.IER;
	Config->LCR = UARTx->LCR;
	Config->MCR = UARTx->MCR;

	Config->SDMAM = UARTx->SDMAM;
	Config->SFE = UARTx->SFE;
	Config->SRT = UARTx->SRT;
	Config->STET = UARTx->STET;
}


/**
  * @method	UART_ConfigRecover
  * @brief	
  * @param	
  * @retval 
  */
__attribute__( ( always_inline ) ) __STATIC_INLINE void UART_ConfigRecover(UART_TypeDef *UARTx, UART_ConfigTypeDef *Config)
{
	// LCR = 1.
	UARTx->LCR |= UART_LCR_DLAB;
	UARTx->OFFSET_0.DLL = Config->DLL;
	UARTx->OFFSET_4.DLH = Config->DLH;
	// LCR = 0.
	UARTx->LCR &= ~UART_LCR_DLAB;

	UARTx->SDMAM = Config->SDMAM;
	UARTx->SFE = Config->SFE;
	UARTx->SRT = Config->SRT;
	UARTx->STET = Config->STET;

	UARTx->OFFSET_4.IER = Config->IER;
	UARTx->LCR = Config->LCR;
	UARTx->MCR = Config->MCR;
}
	 
	 

typedef struct
{
	uint32_t SCI_CR0;    
	uint32_t SCI_CR1;
	uint32_t SCI_CR2;
	uint32_t SCI_RETRY;
	uint32_t SCI_RXTIME;
	uint32_t SCI_STABLE;
	uint32_t SCI_ATIME;
	uint32_t SCI_DTIME;

	uint32_t SCI_ATRSTIME;
	uint32_t SCI_ATRDTIME;
	uint32_t SCI_BLKTIME;
	uint32_t SCI_CHTIME;
	uint32_t SCI_CLKICC;
	uint32_t SCI_BAUD;
	uint32_t SCI_VALUE;
	uint32_t SCI_CHGUARD;
	uint32_t SCI_BLKGUARD;
}SCI_ConfigTypeDef;

/**
  * @method	SCI_ConfigBackup
  * @brief	
  * @param	
  * @retval 
  */
__attribute__( ( always_inline ) ) __STATIC_INLINE void SCI_ConfigBackup(SCI_ConfigTypeDef *Config, SCI_TypeDef * SCIx)
{
	Config->SCI_CR0 = SCIx->SCI_CR0;    
	Config->SCI_CR1 = SCIx->SCI_CR1;
	Config->SCI_CR2 = SCIx->SCI_CR2;
	Config->SCI_RETRY = SCIx->SCI_RETRY;
	Config->SCI_RXTIME = SCIx->SCI_RXTIME;
	Config->SCI_STABLE = SCIx->SCI_STABLE;
	Config->SCI_ATIME = SCIx->SCI_ATIME;
	Config->SCI_DTIME = SCIx->SCI_DTIME;

	Config->SCI_ATRSTIME = SCIx->SCI_ATRSTIME;
	Config->SCI_ATRDTIME = SCIx->SCI_ATRDTIME;
	Config->SCI_BLKTIME = SCIx->SCI_BLKTIME;
	Config->SCI_CHTIME = SCIx->SCI_CHTIME;
	Config->SCI_CLKICC = SCIx->SCI_CLKICC;
	Config->SCI_BAUD = SCIx->SCI_BAUD;
	Config->SCI_VALUE = SCIx->SCI_VALUE;
	Config->SCI_CHGUARD = SCIx->SCI_CHGUARD;
	Config->SCI_BLKGUARD = SCIx->SCI_BLKGUARD;
}

/**
  * @method	SCI_ConfigRecover
  * @brief	
  * @param	
  * @retval 
  */
__attribute__( ( always_inline ) ) __STATIC_INLINE void SCI_ConfigRecover(SCI_TypeDef * SCIx, SCI_ConfigTypeDef *Config)
{
	SCIx->SCI_CR0 = Config->SCI_CR0;    
	SCIx->SCI_CR1 = Config->SCI_CR1;
	SCIx->SCI_CR2 = Config->SCI_CR2;
	SCIx->SCI_RETRY = Config->SCI_RETRY;
	SCIx->SCI_RXTIME = Config->SCI_RXTIME;
	SCIx->SCI_STABLE = Config->SCI_STABLE;
	SCIx->SCI_ATIME = Config->SCI_ATIME;
	SCIx->SCI_DTIME = Config->SCI_DTIME;

	SCIx->SCI_ATRSTIME = Config->SCI_ATRSTIME;
	SCIx->SCI_ATRDTIME = Config->SCI_ATRDTIME;
	SCIx->SCI_BLKTIME = Config->SCI_BLKTIME;
	SCIx->SCI_CHTIME = Config->SCI_CHTIME;
	SCIx->SCI_CLKICC = Config->SCI_CLKICC;
	SCIx->SCI_BAUD = Config->SCI_BAUD;
	SCIx->SCI_VALUE = Config->SCI_VALUE;
	SCIx->SCI_CHGUARD = Config->SCI_CHGUARD;
	SCIx->SCI_BLKGUARD = Config->SCI_BLKGUARD;
}


typedef struct
{
	uint32_t WDT_CR;
	uint32_t WDT_CRR;
	uint32_t WDT_RLD;
}WDT_ConfigTypeDef;


/**
  * @method	WDT_ConfigBackup
  * @brief	
  * @param	
  * @retval 
  */
__attribute__( ( always_inline ) ) __STATIC_INLINE void WDT_ConfigBackup(WDT_ConfigTypeDef *Config)
{
	Config->WDT_RLD = WDT->WDT_RLD;
	Config->WDT_CR = WDT->WDT_CR;
	Config->WDT_CRR = 0x76;
}

/**
  * @method	WDT_ConfigRecover
  * @brief	
  * @param	
  * @retval 
  */
__attribute__( ( always_inline ) ) __STATIC_INLINE void WDT_ConfigRecover(WDT_ConfigTypeDef *Config)
{
	WDT->WDT_RLD = Config->WDT_RLD;
	WDT->WDT_CR = Config->WDT_CR;
	WDT->WDT_CRR = Config->WDT_CRR;
}


typedef struct
{
	uint32_t lcdi_ctrl;
	uint32_t lcdi_cycle;
}LCD_ConfigTypeDef;

/**
  * @method	LCD_ConfigBackup
  * @brief	
  * @param	
  * @retval 
  */
__attribute__( ( always_inline ) ) __STATIC_INLINE void LCD_ConfigBackup(LCD_ConfigTypeDef *Config)
{
	Config->lcdi_ctrl = LCD->lcdi_ctrl;
	Config->lcdi_cycle = LCD->lcdi_cycle;
}

/**
  * @method	LCD_ConfigRecover
  * @brief	
  * @param	
  * @retval 
  */
__attribute__( ( always_inline ) ) __STATIC_INLINE void LCD_ConfigRecover(LCD_ConfigTypeDef *Config)
{
	LCD->lcdi_ctrl = Config->lcdi_ctrl;
	LCD->lcdi_cycle = Config->lcdi_cycle;
}


typedef struct
{
	DMA_TypeDef DMA_Channel[4]; 
	uint32_t DmaCfgReg_L;
	uint32_t DmaCfgReg_H;
}DMA_ConfigTypeDef;

/**
  * @method	DMA_ConfigBackup
  * @brief	
  * @param	
  * @retval 
  */
__attribute__( ( always_inline ) ) __STATIC_INLINE void DMA_ConfigBackup(DMA_ConfigTypeDef *Config)
{
// 	memcpy(Config->DMA_Channel, DMA->DMA_Channel, sizeof(DMA_TypeDef));
	Config->DMA_Channel[0].SAR_L = DMA->DMA_Channel[0].SAR_L;
	Config->DMA_Channel[0].SAR_H = DMA->DMA_Channel[0].SAR_H;
	Config->DMA_Channel[0].DAR_L = DMA->DMA_Channel[0].DAR_L;
	Config->DMA_Channel[0].DAR_H = DMA->DMA_Channel[0].DAR_H;
	Config->DMA_Channel[0].LLP_L = DMA->DMA_Channel[0].LLP_L;
	Config->DMA_Channel[0].LLP_H = DMA->DMA_Channel[0].LLP_H;
	Config->DMA_Channel[0].CTL_L = DMA->DMA_Channel[0].CTL_L;
	Config->DMA_Channel[0].CTL_H = DMA->DMA_Channel[0].CTL_H;
	Config->DMA_Channel[0].CFG_L = DMA->DMA_Channel[0].CFG_L;
	Config->DMA_Channel[0].CFG_H = DMA->DMA_Channel[0].CFG_H;
	
	Config->DMA_Channel[1].SAR_L = DMA->DMA_Channel[1].SAR_L;
	Config->DMA_Channel[1].SAR_H = DMA->DMA_Channel[1].SAR_H;
	Config->DMA_Channel[1].DAR_L = DMA->DMA_Channel[1].DAR_L;
	Config->DMA_Channel[1].DAR_H = DMA->DMA_Channel[1].DAR_H;
	Config->DMA_Channel[1].LLP_L = DMA->DMA_Channel[1].LLP_L;
	Config->DMA_Channel[1].LLP_H = DMA->DMA_Channel[1].LLP_H;
	Config->DMA_Channel[1].CTL_L = DMA->DMA_Channel[1].CTL_L;
	Config->DMA_Channel[1].CTL_H = DMA->DMA_Channel[1].CTL_H;
	Config->DMA_Channel[1].CFG_L = DMA->DMA_Channel[1].CFG_L;
	Config->DMA_Channel[1].CFG_H = DMA->DMA_Channel[1].CFG_H;
	
	Config->DMA_Channel[2].SAR_L = DMA->DMA_Channel[2].SAR_L;
	Config->DMA_Channel[2].SAR_H = DMA->DMA_Channel[2].SAR_H;
	Config->DMA_Channel[2].DAR_L = DMA->DMA_Channel[2].DAR_L;
	Config->DMA_Channel[2].DAR_H = DMA->DMA_Channel[2].DAR_H;
	Config->DMA_Channel[2].LLP_L = DMA->DMA_Channel[2].LLP_L;
	Config->DMA_Channel[2].LLP_H = DMA->DMA_Channel[2].LLP_H;
	Config->DMA_Channel[2].CTL_L = DMA->DMA_Channel[2].CTL_L;
	Config->DMA_Channel[2].CTL_H = DMA->DMA_Channel[2].CTL_H;
	Config->DMA_Channel[2].CFG_L = DMA->DMA_Channel[2].CFG_L;
	Config->DMA_Channel[2].CFG_H = DMA->DMA_Channel[2].CFG_H;
	
	Config->DMA_Channel[3].SAR_L = DMA->DMA_Channel[3].SAR_L;
	Config->DMA_Channel[3].SAR_H = DMA->DMA_Channel[3].SAR_H;
	Config->DMA_Channel[3].DAR_L = DMA->DMA_Channel[3].DAR_L;
	Config->DMA_Channel[3].DAR_H = DMA->DMA_Channel[3].DAR_H;
	Config->DMA_Channel[3].LLP_L = DMA->DMA_Channel[3].LLP_L;
	Config->DMA_Channel[3].LLP_H = DMA->DMA_Channel[3].LLP_H;
	Config->DMA_Channel[3].CTL_L = DMA->DMA_Channel[3].CTL_L;
	Config->DMA_Channel[3].CTL_H = DMA->DMA_Channel[3].CTL_H;
	Config->DMA_Channel[3].CFG_L = DMA->DMA_Channel[3].CFG_L;
	Config->DMA_Channel[3].CFG_H = DMA->DMA_Channel[3].CFG_H;
	
	Config->DmaCfgReg_H = DMA->DmaCfgReg_H;
	Config->DmaCfgReg_L = DMA->DmaCfgReg_L;
}

/**
  * @method	DMA_ConfigRecover
  * @brief	
  * @param	
  * @retval 
  */
__attribute__( ( always_inline ) ) __STATIC_INLINE void DMA_ConfigRecover(DMA_ConfigTypeDef *Config)
{
// 	memcpy(DMA->DMA_Channel, Config->DMA_Channel, sizeof(DMA_TypeDef)*4);
	DMA->DMA_Channel[0].SAR_L = Config->DMA_Channel[0].SAR_L;
	DMA->DMA_Channel[0].SAR_H = Config->DMA_Channel[0].SAR_H;
	DMA->DMA_Channel[0].DAR_L = Config->DMA_Channel[0].DAR_L;
	DMA->DMA_Channel[0].DAR_H = Config->DMA_Channel[0].DAR_H;
	DMA->DMA_Channel[0].LLP_L = Config->DMA_Channel[0].LLP_L;
	DMA->DMA_Channel[0].LLP_H = Config->DMA_Channel[0].LLP_H;
	DMA->DMA_Channel[0].CTL_L = Config->DMA_Channel[0].CTL_L;
	DMA->DMA_Channel[0].CTL_H = Config->DMA_Channel[0].CTL_H;
	DMA->DMA_Channel[0].CFG_L = Config->DMA_Channel[0].CFG_L;
	DMA->DMA_Channel[0].CFG_H = Config->DMA_Channel[0].CFG_H;
	
	DMA->DMA_Channel[1].SAR_L = Config->DMA_Channel[1].SAR_L;
	DMA->DMA_Channel[1].SAR_H = Config->DMA_Channel[1].SAR_H;
	DMA->DMA_Channel[1].DAR_L = Config->DMA_Channel[1].DAR_L;
	DMA->DMA_Channel[1].DAR_H = Config->DMA_Channel[1].DAR_H;
	DMA->DMA_Channel[1].LLP_L = Config->DMA_Channel[1].LLP_L;
	DMA->DMA_Channel[1].LLP_H = Config->DMA_Channel[1].LLP_H;
	DMA->DMA_Channel[1].CTL_L = Config->DMA_Channel[1].CTL_L;
	DMA->DMA_Channel[1].CTL_H = Config->DMA_Channel[1].CTL_H;
	DMA->DMA_Channel[1].CFG_L = Config->DMA_Channel[1].CFG_L;
	DMA->DMA_Channel[1].CFG_H = Config->DMA_Channel[1].CFG_H;
	
	DMA->DMA_Channel[2].SAR_L = Config->DMA_Channel[2].SAR_L;
	DMA->DMA_Channel[2].SAR_H = Config->DMA_Channel[2].SAR_H;
	DMA->DMA_Channel[2].DAR_L = Config->DMA_Channel[2].DAR_L;
	DMA->DMA_Channel[2].DAR_H = Config->DMA_Channel[2].DAR_H;
	DMA->DMA_Channel[2].LLP_L = Config->DMA_Channel[2].LLP_L;
	DMA->DMA_Channel[2].LLP_H = Config->DMA_Channel[2].LLP_H;
	DMA->DMA_Channel[2].CTL_L = Config->DMA_Channel[2].CTL_L;
	DMA->DMA_Channel[2].CTL_H = Config->DMA_Channel[2].CTL_H;
	DMA->DMA_Channel[2].CFG_L = Config->DMA_Channel[2].CFG_L;
	DMA->DMA_Channel[2].CFG_H = Config->DMA_Channel[2].CFG_H;
	
	DMA->DMA_Channel[3].SAR_L = Config->DMA_Channel[3].SAR_L;
	DMA->DMA_Channel[3].SAR_H = Config->DMA_Channel[3].SAR_H;
	DMA->DMA_Channel[3].DAR_L = Config->DMA_Channel[3].DAR_L;
	DMA->DMA_Channel[3].DAR_H = Config->DMA_Channel[3].DAR_H;
	DMA->DMA_Channel[3].LLP_L = Config->DMA_Channel[3].LLP_L;
	DMA->DMA_Channel[3].LLP_H = Config->DMA_Channel[3].LLP_H;
	DMA->DMA_Channel[3].CTL_L = Config->DMA_Channel[3].CTL_L;
	DMA->DMA_Channel[3].CTL_H = Config->DMA_Channel[3].CTL_H;
	DMA->DMA_Channel[3].CFG_L = Config->DMA_Channel[3].CFG_L;
	DMA->DMA_Channel[3].CFG_H = Config->DMA_Channel[3].CFG_H;
	
	DMA->DmaCfgReg_H = Config->DmaCfgReg_H;
	DMA->DmaCfgReg_L = Config->DmaCfgReg_L;
}


typedef struct
{
	uint32_t RNG_CSR;
	uint32_t RNG0_DATA;
	uint32_t RNG_AMA;
	uint32_t RNG_PN;
}TRNG_ConfigTypeDef;

/**
  * @method	TRNG_ConfigBackup
  * @brief	
  * @param	Config
  * @retval 
  */
__attribute__( ( always_inline ) ) __STATIC_INLINE void TRNG_ConfigBackup(TRNG_ConfigTypeDef *Config)
{
	Config->RNG_CSR = TRNG->RNG_CSR & TRNG_RNG_CSR_INTP_EN_Mask;
	Config->RNG_AMA = TRNG->RNG_AMA;
	Config->RNG_PN = TRNG->RNG_PN;
}

/**
  * @method	TRNG_ConfigRecover
  * @brief	
  * @param	Config
  * @retval 
  */
__attribute__( ( always_inline ) ) __STATIC_INLINE void TRNG_ConfigRecover(TRNG_ConfigTypeDef *Config)
{
	TRNG->RNG_CSR &= ~TRNG_RNG_CSR_INTP_EN_Mask;
	TRNG->RNG_CSR |= Config->RNG_CSR;
	TRNG->RNG_AMA = Config->RNG_AMA;
	TRNG->RNG_PN = Config->RNG_PN;
}



#ifdef __cplusplus
}
#endif

#endif 
