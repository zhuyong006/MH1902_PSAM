#include "mhscpu_spi.h"

extern void SYSCTRL_SPI_Reset(SPI_TypeDef* SPIx);
extern void SYSCTRL_SPI_MasterSlaveModeSet(SPI_TypeDef* SPIx);

void SPI_MasterSlaveModeSet(SPI_TypeDef* SPIx);

void SPI_DeInit(SPI_TypeDef* SPIx)
{
	SYSCTRL_SPI_Reset(SPIx);
}

void SPI_Init(SPI_TypeDef* SPIx, SPI_InitTypeDef* SPI_InitStruct)
{
	assert_param(IS_SPI_DIRECTION_MODE(SPI_InitStruct->SPI_Direction));
	assert_param(IS_SPI_DATASIZE(SPI_InitStruct->SPI_DataSize));
	assert_param(IS_SPI_NSS(SPI_InitStruct->SPI_NSS));
	assert_param(IS_SPI_CPOL(SPI_InitStruct->SPI_CPOL));
	assert_param(IS_SPI_CPHA(SPI_InitStruct->SPI_CPHA));
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_InitStruct->SPI_BaudRatePrescaler));
	assert_param(IS_SPI_RX_FIFO_FULL_THRESHOLD(SPI_InitStruct->SPI_RXFIFOFullThreshold));
	assert_param(IS_SPI_TX_FIFO_EMPTY_THRESHOLD(SPI_InitStruct->SPI_TXFIFOEmptyThreshold));

	SPIx->SSIENR = 0;		//DISABLE current SPI before configure CONTROL registers
	
	SPIx->IMR = 0;

	SPIx->CTRLR0 = (SPI_InitStruct->SPI_Direction | SPI_InitStruct->SPI_CPOL | SPI_InitStruct->SPI_CPHA | SPI_InitStruct->SPI_DataSize);
	
	SPIx->BAUDR = SPI_InitStruct->SPI_BaudRatePrescaler;
	
	SPIx->SER = SPI_InitStruct->SPI_NSS;

	SPIx->RXFTLR = SPI_InitStruct->SPI_RXFIFOFullThreshold;
	SPIx->TXFTLR = SPI_InitStruct->SPI_TXFIFOEmptyThreshold;

	SPI_MasterSlaveModeSet(SPIx);
}

void SPI_StructInit(SPI_InitTypeDef* SPI_InitStruct)
{
/*--------------- Reset SPI init structure parameters values -----------------*/
  /* Initialize the SPI_Direction member */
  SPI_InitStruct->SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  /* initialize the SPI_DataSize member */
  SPI_InitStruct->SPI_DataSize = SPI_DataSize_8b;
  /* Initialize the SPI_CPOL member */
  SPI_InitStruct->SPI_CPOL = SPI_CPOL_Low;
  /* Initialize the SPI_CPHA member */
  SPI_InitStruct->SPI_CPHA = SPI_CPHA_1Edge;
  /* Initialize the SPI_NSS member */
  SPI_InitStruct->SPI_NSS = SPI_NSS_0;
  /* Initialize the SPI_BaudRatePrescaler member */
  SPI_InitStruct->SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
  /* Initialize the SPI_RXFIFOFullThreshold member */
  SPI_InitStruct->SPI_RXFIFOFullThreshold = SPI_RXFIFOFullThreshold_1;
  /* Initialize the SPI_TXFIFOEmptyThreshold member */
  SPI_InitStruct->SPI_TXFIFOEmptyThreshold = SPI_TXFIFOEmptyThreshold_0;
}

void SPI_Cmd(SPI_TypeDef* SPIx, FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	if (NewState != DISABLE)
	{
		/* Enable the selected SPI peripheral */
		SPIx->SSIENR = 1;		//ENABLE current SPI
	}
	else
	{
		/* Disable the selected SPI peripheral */
		SPIx->SSIENR = 0;		//DISABLE current SPI
	}
}

void SPI_DMAInit(SPI_TypeDef* SPIx, SPI_DMAInitTypeDef* SPI_DMAInitStruct)
{
	assert_param(IS_SPI_DMAREQ(SPI_DMAInitStruct->SPI_DMAReq));
	assert_param(IS_SPI_DMA_RECEIVE_LEVEL(SPI_DMAInitStruct->SPI_DMAReceiveLevel));
	assert_param(IS_SPI_DMA_TRANSMIT_LEVEL(SPI_DMAInitStruct->SPI_DMATransmitLevel));
	assert_param(IS_FUNCTIONAL_STATE(SPI_DMAInitStruct->SPI_DMAEnCmd));
	SPIx->DMARDLR = SPI_DMAInitStruct->SPI_DMAReceiveLevel;
	SPIx->DMATDLR = SPI_DMAInitStruct->SPI_DMATransmitLevel;
	if (DISABLE != SPI_DMAInitStruct->SPI_DMAEnCmd)
	{
		SPIx->DMACR |= SPI_DMAInitStruct->SPI_DMAReq;
	} 
	else
	{
		SPIx->DMACR &= ~SPI_DMAInitStruct->SPI_DMAReq;
	}
}

void SPI_DMAStructInit(SPI_DMAInitTypeDef* SPI_DMAInitStruct)
{
	SPI_DMAInitStruct->SPI_DMAReq = SPI_DMAReq_Rx | SPI_DMAReq_Tx;
	/* Initialize the SPI_DMAReceiveLevel member */
	SPI_DMAInitStruct->SPI_DMAReceiveLevel = SPI_DMAReceiveLevel_8;
	/* Initialize the SPI_DMATransmitLevel member */
	SPI_DMAInitStruct->SPI_DMATransmitLevel = SPI_DMATransmitLevel_8;

	SPI_DMAInitStruct->SPI_DMAEnCmd = DISABLE;
}

void SPI_DMACmd(SPI_TypeDef* SPIx, uint32_t SPI_DMAReq, FunctionalState NewState)
{
	assert_param(IS_SPI_DMAREQ(SPI_DMAReq));
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	
	if (DISABLE != NewState)
	{
		SPIx->DMACR |= SPI_DMAReq;
	} 
	else
	{
		SPIx->DMACR &= ~SPI_DMAReq;
	}
}

void SPI_ITConfig(SPI_TypeDef* SPIx, uint32_t SPI_IT, FunctionalState NewState)
{
	assert_param(IS_SPI_GET_IT(SPI_IT));
	assert_param(IS_FUNCTIONAL_STATE(NewState));
	if(NewState != DISABLE)
		SPIx->IMR |= SPI_IT;
	else
		SPIx->IMR &= ~SPI_IT;
}


void SPI_SendData(SPI_TypeDef* SPIx, uint16_t Data)
{
	/* Write in the DR register the data to be sent */
	SPIx->DR = Data;
}

uint16_t SPI_ReceiveData(SPI_TypeDef* SPIx)
{
	/* Return the data in the DR register */
	return (uint16_t)SPIx->DR;
}

void SPI_DataSizeConfig(SPI_TypeDef* SPIx, uint32_t SPI_DataSize)
{
	assert_param(IS_SPI_DATASIZE(SPI_DataSize));
	SPIx->CTRLR0 &= ~SPI_CTRLR0_DFS;
	SPIx->CTRLR0 |= SPI_DataSize;
}

void SPI_BiDirectionalLineConfig(SPI_TypeDef* SPIx, uint32_t SPI_Direction)
{
	assert_param(IS_SPI_DIRECTION_MODE(SPI_Direction)); 
	SPIx->CTRLR0 &= ~SPI_CTRLR0_TMOD;
	SPIx->CTRLR0 |= SPI_Direction;
}

ITStatus SPI_GetITStatus(SPI_TypeDef* SPIx, uint32_t SPI_IT)
{
	ITStatus bitstatus = RESET; 
	assert_param(IS_SPI_GET_IT(SPI_IT));
	if ((SPIx->ISR & SPI_IT) != RESET)
	{
		bitstatus = SET;
	}
	else
	{
		bitstatus = RESET;
	}
	return bitstatus;
}

void SPI_ClearITPendingBit(SPI_TypeDef* SPIx, uint32_t SPI_IT)
{
	assert_param(IS_SPI_GET_IT(SPI_IT));
	if (SPI_IT_MMC | SPI_IT)
	{
		SPIx->MSTICR;
	}
	if (SPI_IT_RXUDF | SPI_IT)
	{
		SPIx->RXUICR;
	}
	if (SPI_IT_RXOVF | SPI_IT)
	{
		SPIx->RXOICR;
	}
	if (SPI_IT_TXOVF | SPI_IT)
	{
		SPIx->TXOICR;
	}
}

FlagStatus SPI_GetFlagStatus(SPI_TypeDef* SPIx, uint32_t SPI_FLAG)
{
	FlagStatus bitstatus = RESET; 
	assert_param(IS_SPI_GET_FLAG(SPI_FLAG));
  
	if ((SPIx->SR & SPI_FLAG) != RESET)
	{
		bitstatus = SET;
	}
	else
	{
		bitstatus = RESET;
	}
	return bitstatus;
}

uint32_t SPI_GetFlagStatusReg(SPI_TypeDef* SPIx)
{
	return ((uint32_t)SPIx->SR);
}

FlagStatus SPI_IsBusy(SPI_TypeDef* SPIx)
{
	FlagStatus bitstatus = RESET;

	if ((SPIx->SR & SPI_SR_BUSY) != RESET)
	{
		bitstatus = SET;
	}
	else
	{
		bitstatus = RESET;
	}
	return bitstatus;
}

FlagStatus SPI_IsTXErr(SPI_TypeDef* SPIx)
{
	FlagStatus bitstatus = RESET;

	if ((SPIx->SR & SPI_SR_TXE) != RESET)
	{
		bitstatus = SET;
	}
	else
	{
		bitstatus = RESET;
	}
	return bitstatus;
}

FlagStatus SPI_IsDataCollisionErr(SPI_TypeDef* SPIx)
{
	FlagStatus bitstatus = RESET;   

	if ((SPIx->SR & SPI_SR_DCOL) != RESET)
	{
		bitstatus = SET;
	}
	else
	{
		bitstatus = RESET;
	}
	return bitstatus;
}

void SSP_Init(SPI_TypeDef* SPIx, SSP_InitTypeDef* SSP_InitStruct)
{
	assert_param(IS_SPI_DIRECTION_MODE(SSP_InitStruct->SSP_Direction));
	assert_param(IS_SPI_DATASIZE(SSP_InitStruct->SSP_DataSize));
	assert_param(IS_SPI_NSS(SSP_InitStruct->SSP_NSS));
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SSP_InitStruct->SSP_BaudRatePrescaler));
	assert_param(IS_SPI_RX_FIFO_FULL_THRESHOLD(SSP_InitStruct->SSP_RXFIFOFullThreshold));
	assert_param(IS_SPI_TX_FIFO_EMPTY_THRESHOLD(SSP_InitStruct->SSP_TXFIFOEmptyThreshold));

	SPIx->SSIENR = 0;		//DISABLE current SPI before configure CONTROL registers

	SPIx->RXFTLR = SSP_InitStruct->SSP_RXFIFOFullThreshold;
	SPIx->TXFTLR = SSP_InitStruct->SSP_TXFIFOEmptyThreshold;

	SPIx->CTRLR0 = (SSP_InitStruct->SSP_Direction | SSP_InitStruct->SSP_DataSize);

	SPIx->BAUDR = SSP_InitStruct->SSP_BaudRatePrescaler;

	SPIx->SER = SSP_InitStruct->SSP_NSS;

	SPIx->CTRLR0 |= SPI_CTRLR0_FRF_0;			//SSI set to SSP mode

	SPI_MasterSlaveModeSet(SPIx);
}

void SSP_StructInit(SSP_InitTypeDef* SSP_InitStruct)
{
	/*--------------- Reset SSP init structure parameters values -----------------*/
	/* Initialize the SSP_Direction member */
	SSP_InitStruct->SSP_Direction = SPI_Direction_2Lines_FullDuplex;
	/* initialize the SSP_DataSize member */
	SSP_InitStruct->SSP_DataSize = SPI_DataSize_8b;
	/* Initialize the SSP_NSS member */
	SSP_InitStruct->SSP_NSS = SPI_NSS_0;
	/* Initialize the SSP_BaudRatePrescaler member */
	SSP_InitStruct->SSP_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	/* Initialize the SSP_RXFIFOFullThreshold member */
	SSP_InitStruct->SSP_RXFIFOFullThreshold = SPI_RXFIFOFullThreshold_1;
	/* Initialize the SSP_TXFIFOEmptyThreshold member */
	SSP_InitStruct->SSP_TXFIFOEmptyThreshold = SPI_TXFIFOEmptyThreshold_0;
}

void NSM_Init(SPI_TypeDef* SPIx, NSM_InitTypeDef* NSM_InitStruct)
{
	assert_param(IS_NSM_DIRECTION_MODE(NSM_InitStruct->NSM_Direction));
	assert_param(IS_NSM_TRANSFER_MODE(NSM_InitStruct->NSM_TransferMode));
	assert_param(IS_NSM_DATASIZE(NSM_InitStruct->NSM_DataSize));
	assert_param(IS_NSM_CONTROL_DATASIZE(NSM_InitStruct->NSM_ControlDataSize));
	assert_param(IS_SPI_NSS(NSM_InitStruct->NSM_NSS));
	assert_param(IS_SPI_BAUDRATE_PRESCALER(NSM_InitStruct->NSM_BaudRatePrescaler));
	assert_param(IS_FUNCTIONAL_STATE(NSM_InitStruct->NSM_HandShakingCmd));
	assert_param(IS_SPI_RX_FIFO_FULL_THRESHOLD(NSM_InitStruct->NSM_RXFIFOFullThreshold));
	assert_param(IS_SPI_TX_FIFO_EMPTY_THRESHOLD(NSM_InitStruct->NSM_TXFIFOEmptyThreshold));

	SPIx->SSIENR = 0;		//DISABLE current SPI before configure CONTROL registers

	SPIx->RXFTLR = NSM_InitStruct->NSM_RXFIFOFullThreshold;
	SPIx->TXFTLR = NSM_InitStruct->NSM_TXFIFOEmptyThreshold;

	SPIx->CTRLR0 = ((NSM_InitStruct->NSM_ControlDataSize) << 12 | NSM_InitStruct->NSM_DataSize);

	SPIx->BAUDR = NSM_InitStruct->NSM_BaudRatePrescaler;

	SPIx->SER = NSM_InitStruct->NSM_NSS;

	if (DISABLE != NSM_InitStruct->NSM_HandShakingCmd)
	{
		SPIx->MWCR |= SPI_MWCR_MHS;
	} 
	else
	{
		SPIx->MWCR &= ~SPI_MWCR_MHS;
	}
	if (NSM_Direction_Data_Transmit == NSM_InitStruct->NSM_Direction)
	{
		SPIx->MWCR |= SPI_MWCR_MDD;
	} 
	else if (NSM_Direction_Data_Receive == NSM_InitStruct->NSM_Direction)
	{
		SPIx->MWCR &= ~SPI_MWCR_MDD;
	}
	if (NSM_TransferMode_Sequential == NSM_InitStruct->NSM_TransferMode)
	{
		SPIx->MWCR |= SPI_MWCR_MWMOD;
	} 
	else if (NSM_TransferMode_Non_Sequential == NSM_InitStruct->NSM_TransferMode)
	{
		SPIx->MWCR &= ~SPI_MWCR_MWMOD;
	}

	SPIx->CTRLR0 |= SPI_CTRLR0_FRF_1;			//SSI set to NSM mode

	SPI_MasterSlaveModeSet(SPIx);
}

void NSM_StructInit(NSM_InitTypeDef* NSM_InitStruct)
{
	/*--------------- Reset NSM init structure parameters values -----------------*/
	/* Initialize the NSM_Direction member */
	NSM_InitStruct->NSM_Direction = NSM_Direction_Data_Receive;
	/* Initialize the NSM_Direction member */
	NSM_InitStruct->NSM_TransferMode = NSM_TransferMode_Non_Sequential;
	/* initialize the NSM_DataSize member */
	NSM_InitStruct->NSM_DataSize = NSM_DataSize_8b;
	/* initialize the NSM_DataSize member */
	NSM_InitStruct->NSM_ControlDataSize = NSM_ControlDataSize_8b;
	/* Initialize the NSM_NSS member */
	NSM_InitStruct->NSM_NSS = SPI_NSS_0;
	/* Initialize the NSM_BaudRatePrescaler member */
	NSM_InitStruct->NSM_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	/* Initialize the NSM_HandShakingCmd */
	NSM_InitStruct->NSM_HandShakingCmd = DISABLE;
	/* Initialize the NSM_RXFIFOFullThreshold member */
	NSM_InitStruct->NSM_RXFIFOFullThreshold = SPI_RXFIFOFullThreshold_1;
	/* Initialize the NSM_TXFIFOEmptyThreshold member */
	NSM_InitStruct->NSM_TXFIFOEmptyThreshold = SPI_TXFIFOEmptyThreshold_0;
}

void SPI_MasterSlaveModeSet(SPI_TypeDef* SPIx)
{
	SYSCTRL_SPI_MasterSlaveModeSet(SPIx);
}
