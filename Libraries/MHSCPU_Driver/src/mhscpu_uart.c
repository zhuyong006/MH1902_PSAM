#include "mhscpu_uart.h"
#include "mhscpu_sysctrl.h"

extern void SYSCTRL_UART_Reset(UART_TypeDef* UARTx);

/**
  * @brief  Deinitializes the UARTx peripheral registers to their default reset values.
  * @param  UARTx: Select the UART peripheral. 
  *   This parameter can be one of the following values: 
  *      UART0, UART1, UART2 or UART3.
  * @retval None
  */
void UART_DeInit(UART_TypeDef* UARTx)
{
	/* Check the parameters */
	SYSCTRL_UART_Reset(UARTx);
}

void UART_Init(UART_TypeDef* UARTx, UART_InitTypeDef* UART_InitStruct)
{
	uint32_t tmpBaudRateDiv;
	SYSCTRL_ClocksTypeDef  clocks;
	
	/* Check the parameters */
	assert_param(IS_UART_BAUDRATE(UART_InitStruct->UART_BaudRate));  
	assert_param(IS_UART_WORD_LENGTH(UART_InitStruct->UART_WordLength));
	assert_param(IS_UART_STOPBITS(UART_InitStruct->UART_StopBits,UART_InitStruct->UART_WordLength));
	assert_param(IS_UART_PARITY(UART_InitStruct->UART_Parity));

	// get Clock Frequence
	SYSCTRL_GetClocksFreq(&clocks);
	
	// LCR = 1.
	UARTx->LCR |= UART_LCR_DLAB;
	
	// baud rate = (serial clock freq) / (16 * divisor).
	tmpBaudRateDiv = ((clocks.PCLK_Frequency / 16) + UART_InitStruct->UART_BaudRate / 2) / UART_InitStruct->UART_BaudRate;
	UARTx->OFFSET_0.DLL = (tmpBaudRateDiv & 0x00FF);
	UARTx->OFFSET_4.DLH = ((tmpBaudRateDiv >> 8) & 0x00FF);
	
	// LCR = 0.
	UARTx->LCR &= ~UART_LCR_DLAB;
	
	UARTx->LCR = UART_InitStruct->UART_WordLength | UART_InitStruct->UART_StopBits | UART_InitStruct->UART_Parity;
}


void UART_StructInit(UART_InitTypeDef* UART_InitStruct)
{
	/* UART_InitStruct members default value */
	UART_InitStruct->UART_BaudRate = 9600;
	UART_InitStruct->UART_WordLength = UART_WordLength_8b;
	UART_InitStruct->UART_StopBits = UART_StopBits_1;
	UART_InitStruct->UART_Parity = UART_Parity_No ;
}

void UART_ITConfig(UART_TypeDef* UARTx, uint32_t UART_IT, FunctionalState NewState)
{
	if(DISABLE != NewState)
	{
		UARTx->OFFSET_4.IER |= UART_IT;
	}
	else
	{
		UARTx->OFFSET_4.IER &= ~UART_IT;
	}
}

void UART_SendData(UART_TypeDef* UARTx, uint8_t Data)
{
	UARTx->OFFSET_0.THR = (Data & 0xFF);
}

uint8_t UART_ReceiveData(UART_TypeDef* UARTx)
{
	return (uint8_t)(UARTx->OFFSET_0.RBR & 0xFF);
}

void UART_AutoFlowCtrlCmd(UART_TypeDef* UARTx, FunctionalState NewState)
{
	if (DISABLE != NewState)
	{
		UARTx->MCR |= UART_MCR_AFCE;
	}
	else
	{
		UARTx->MCR &= ~UART_MCR_AFCE;
	}
}

void UART_SetDTR(UART_TypeDef* UARTx)
{
	UARTx->MCR |= UART_MCR_DTR;
}

void UART_ResetDTR(UART_TypeDef* UARTx)
{
	UARTx->MCR &= ~UART_MCR_DTR;
}

void UART_SetRTS(UART_TypeDef* UARTx)
{
	UARTx->MCR |= UART_MCR_RTS;
}

void UART_ResetRTS(UART_TypeDef* UARTx)
{
	UARTx->MCR &= ~UART_MCR_RTS;
}

void UART_FIFOInit(UART_TypeDef* UARTx, UART_FIFOInitTypeDef* UART_FIFOInitStruct)
{
	/**************************  FIFO Tx Interrupt Config ******************************/

	if (DISABLE != UART_FIFOInitStruct->FIFO_TX_TriggerIntEnable)
		UARTx->OFFSET_4.IER |= UART_IER_PTIME;
	else
		UARTx->OFFSET_4.IER &= ~UART_IER_PTIME;

	/**************************  FIFO Config ******************************/
	
	/* FCR Write Only So Here we Use FCR Shadow Register SDMAM(WR) */
	if (UARTx->SFE | UART_SFE_SFE)
		UARTx->SFE &= ~UART_SFE_SFE;

	if (UART_FIFO_DMA_Mode_0 == UART_FIFOInitStruct->FIFO_DMA_Mode)
	{
		UARTx->SDMAM &= ~UART_SDMAM_SDMAM;
	}
	else if(UART_FIFO_DMA_Mode_1 == UART_FIFOInitStruct->FIFO_DMA_Mode)
	{
		UARTx->SDMAM |= UART_SDMAM_SDMAM;
	}

	/* FCR Write Only So Here we Use FCR Shadow Register SRT and STET(WR) */
	UARTx->SRT = UART_FIFOInitStruct->FIFO_RX_Trigger;
	UARTx->STET = UART_FIFOInitStruct->FIFO_TX_Trigger;

	if (DISABLE != UART_FIFOInitStruct->FIFO_Enable)
	{
		UARTx->SFE |= UART_SFE_SFE;
	}
	else
	{
		UARTx->SFE &= ~UART_SFE_SFE;
	}
}
void UART_FIFOStructInit(UART_FIFOInitTypeDef* UART_FIFOInitStruct)
{
	UART_FIFOInitStruct->FIFO_Enable = DISABLE;
	UART_FIFOInitStruct->FIFO_DMA_Mode = UART_FIFO_DMA_Mode_0;
	UART_FIFOInitStruct->FIFO_RX_Trigger = UART_FIFO_RX_Trigger_1_2_Full;
	UART_FIFOInitStruct->FIFO_TX_Trigger = UART_FIFO_TX_Trigger_1_2_Full;
	UART_FIFOInitStruct->FIFO_TX_TriggerIntEnable = DISABLE;
}

void UART_FIFOReset(UART_TypeDef* UARTx, uint32_t UART_FIFO)
{
	uint32_t u32tmp = 0;

	if (0 != (UART_FIFO | UART_FIFO_TX))
		u32tmp |= UART_SRR_XFR;
	if (0 != (UART_FIFO | UART_FIFO_RX))
		u32tmp |= UART_SRR_RFR;

	/* FCR Write Only So Here we Use FCR Shadow Register SRR to protect FCR's value*/
	UARTx->SRR = u32tmp;
}

void UART_FIFOCmd(UART_TypeDef* UARTx,FunctionalState NewState)
{
	/* FCR Write Only So Here we Use FCR Shadow Register SFE(WR) */
	if (DISABLE != NewState)
		UARTx->SFE |= UART_SFE_SFE;
	else
		UARTx->SFE &= ~UART_SFE_SFE;
}

uint32_t UART_GetLineStatus(UART_TypeDef* UARTx)
{
	return (UARTx->LSR);
}

uint32_t UART_GetModemStatus(UART_TypeDef* UARTx)
{
	return (UARTx->MSR);
}

uint32_t UART_GetITIdentity(UART_TypeDef* UARTx)
{
	return (UARTx->OFFSET_8.IIR);
}

Boolean UART_IsRXFIFOError(UART_TypeDef* UARTx)
{
	if (0 != (UARTx->LSR & UART_LINE_STATUS_RX_FIFO_ERROR))
		return TRUE;
	else
		return FALSE;
}

Boolean UART_IsRXFramingError(UART_TypeDef* UARTx)
{
	if (0 != (UARTx->LSR & UART_LINE_STATUS_RX_FRAMING_ERROR))
		return TRUE;
	else
		return FALSE;
}

Boolean UART_IsRXParityError(UART_TypeDef* UARTx)
{
	if (0 != (UARTx->LSR & UART_LINE_STATUS_RX_PARITY_ERROR))
		return TRUE;
	else
		return FALSE;
}

Boolean UART_IsRXOverrunError(UART_TypeDef* UARTx)
{
	if (0 != (UARTx->LSR & UART_LINE_STATUS_RX_OVERRUN_ERROR))
		return TRUE;
	else
		return FALSE;
}

Boolean UART_IsRXReceived(UART_TypeDef* UARTx)
{
	if (0 != (UARTx->LSR & UART_LINE_STATUS_RX_RECVD))
		return TRUE;
	else
		return FALSE;
}

Boolean UART_IsTXEmpty(UART_TypeDef* UARTx)
{
	if (0 != (UARTx->LSR & UART_LINE_STATUS_TX_EMPTY))
		return TRUE;
	else
		return FALSE;
}

Boolean UART_IsTXHoldingRegisterEmpty(UART_TypeDef* UARTx)
{
	if (0 != (UARTx->LSR & UART_LINE_STATUS_TX_HOLDING_REGISTER_EMPTY))
		return TRUE;
	else
		return FALSE;
}

Boolean UART_IsTXFIFOTrigger(UART_TypeDef* UARTx)
{
	if (0 != (UARTx->LSR & UART_LINE_STATUS_TX_HOLDING_REGISTER_EMPTY))
		return TRUE;
	else
		return FALSE;
}


Boolean UART_IsRXFIFOFull(UART_TypeDef* UARTx)
{
	if (0 != (UARTx->USR & UART_STATUS_RX_FIFO_FULL))
		return TRUE;
	else
		return FALSE;
}

Boolean UART_IsRXFIFONotEmpty(UART_TypeDef* UARTx)
{
	if (0 != (UARTx->USR & UART_STATUS_RX_FIFO_NOT_EMPTY))
		return TRUE;
	else
		return FALSE;
}

Boolean UART_IsTXFIFOEmpty(UART_TypeDef* UARTx)
{
	if (0 != (UARTx->USR & UART_STATUS_TX_FIFO_EMPTY))
		return TRUE;
	else
		return FALSE;
}

Boolean UART_IsTXFIFONotFull(UART_TypeDef* UARTx)
{
	if (0 != (UARTx->USR & UART_STATUS_TX_FIFO_NOT_FULL))
		return TRUE;
	else
		return FALSE;
}

Boolean UART_IsUARTBusy(UART_TypeDef* UARTx)
{
	if (0 != (UARTx->USR & UART_STATUS_BUSY))
		return TRUE;
	else
		return FALSE;
}

void UART_DMAGenerateSoftAck(UART_TypeDef* UARTx)
{
	UARTx->DMASA |= UART_DMASA_DMASA;
}

void UART_TXHaltCmd(UART_TypeDef* UARTx, FunctionalState NewStatus)
{
	if (DISABLE != NewStatus)
		UARTx->HTX |= UART_HTX_HTX;
	else
		UARTx->HTX &= ~UART_HTX_HTX;
}

void UART_FIFOAccessModeCmd(UART_TypeDef* UARTx, FunctionalState NewStatus)
{
	if (DISABLE != NewStatus)
		UARTx->FAR |= UART_FAR_FAR;
	else
		UARTx->FAR &= ~UART_FAR_FAR;
}

uint8_t UART_FIFOTxRead(UART_TypeDef* UARTx)
{
	return (uint8_t)(UARTx->TFR & 0xFF);
}

void UART_IrDACmd(UART_TypeDef* UARTx, FunctionalState NewState)
{
	/* Check the parameters */
	assert_param(IS_FUNCTIONAL_STATE(NewState));

	if (NewState != DISABLE)
	{
		/* Enable the IrDA mode by setting the SIRE bit in the MCR register */
		UARTx->MCR |= UART_MCR_SIRE;
	}
	else
	{
		/* Disable the IrDA mode by clearing the SIRE bit in the MCR register */
		UARTx->MCR &= UART_MCR_SIRE;
	}
}

void UART_SendBreak(UART_TypeDef* UARTx)
{
	/* Send break characters */
	UARTx->SBCR |= UART_SBCR_SBCR;
}
