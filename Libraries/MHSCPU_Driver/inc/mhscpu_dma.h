#ifndef __MHSCPU_DMA_H
#define __MHSCPU_DMA_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "mhscpu.h"

typedef struct
{
  uint32_t DMA_Peripheral;

  uint32_t DMA_PeripheralBaseAddr; /*!< Specifies the peripheral base address for DMAy Channelx. */

  uint32_t DMA_MemoryBaseAddr;     /*!< Specifies the memory base address for DMAy Channelx. */

  uint32_t DMA_DIR;                /*!< Specifies if the peripheral is the source or destination.
                                        This parameter can be a value of @ref DMA_data_transfer_direction */

  uint32_t DMA_PeripheralInc;      /*!< Specifies whether the Peripheral address register is incremented or not.
                                        This parameter can be a value of @ref DMA_incremented_mode */

  uint32_t DMA_MemoryInc;          /*!< Specifies whether the memory address register is incremented or not.
                                        This parameter can be a value of @ref DMA_incremented_mode */

  uint32_t DMA_PeripheralDataSize; /*!< Specifies the Peripheral data item width.
                                        This parameter can be a value of @ref DMA_data_size */

  uint32_t DMA_MemoryDataSize;     /*!< Specifies the Memory data item width.
								   This parameter can be a value of @ref DMA_data_size */

  uint32_t DMA_PeripheralBurstSize; /*!< Specifies the Peripheral Number of data items during per burst transaction.
									read or write from the Peripheral every time a burst transaction request 
									This parameter can be a value of @ref DMA_burst_size */

  uint32_t DMA_MemoryBurstSize;     /*!< Specifies the Memory Number of data items during per burst transaction.
									read or write from the Memory every time a burst transaction request 
									This parameter can be a value of @ref DMA_burst_size */

  uint32_t DMA_PeripheralHandShake;	/*!< Specifies the HandShake to control the DMA transacation.
									This parameter can be a value of @ref DMA_peripheral_handshake */

  uint32_t DMA_BlockSize;			/*!< Specifies the Total Number of data items during the transaction. */

  uint32_t DMA_Priority;           /*!< Specifies the software priority for the DMAy Channelx.
										This parameter can be a value of @ref DMA_priority_level */
}DMA_InitTypeDef;

/** @defgroup DMA_data_transfer_direction 
  * @{
  */
#define DMA_DIR_Memory_To_Memory								((uint32_t)0x0000)
#define DMA_DIR_Peripheral_To_Memory							((uint32_t)0x0001)
#define DMA_DIR_Memory_To_Peripheral							((uint32_t)0x0002)
/**
  * @}
  */

/** @defgroup DMA_incremented_mode 
  * @{
  */

#define DMA_Inc_Increment						((uint32_t)0x00000000)
#define DMA_Inc_Decrement						((uint32_t)0x00000001)
#define DMA_Inc_Nochange						((uint32_t)0x00000002)
#define IS_DMA_INC_STATE(STATE) (((STATE) == DMA_Inc_Increment) || \
                                 ((STATE) == DMA_Inc_Decrement) || \
                                 ((STATE) == DMA_Inc_Nochange))
/**
  * @}
  */

/** @defgroup DMA_data_size 
  * @{
  */

#define DMA_DataSize_Byte					    ((uint32_t)0x0000)
#define DMA_DataSize_HalfWord				    ((uint32_t)0x0001)
#define DMA_DataSize_Word						((uint32_t)0x0002)
#define IS_DMA_DATA_SIZE(SIZE)					(((SIZE) == DMA_DataSize_Byte) || \
												 ((SIZE) == DMA_DataSize_HalfWord) || \
												 ((SIZE) == DMA_DataSize_Word))
/**
  * @}
  */

/** @defgroup DMA_burst_size
  * @{
  */
#define DMA_BurstSize_1							((uint32_t)0x00)
#define DMA_BurstSize_4							((uint32_t)0x01)
#define DMA_BurstSize_8							((uint32_t)0x02)
/**
  * @}
  */

/** @defgroup DMA_peripheral_handshake
  * @{
  */
#define DMA_PeripheralHandShake_Hardware				((uint32_t)0x0000)
#define DMA_PeripheralHandShake_Software				((uint32_t)0x0001)
/**
  * @}
  */

/** @defgroup DMA_Priority
  * @{
  */
#define DMA_Priority_0									((uint32_t)0x00000000)
#define DMA_Priority_1									((uint32_t)0x00000020)
#define DMA_Priority_2									((uint32_t)0x00000040)
#define DMA_Priority_3									((uint32_t)0x00000060)
/**
  * @}
  */

/** @defgroup DMA_IT
  * @{
  */
#define DMA_IT_BlockTransferComplete					((uint32_t)0x01)
#define DMA_IT_DestinationTransactionComplete			((uint32_t)0x02)
#define DMA_IT_Error									((uint32_t)0x04)
#define DMA_IT_SourceTransactionComplete				((uint32_t)0x08)
#define DMA_IT_DMATransferComplete						((uint32_t)0x10)
/**
  * @}
  */

void DMA_Init(DMA_TypeDef* DMA_Channelx, DMA_InitTypeDef* DMA_InitStruct);
void DMA_ChannelCmd(DMA_TypeDef* DMA_Channelx, FunctionalState NewState);
void DMA_Cmd(FunctionalState NewState);

void DMA_SetSRCAddress(DMA_TypeDef* DMA_Channelx, uint32_t Address);
void DMA_SetDSRAddress(DMA_TypeDef* DMA_Channelx, uint32_t Address);
void DMA_SetChannelHardShakePeriph(DMA_TypeDef* DMA_Channelx,uint32_t HardwareHandshake_Peripheral);
	 
void DMA_ITConfig(DMA_TypeDef* DMA_Channelx, uint32_t DMA_IT, FunctionalState NewState);
	 
FlagStatus DMA_GetFlagStatus(uint32_t DMA_FLAG);	 
void DMA_ClearFlag(uint32_t DMA_FLAG);

FunctionalState DMA_IsChannelEnabled(DMA_TypeDef* DMA_Channelx);
	 
ITStatus DMA_GetITStatus(DMA_TypeDef* DMA_Channelx,uint32_t DMA_IT);
FlagStatus DMA_GetRawStatus(DMA_TypeDef* DMA_Channelx,uint32_t DMA_IT);
void DMA_ClearITPendingBit(DMA_TypeDef* DMA_Channelx,uint32_t DMA_IT);

#ifdef __cplusplus
}
#endif

#endif 
