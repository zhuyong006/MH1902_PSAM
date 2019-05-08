

#ifndef __MHSCPU_BPK_H
#define __MHSCPU_BPK_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "mhscpu.h"

     
#define BPK_KEY_Region_0                      (0x0001)
#define BPK_KEY_Region_1                      (0x0002)
#define BPK_KEY_Region_2                      (0x0004)
#define BPK_KEY_Region_3                      (0x0008)  
#define IS_BPK_KEY_REGION(KEY)				((((KEY) & BPK_KEY_Region_0) != 0x00) ||  \
											(((KEY) & BPK_KEY_Region_1) != 0x00) ||  \
											(((KEY) & BPK_KEY_Region_2) != 0x00) ||  \
											(((KEY) & BPK_KEY_Region_3) != 0x00))     

#define BPK_LOCK_LOCK						(0x0001)
#define BPK_LOCK_Reset						(0x0002) 
#define BPK_LOCK_KeyWriteLock				(0x0004) 
#define BPK_LOCK_KeyReadLock				(0x0008) 
#define BPK_LOCK_KeyClear					(0x0010)    
#define BPK_LOCK_SetScramber				(0x0020)      
#define IS_BPK_LOCK(LOCK)					((((LOCK) & BPK_LOCK_Reset) != 0x00) ||  \
											(((LOCK) & BPK_LOCK_KeyWriteLock) != 0x00) ||  \
											(((LOCK) & BPK_LOCK_KeyReadLock) != 0x00) ||  \
											(((LOCK) & BPK_LOCK_KeyClear) != 0x00) ||  \
											(((LOCK) & BPK_LOCK_SetScramber) != 0x00))    
/** @defgroup BPK_Exported_Types
  * @{
  */
     
FlagStatus BPK_IsReady(void);
ErrorStatus BPK_WriteKey(uint32_t * Key,uint32_t Key_Len,uint32_t Key_Offset);
ErrorStatus BPK_ReadKey(uint32_t * Key,uint32_t Key_Len,uint32_t Key_Offset);
void BPK_KeyWriteLock(uint16_t BPK_KEY_Region, FunctionalState NewState);
void BPK_KeyReadLock(uint16_t BPK_KEY_Region, FunctionalState NewState);
void BPK_KeyClear(uint16_t BPK_KEY_Region);
void BPK_SetScramber(uint32_t Scram);
void BPK_Reset(void);
void BPK_Lock(uint32_t BPK_LOCK, FunctionalState NewState);
FlagStatus BPK_GetLockStatus(uint32_t BPK_LOCK);
void BPK_LockLock(void);
FlagStatus BPK_GetLockLockStatus(void);

#ifdef __cplusplus
}
#endif

#endif 
