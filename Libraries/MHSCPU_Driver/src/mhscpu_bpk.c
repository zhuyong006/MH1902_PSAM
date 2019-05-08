#include "mhscpu_bpk.h"

#define BPK_KEY_WORD_SIZE								(32)


FlagStatus BPK_IsReady(void)
{
	if (BPK->BPK_RDY & 0x01)
	{
		return SET;
	}
	else
	{
		return RESET;
	}
}

ErrorStatus BPK_WriteKey(uint32_t *Key, uint32_t Key_Len, uint32_t Key_Offset)
{
	uint32_t index = 0;
	
	if (BPK_IsReady() == RESET)
	{
		return ERROR;
	}
	
	if (Key_Offset + Key_Len > BPK_KEY_WORD_SIZE)
	{
		return ERROR;
	}

	for (index = 0; index < Key_Len; index++)
	{
		BPK->KEY[Key_Offset + index] = Key[index];
	}
    
	return SUCCESS;
}


ErrorStatus BPK_ReadKey(uint32_t *Buf, uint32_t Key_Len, uint32_t Key_Offset)
{
	uint32_t index = 0;
	
	if (BPK_IsReady() == RESET)
	{
		return ERROR;
	}

	if (Key_Offset + Key_Len > BPK_KEY_WORD_SIZE)
	{
		return ERROR;
	}

	for (index = 0; index < Key_Len; index++)
	{
		Buf[index] = BPK->KEY[Key_Offset + index];
	}
	
	return SUCCESS;
}

void BPK_KeyClear(uint16_t BPK_KEY_Region)
{
    assert_param(IS_BPK_KEY_REGION(BPK_KEY_Region));
	
    BPK->BPK_CLR |= BPK_KEY_Region;
	while (BPK->BPK_CLR);
}

void BPK_KeyWriteLock(uint16_t BPK_KEY_Region, FunctionalState NewState)
{
    assert_param(IS_BPK_KEY_REGION(BPK_KEY_Region));
	
    if (DISABLE != NewState)
    {
        BPK->BPK_LWA |= BPK_KEY_Region;
    }
	else
	{
        BPK->BPK_LWA &= ~BPK_KEY_Region;
    }
}

void BPK_KeyReadLock(uint16_t BPK_KEY_Region, FunctionalState NewState)
{
    assert_param(IS_BPK_KEY_REGION(BPK_KEY_Region));
    
	if (DISABLE != NewState)
    {
        BPK->BPK_LRA |= BPK_KEY_Region;
    }
	else
	{
        BPK->BPK_LRA &= ~BPK_KEY_Region;
    }
}

void BPK_Reset(void)
{
//    BPK->BPK_RR |= 1;
//    while (BPK->BPK_RR);
	
	BPK->BPK_CLR |= BPK_KEY_Region_0 + BPK_KEY_Region_1 + BPK_KEY_Region_2 + BPK_KEY_Region_3;
	while (BPK->BPK_CLR);	
} 

void BPK_SetScramber(uint32_t Scram)
{
	BPK->BPK_SCR = Scram;
}

void BPK_Lock(uint32_t BPK_LOCK, FunctionalState NewState)
{
	assert_param(IS_BPK_LOCK(BPK_LOCK));
	
	BPK_LOCK &= ~BPK_LOCK_LOCK;
	if (DISABLE != NewState)
	{
		BPK->BPK_LR |= BPK_LOCK;
	}
	else
	{
		BPK->BPK_LR &= ~BPK_LOCK;
	}
}

FlagStatus BPK_GetLockStatus(uint32_t BPK_Locks)
{
	assert_param(IS_BPK_LOCK(BPK_Locks));
	
	BPK_Locks &= ~BPK_LOCK_LOCK;
	if ((BPK->BPK_LR & BPK_Locks) != (uint16_t)RESET)
	{
		return SET;
	}
	else
	{
		return RESET;
	}
}

void BPK_LockLock(void)
{
	BPK->BPK_LR |= BPK_LOCK_LOCK;
}

FlagStatus BPK_GetLockLockStatus(void)
{
	if ((BPK->BPK_LR & BPK_LOCK_LOCK) != (uint16_t)RESET)
	{
		return SET;
	}
	else
	{
		return RESET;
	}
}

void BPK_Calibrate32K(void)
{
    #define BPK_RSVD                    (*((uint32_t *)0x4003015C))    
    #define IN32K_CALIBRATED_FLAG       (0xA)
	
	uint32_t u32TimeDelay = 0xffff;
    
    if (IN32K_CALIBRATED_FLAG != BPK_RSVD)
    {   
		RTC->OSC32K_CR &= ~BIT(6);
		while (--u32TimeDelay);
	
        RTC->OSC32K_CR |= BIT(6);
        while (RTC->OSC32K_CR & BIT(6));

        BPK_RSVD = IN32K_CALIBRATED_FLAG;
    }
}
