#include <stdio.h>
#include "mhscpu.h"
#include "test_emv.h"
#include "uart.h"





const static IRQn_Type aTIM_IRQn[4] = {TIM0_0_IRQn, TIM0_1_IRQn, TIM0_2_IRQn, TIM0_3_IRQn};


void print_hex(char *s, uint8_t  *buf, int len)
{
    DBG_PRINT("%s\n", s);
    while (len--)
    {
        DBG_PRINT("%02X ", (uint16_t)*buf++);
    }
    DBG_PRINT("\n");
    //DBG_PRINT("addr = %04X\n", (uint16_t)buf);
}



void timer0_1_stop(uint8_t n)
{
    TIMM0->TIM[0].EOI |= 1;
    NVIC_DisableIRQ(aTIM_IRQn[n]);
    TIMM0->TIM[n].ControlReg = 0x06;
}

void Timer0_0_IRQHandler(void)
{
    if (0 == TIMM0->TIM_ReloadCount[0])
    {
        NVIC_DisableIRQ(TIM0_0_IRQn);
    }
    TIMM0->TIM[0].EOI |= 1;
}


uint32_t timer0_1_IsTimeout(uint8_t n)
{
    return (0 != TIMM0->TIM[n].IntStatus);
}


void timer0_1_start(uint8_t n, uint32_t init_count, uint32_t reload_count)
{
    NVIC_DisableIRQ(aTIM_IRQn[n]);
    //|Disable | User define mode | Masked | Disable PWM |
    TIMM0->TIM[n].ControlReg = 0x06;
    TIMM0->TIM[n].LoadCount = init_count;
    TIMM0->TIM_ReloadCount[n] = reload_count;
    //Read clear.Avoid warning
    TIMM0->TIM[n].EOI |= 1;
    NVIC_ClearPendingIRQ(aTIM_IRQn[n]);
    TIMM0->TIM[n].ControlReg = 0x03;
    //irq_remove(aTIM_IRQn[n]);
    //irq_register(aTIM_IRQn[n], timer_Int[n]);
    //NVIC_EnableIRQ(aTIM_IRQn[n]);
}

void delay_Ms(uint32_t u32Us)
{
    long long u64OneMs;

    u64OneMs = SYSCTRL->PCLK_1MS_VAL;
    u64OneMs = u64OneMs * u32Us;
    if (u64OneMs > ~0UL)
    {
        while (1);
    }
    timer0_1_start(0, (uint32_t)u64OneMs, 0);
    while (0 == timer0_1_IsTimeout(0));
}


void wait_Ns(uint8_t u8N)
{
    uint32_t i;
    
    DBG_PRINT("Wait %d second:", u8N);
    for (i = 0; i < u8N; i++)
    {
        DBG_PRINT("#");
        delay_Ms(1000);
    }
    DBG_PRINT("$\n");
}

uint32_t wait_ChangeNs(uint8_t u8Wait)
{
    uint32_t i, u32Loop = u8Wait;
    int32_t s32Recv = -1;
    
    DBG_PRINT("Wait %d second:", u8Wait);
    for (i = 0; i < u32Loop; i++)
    {
        //Timer 1s
        timer0_1_start(0, SYSCTRL->PCLK_1MS_VAL * 1000, 0);
        DBG_PRINT("#");
        while (0 == timer0_1_IsTimeout(0))
        {
            s32Recv = uart_RecvChar();
            if (s32Recv > -1)
            {
                DBG_PRINT("%d", s32Recv);
                if ('L' == s32Recv)
                {
                    u32Loop = 30;
                }
                else if ('S' == s32Recv)
                {
                    u32Loop = 5;
                }
                else if('E' == s32Recv)
                {
                    i = u32Loop;
                    break;
                }
                else
                {
                    DBG_PRINT("Unknown time!It can be S:Short wait(5S) or L:Long wait(30S) E:Exit this wait!");
                }
            }
        }
    }
    DBG_PRINT("$\n");
    return u32Loop;
}
