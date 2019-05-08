
#include <string.h>
#include <stdio.h>
#include "mhscpu.h"
#include "mhscpu_sci.h"
#include "iso7816_3.h"
#include "emv_errno.h"
#include "emv_hard.h"
#include "uart.h"
#include "define.h"
#include "mhscpu_gpio.h"

#define SCI_DBG 1


#define ALIGN_4(pAddr) ((uint8_t *)((int32_t)((uint8_t *)pAddr + 3) & ~0x03UL))

#ifdef SCI_DBG
#define DBG_PRINT(format, args...)\
        printf(format, ##args)
#else
#define BOOT_DBG(fmt, args...)
#endif

void NVIC_Configuration(void);
void SCI_Init(void);
uint8_t atr_buf[64];

extern struct emv_core emv_devs[];
#define MH_SMCU_SCU_DES_MASK_BASE       (0x40003600)
//static uint32_t                 *   pRom_Boot_Info      = (uint32_t *)MH_SMCU_SCU_DES_MASK_BASE;



extern uint32_t __Vectors;
extern RNG_BUF_ID  uart_SendBuf;
extern RNG_BUF_ID  uart_RecvBuf;
extern RNG_BUF_ID  spi_SendBuf;
extern RNG_BUF_ID  spi_RecvBuf;


//uint8_t au8RecvBuf[UART_RECV_BUF_SIZE + sizeof(RNG_BUF) + 32];
//uint8_t au8SendBuf[UART_SEND_BUF_SIZE + sizeof(RNG_BUF) + 32];

//#define PIN_ALT_MASK(pin)     (0x03UL << ((pin) << 1))
//#define PIN_ALT(pin, alt)     (((alt) & 0x03UL) << ((pin) << 1))


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

uint8_t cal_lrc(uint8_t *p_buf, uint16_t len)
{
    uint8_t lrc = 0;
    int i = 0;

    for (i = 0; i < len; i++)
    {
        lrc ^= p_buf[i];
    }
    return lrc;
}

void timer0_1_stop(uint8_t n)
{
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

void Timer0_1_IRQHandler(void)
{
    if (0 == TIMM0->TIM_ReloadCount[1])
    {
        NVIC_DisableIRQ(TIM0_1_IRQn);
    }
    TIMM0->TIM[1].EOI |= 1;
}

/*
uint32_t timer0_1_IsTimeout(uint8_t n)
{
    return (0 != (TIMM0->TIM_RawIntStatus & BIT(n)));
    //return (0 != TIMER0->timer[0 != n].IntStatus);
}
*/

uint32_t timer0_1_IsTimeout(uint8_t n)
{
    return (0 != TIMM0->TIM[n].IntStatus);
}

/*
void timer0_1_start(uint8_t n, uint32_t init_count, uint32_t reload_count)
{
    //SYSCTRL->SOFT_RST |= 0 != n ? BIT(18) : BIT(17);
    NVIC->ICER[0] = 0 != n ? BIT(18) : BIT(17);
    //|Disable | User define mode | Masked | Disable PWM |
    TIMM0->timer[0 != n].ControlReg = 0x06;
    TIMER0->timer[0 != n].LoadCount = init_count;
    TIMER0->ReloadCount[0 != n] = reload_count;
    //Read clear.Avoid warning
    TIMER0->timer[0 != n].EOI |= 1;
    NVIC->ICPR[0] = 0 != n ? BIT(18) : BIT(17);
    TIMER0->timer[0 != n].ControlReg = 0x03;
    NVIC->ISER[0] = 0 != n ? BIT(18) : BIT(17);
}
*/

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

static void delay_Ms(uint32_t u32Us)
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


#define EMV_VERSION             (0x43A)

#if 0x410 == EMV_VERSION
void loop_back(uint8_t u8Slot)
{
    uint8_t i, lrc = 0;
    int32_t s32Stat = 0;
    uint8_t  atr[65];
    ST_APDU_RSP     rsp;
    ST_APDU_REQ     apdu_req;

#if 1
    ST_APDU_REQ  sel_visa_req = {{0x00, 0xA4, 0x04, 0x00},  //cmd
                                  0x00000007,               //lc
                                {0xA0, 0x00, 0x00, 0x00, 0x03, 0x10, 0x10, 0x00},
                                  0x00000100                //le
                                };
#endif

    while (1)
    {
        //Init emv_devs param.
        iso7816_device_init();
        DBG_PRINT("Insert IC\n");
        s32Stat = iso7816_init(u8Slot, VCC_5000mV | SPD_1X, atr);
        if (0 == s32Stat)
        {
            DBG_PRINT("iso7816_init() finished.\n");
        }
        else
        {
            DBG_PRINT("iso7816_init() failed %d.\n", s32Stat);
        }

        lrc = 0;
        rsp.len_out = 0;
        while (0 == iso7816_detect(u8Slot) && 0 == s32Stat)
        {
            if (rsp.len_out < 8)
            {
                memcpy(&apdu_req, &sel_visa_req, 16);//lc is four bytes len
                apdu_req.cmd[3] = lrc;
                apdu_req.le = 256;

            }
            else
            {
                memcpy(apdu_req.cmd, rsp.data_out, 3);
                apdu_req.cmd[3] = lrc;
                apdu_req.lc = 0;
                apdu_req.le = 0;

                if (rsp.data_out[3] > 2 && rsp.data_out[4] > 0)
                {
                    apdu_req.lc = rsp.data_out[4];
                    for (i = 0; i < apdu_req.lc; i++)
                    {
                        apdu_req.data_in[i] = i;
                    }
                }
                if (4 == rsp.data_out[3] || 2 == rsp.data_out[3])
                {
                    if (0 == rsp.data_out[5])
                    {
                        apdu_req.le = 256;
                    }
                    else
                    {
                        apdu_req.le = rsp.data_out[5];
                    }
                }
            }
            s32Stat = iso7816_exchange(u8Slot, 1, &apdu_req, &rsp);
            if (0x6A == rsp.swa && 0x82 == rsp.swb)
            {
                break;
            }
            lrc = cal_lrc(rsp.data_out, rsp.len_out);

        }
        iso7816_close(u8Slot);
        DBG_PRINT("exchange finished!\n");
        while('c' != s32Stat)
        {
            DBG_PRINT("Press 'c' to continue\n");
            while (-1UL == (s32Stat = uart_RecvChar()));
            DBG_PRINT("%c\n", s32Stat);
        }
    }
}

void electr_test(uint8_t u8Slot)
{
    int32_t s32Recv;
    uint32_t u32Stat;
    uint8_t single_mode = 0, atr[65];
    ST_APDU_RSP         rsp;
    ST_APDU_REQ        apdu_req;

    while (1)
    {
        if (single_mode)
        {
            DBG_PRINT("Single mode.\nPress 'T' to time-cycle mode.\n");
            while(-1UL == (s32Recv = uart_RecvChar()));
            DBG_PRINT("%c\n", s32Recv);
            //'C' means time-cycle mode.
            if ('T' == s32Recv)
            {
                single_mode = 0;
            }
        }
        else
        {
            DBG_PRINT("Time-cycle mode.\nPress 'S' to single mode.\n");
            s32Recv = uart_RecvChar();
            if ('S' == s32Recv)
            {
                single_mode = 1;
            }
        }

        iso7816_device_init();
        DBG_PRINT("Insert IC\n");
        if (0 != iso7816_init(u8Slot, VCC_5000mV | SPD_1X, atr))
        {
            DBG_PRINT("iso7816_init() finished\n");
        }

        memcpy(apdu_req.cmd, "\x00\xa4\x04\x00", 4 );
        apdu_req.le = 256;
        apdu_req.lc = 14;
        memcpy( apdu_req.data_in, "1PAY.SYS.DDF01", 14 );

        while (0 != apdu_req.lc && 0xA4 == apdu_req.cmd[1])
        {
            u32Stat = (uint32_t)iso7816_exchange(u8Slot, 1, &apdu_req, &rsp);
            memcpy(apdu_req.cmd, rsp.data_out, sizeof(apdu_req.cmd));
            apdu_req.lc = rsp.data_out[sizeof(apdu_req.cmd)];
            memcpy(apdu_req.data_in, rsp.data_out + 1 + sizeof(apdu_req.cmd), apdu_req.lc);
            apdu_req.le = 256;
        }
        iso7816_close(u8Slot);
        if (!single_mode)
        {
            delay_Ms(5000);
        }
    }

}

int main(void)
{
    uint32_t u32Ms;
    int32_t s32Recv;
    uint8_t *pMemMap = (uint8_t *)((__Vectors + 4) & ~0x03UL);

    //pMemMap = (uint8_t *)(0x20030000);
    uart_SendBuf = (RNG_BUF_ID)pMemMap;
    pMemMap += UART_SEND_BUF_SIZE + sizeof(RNG_BUF);
    pMemMap = ALIGN_4(pMemMap);
    
    //uart_SendBuf = (RNG_BUF_ID)au8SendBuf;

    uart_RecvBuf = (RNG_BUF_ID)pMemMap;
    pMemMap += UART_RECV_BUF_SIZE + sizeof(RNG_BUF) + 64;
    pMemMap = ALIGN_4(pMemMap);
    
    //uart_RecvBuf = (RNG_BUF_ID)au8RecvBuf;
    
    //Enable clock SCI0, SCI1, SCI2, UART0, and GPIO.
    SYSCTRL_APBPeriphClockCmd(SYSCTRL_APBPeriph_SCI0 | SYSCTRL_APBPeriph_UART0 | SYSCTRL_APBPeriph_GPIO, ENABLE);
    SYSCTRL_APBPeriphResetCmd(SYSCTRL_APBPeriph_SCI0 | SYSCTRL_APBPeriph_UART0, ENABLE);

    //card detect
    SYSCTRL->PHER_CTRL |= BIT(16);
    //Choose active level(Low level active).
    SYSCTRL->PHER_CTRL |= BIT(20);

    u32Ms = SYSCTRL->PCLK_1MS_VAL * 5;
    uart_Config((u32Ms + (1152 << 2))/ (1152 << 3));

    GPIO_PinRemapConfig(GPIOA, GPIO_Pin_4  | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_14, GPIO_Remap_0);
    SYSCTRL->SCI_GLF = SYSCTRL->SCI_GLF & ~BIT(29) | BIT(28);
    SCI_ConfigEMV(0x01, 1000000);

    NVIC_Configuration();
    
    s32Recv = -1;
    while(1)
    {
        while('p' != s32Recv && 'e' != s32Recv)
        {
            DBG_PRINT("Press 'p' to protocol testing, 'e' to electrical testting\n");
            while (-1UL == (s32Recv = uart_RecvChar()));
            DBG_PRINT("%c\n", s32Recv);
        }
        if ('p' == s32Recv)
        {
            loop_back(0);
        }
        else if ('e' == s32Recv)
        {
            electr_test(0);
        }
    }

    return 0;
}

#elif 0x43A == EMV_VERSION
void loop_back(uint8_t u8Slot)
{
    uint8_t i, lrc = 0;
    int32_t s32Stat = 0;
    uint8_t  atr[65];
    ST_APDU_RSP     rsp;
    ST_APDU_REQ     apdu_req;
    
    while (1)
    {
        //Init emv_devs param.
        iso7816_device_init();
		
		//ÂÇÃ«´ÌÊ¹ÄÜ
		SYSCTRL->SCI_GLF &= ~BIT(31);
        
        DBG_PRINT("Waiting for insert IC card\n");
        while (0 != iso7816_detect(u8Slot));
        delay_Ms(1000);
        if (0 == (s32Stat = iso7816_init(u8Slot, VCC_3000mV | SPD_1X, atr)))
        {
            DBG_PRINT("iso7816_init finished!\n");
            rsp.len_out = 0;
            while (0 == iso7816_detect(u8Slot))
            {
                if (rsp.len_out < 6)
                {
                    DBG_PRINT("Prepare to send select PSE!\n");
                    memcpy(apdu_req.cmd, "\x00\xa4\x04\x00", 4);
                    apdu_req.lc = 14;
                    memcpy( apdu_req.data_in, "1PAY.SYS.DDF01", apdu_req.lc);
                    apdu_req.le = 256;
                }
                else
                {
                    //INS equal 70 then deactivate card.
                    if (0x70 == rsp.data_out[1])
                    {
                        DBG_PRINT("This test Case finished!\n");
                        break;
                    }
                    DBG_PRINT("Prepare to send loopback data!\n");
                    //Copy next C-APDU from this R-APDU.
                    memcpy(apdu_req.cmd, rsp.data_out, sizeof(apdu_req.cmd));
                    apdu_req.lc = rsp.data_out[sizeof(apdu_req.cmd)];
                    memcpy(apdu_req.data_in, rsp.data_out + 1 + sizeof(apdu_req.cmd), apdu_req.lc);
                    apdu_req.le = 256;
                }
                delay_Ms(200);
                if (0 != iso7816_exchange(u8Slot, AUTO_GET_RSP, &apdu_req, &rsp))
                {
                    rsp.len_out = 0;
                }
            }
        }
        else
        {
            DBG_PRINT("iso7816_init failed %d!\n", s32Stat);
        }
        DBG_PRINT("Shut down ICC!\n");
        //Deinit struct power down card and.
        iso7816_close(u8Slot);
        //Waiting for card drawn.
        while (0 == iso7816_detect(u8Slot));
        DBG_PRINT("Remove ICC!\n");
    }

}

int main(int argc, char *argv[])
{
    uint32_t u32Ms;
    uint8_t *pMemMap = (uint8_t *)((__Vectors + 4) & ~0x03UL);

    //pMemMap = (uint8_t *)(0x20030000);
    uart_SendBuf = (RNG_BUF_ID)pMemMap;
    pMemMap += UART_SEND_BUF_SIZE + sizeof(RNG_BUF);
    pMemMap = ALIGN_4(pMemMap);
    
    //uart_SendBuf = (RNG_BUF_ID)au8SendBuf;

    uart_RecvBuf = (RNG_BUF_ID)pMemMap;
    pMemMap += UART_RECV_BUF_SIZE + sizeof(RNG_BUF) + 64;
    pMemMap = ALIGN_4(pMemMap);

    
    //Enable clock SCI0, SCI1, SCI2, UART0, and GPIO.
    SYSCTRL_APBPeriphClockCmd(SYSCTRL_APBPeriph_SCI0 | SYSCTRL_APBPeriph_UART0 | SYSCTRL_APBPeriph_GPIO | SYSCTRL_APBPeriph_TIMM0, ENABLE);
    SYSCTRL_APBPeriphResetCmd(SYSCTRL_APBPeriph_SCI0 | SYSCTRL_APBPeriph_UART0 | SYSCTRL_APBPeriph_TIMM0, ENABLE);

#define USE_EX_12M          0

#if 1 == USE_EX_12M
    //Clear Bit9 use external clock.
    SYSCTRL->FREQ_SEL &= ~BIT(9);
#else
    //Set Bit9 use internal clock.
    SYSCTRL->FREQ_SEL |= BIT(9);
#endif
    
    //card detect 0:High active 1: Low active
    SYSCTRL->PHER_CTRL |= BIT(16);
    //VCC_EN 0: Hign enable 1:Low Enable.
    SYSCTRL->PHER_CTRL |= BIT(20);

    u32Ms = SYSCTRL->PCLK_1MS_VAL * 5;
    uart_Config((u32Ms + (1152 << 2))/ (1152 << 3));

    DBG_PRINT("MH1902 SCI Demo!\n");
    
    GPIO_PinRemapConfig(GPIOA, GPIO_Pin_4  | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_14, GPIO_Remap_0);
    SYSCTRL->SCI_GLF = SYSCTRL->SCI_GLF & ~BIT(29) | BIT(28);
    
    SCI_ConfigEMV(0x01, 3000000);

    NVIC_Configuration();

    loop_back(0);
    return 0;
}



#endif







void NVIC_Configuration(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    //NVIC_SetVectorTable(NVIC_VectTab_RAM,0);
    NVIC_SetPriorityGrouping(NVIC_PriorityGroup_0);
    
    NVIC_InitStructure.NVIC_IRQChannel = SCI0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    /*
    NVIC_InitStructure.NVIC_IRQChannel = SCI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    NVIC_InitStructure.NVIC_IRQChannel = SCI2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    */
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
    /* User can add his own implementation to report the file name and line number,
     ex: PRINT("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small DBG_PRINT (option LD Linker->Libraries->Small DBG_PRINT
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
  #define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
  #define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART */
    
    if (ch == '\n')
    {
        while(0 != uart_SendChar('\r'));
    }
    while(0 != uart_SendChar((uint8_t) ch));

    return ch;
}

GETCHAR_PROTOTYPE
{
    int32_t s32Recv;

    while (-1 == (s32Recv = uart_RecvChar()));

    return s32Recv;
}





