
#include <string.h>
#include <stdio.h>
#include "mhscpu.h"
#include "mhscpu_sci.h"
#include "iso7816_3.h"
#include "emv_errno.h"
#include "emv_hard.h"
#include "uart.h"
#include "define.h"

#define SCI_DBG


#define ALIGN_4(pAddr) ((uint8_t *)((int32_t)((uint8_t *)pAddr + 3) & ~0x03UL))

#ifdef SCI_DBG
#define BOOT_DBG(format, args...)\
        printf(format, ##args)
#else
#define BOOT_DBG(fmt, args...)
#endif

void NVIC_Configuration(void);
void SCI_Init(void);
uint8_t atr_buf[64];

static ST_APDU_REQ apdu_req;
static ST_APDU_RSP apdu_rsp;

extern struct emv_core emv_devs[];
#define MH_SMCU_SCU_DES_MASK_BASE       (0x40003600)
//static uint32_t                 *   pRom_Boot_Info      = (uint32_t *)MH_SMCU_SCU_DES_MASK_BASE;

/*
uint32_t get_HCLK_1Ms(void)
{
	uint8_t u8Tmp;
	uint32_t u32OneMs;

	u8Tmp = (SYSCTRL->FREQ_SEL >> 4) & 0x03;
    if (0 == u8Tmp)
    {
        u32OneMs = 72000;
    }
    else if (1 == u8Tmp)
    {
        u32OneMs = 96000;
    }
    else if (2 == u8Tmp)
    {
        u32OneMs = 108000;
    }
    else
    {
        u32OneMs = 120000;
    }
    if (0 != (SYSCTRL->FREQ_SEL & BIT(3)))
    {
        u32OneMs >>= 1;
        if (0 != (SYSCTRL->FREQ_SEL & BIT(2)))
        {
            u32OneMs >>= 1;
        }
    }

	return u32OneMs;
}

uint32_t get_PCLK_1Ms(void)
{
	uint32_t u32OneMs;

    u32OneMs = get_HCLK_1Ms();
    
    if (0 != (SYSCTRL->FREQ_SEL & BIT(1)))
    {
        u32OneMs >>= 1;
        if (0 != (SYSCTRL->FREQ_SEL & BIT(0)))
        {
            u32OneMs >>= 1;
        }
    }

	return u32OneMs;
}
*/
static int32_t tst_SCIWarmReset(uint32_t u32Slot)
{
    //uint8_t au8Out_ATR[64];
    int32_t s32Slot, state;
    struct emv_core     *pdev;
    struct emv_atr       su_atr;

    memset(atr_buf, 0, sizeof(atr_buf));
    memset(&apdu_req, 0, sizeof(apdu_req));
    memset(&apdu_rsp, 0, sizeof(apdu_rsp));


	BOOT_DBG("Test WarmReset!\n");

    s32Slot = u32Slot;
    s32Slot = select_slot( s32Slot );
    pdev = &( emv_devs[ s32Slot ] );
    pdev->terminal_ch = s32Slot;

    //SCI_ConfigEMV(0x07, 3000000UL);
    state = iso7816_device_init();
    if (state)
    {
        while (1);
    }
	
    /**
     * detect user card whether is in slot or not?
     */
	
    pdev->terminal_vcc = 5000;
	
    pdev->terminal_pps = 0;

    pdev->terminal_fi    = 372;
    pdev->terminal_implict_fi = 372;
	
    pdev->terminal_di = 1;
    pdev->terminal_implict_di = 1;

    pdev->terminal_spec = 0;
	
    pdev->terminal_ifs  = 254;
    pdev->terminal_pcb  = 0x00;
    pdev->terminal_ipcb = 0x00;
    pdev->emv_card_pcb  = 0x00;
    pdev->emv_card_ipcb = 0x00;
    pdev->terminal_igt  = 16;
    pdev->terminal_mode = 0; /* asynchronize card */


    while (0 != (state = emv_hard_power_pump( pdev )));

    emv_hard_cold_reset( pdev );
	
    if( 0 == ( state = emv_atr_analyser( pdev, &su_atr, atr_buf )  ) )
    {
        state = emv_atr_parse( pdev, &su_atr ); 
        state = 1;
        if( 0 != state )
        {
            memset(atr_buf, 0, sizeof(atr_buf));
            emv_hard_warm_reset( pdev );/*warm reset.*/
            
            if( 0 == ( state = emv_atr_analyser( pdev, &su_atr, atr_buf ) ) )
            {
                state = emv_atr_parse( pdev, &su_atr );
            }
        }
    }
    else
    {
        memset(atr_buf, 0, sizeof(atr_buf));
        emv_hard_warm_reset( pdev );/*warm reset.*/

        if( 0 == ( state = emv_atr_analyser( pdev, &su_atr, atr_buf ) ) )
        {
            state = emv_atr_parse( pdev, &su_atr );
        }
    }
	
    while (state);

    BOOT_DBG("recv ATR!\n");
    memcpy(apdu_req.cmd, "\x00\xa4\x04\x00", 4 );
    apdu_req.le = 256;
    apdu_req.lc = 14;
    memcpy( apdu_req.data_in, "1PAY.SYS.DDF01", 14 );

    state = iso7816_exchange(s32Slot, 1, &apdu_req, &apdu_rsp);

    if (state)
    {
        BOOT_DBG("recv response1 %d\n", state);
        while (1);
    }
	BOOT_DBG("WarmReset test finished, reset!\n");
	while (0 == uart_IsSendFinish());
	SYSCTRL->LOCK_R = 0;
	SYSCTRL->SOFT_RST2 = BIT(31);
    while (1);

    return 0;
}

extern uint32_t __Vectors;
extern RNG_BUF_ID  uart_SendBuf;
extern RNG_BUF_ID  uart_RecvBuf;
extern RNG_BUF_ID  spi_SendBuf;
extern RNG_BUF_ID  spi_RecvBuf;


//uint8_t au8RecvBuf[UART_RECV_BUF_SIZE + sizeof(RNG_BUF) + 32];
//uint8_t au8SendBuf[UART_SEND_BUF_SIZE + sizeof(RNG_BUF) + 32];

#define PIN_ALT_MASK(pin)		(0x03UL << ((pin) << 1))
#define PIN_ALT(pin, alt)		(((alt) & 0x03UL) << ((pin) << 1))

int main(int argc, char *argv[])
{
    int32_t state, s32Recv;
	uint32_t slot = 0, u32Ms, u32Recv;
	uint8_t *pMemMap = (uint8_t *)((__Vectors + 4) & ~0x03UL);

    uart_SendBuf = (RNG_BUF_ID)pMemMap;
    pMemMap += UART_SEND_BUF_SIZE + sizeof(RNG_BUF);
    pMemMap = ALIGN_4(pMemMap);
    
    //uart_SendBuf = (RNG_BUF_ID)au8SendBuf;

    uart_RecvBuf = (RNG_BUF_ID)pMemMap;
    pMemMap += UART_RECV_BUF_SIZE + sizeof(RNG_BUF) + 64;
    pMemMap = ALIGN_4(pMemMap);
    
    //uart_RecvBuf = (RNG_BUF_ID)au8RecvBuf;
    
    //Enable clock SCI0, SCI1, SCI2, UART0, and GPIO.
    //SYSCTRL->CG_CTRL = U32_BIT6 | U32_BIT7 | U32_BIT8 | U32_BIT10 | U32_BIT21;
	SYSCTRL->CG_CTRL1 = ~0UL;
	
	//SCI0->SCI_IIR = 0x80;
    //Reset SCI0, SCI1, SCI2, UART0, and GPIO.
    SYSCTRL->SOFT_RST1 = ~BIT(20);

    GPIO_ALT_GROUP[0] = 0;
    GPIO_ALT_GROUP[1] = 0;
    GPIO_ALT_GROUP[2] = 0;
    //card detect
    SYSCTRL->PHER_CTRL |= BIT(16);
    //card power on
	//SYSCTRL->PHER_CTRL &= ~(BIT(20) | BIT(21) | BIT(22));
    SYSCTRL->PHER_CTRL |= BIT(20);
    
    
    
    
    
    //SYSCTRL->PHER_CTRL |= BIT(18) | BIT(19) | BIT(20) | BIT(21) | BIT(22) | BIT(23);
    //SYSCTRL->PHER_CTRL &= ~(BIT(21) | BIT(22) | BIT(23));
    //SYSCTRL->PHER_CTRL = ( BIT21 | BIT22 | BIT23);
    //SYSCTRL->PHER_CTRL = 0;
	//GPIO->PC_ALT &= ~(PIN_ALT_MASK(10) | PIN_ALT_MASK(11) | PIN_ALT_MASK(12) | PIN_ALT_MASK(13) | PIN_ALT_MASK(14));
	//GPIO->PC_ALT |= PIN_ALT(10, 0x01) | PIN_ALT(11, 0x01) | PIN_ALT(12, 0x01) | PIN_ALT(13, 0x01) | PIN_ALT(14, 0x01);

	//GPIO->PB_ALT &= ~(PIN_ALT_MASK(0) | PIN_ALT_MASK(1) | PIN_ALT_MASK(2) | PIN_ALT_MASK(3) | PIN_ALT_MASK(4));
	//GPIO->PB_ALT |= PIN_ALT(0, 0x03) | PIN_ALT(1, 0x03) | PIN_ALT(2, 0x03) | PIN_ALT(3, 0x03) | PIN_ALT(4, 0x03);

	//SYSCTRL->FREQ_SEL = 0x38;

	u32Ms = SYSCTRL->PCLK_1MS_VAL * 5;
	uart_Config(115200, 0);
    slot = 0;

	//BOOT_DBG("Input slot:\n");
	//while (-1 == (slot = uart_RecvChar()));
	//BOOT_DBG("0x%02x\n", slot);
    BOOT_DBG("Input Freq:\n");
    while (-1 == (u32Recv = uart_RecvChar()));
    BOOT_DBG("0x%02x\n", u32Recv);
    while (0 == uart_IsSendFinish());
    if (slot > 2)
    {
        slot = 2;
    }
	//SYSCTRL->FREQ_SEL = (SYSCTRL->FREQ_SEL & ~0xFFUL) | (u32Recv & 0xFF);
	//u32Ms = SYSCTRL->PCLK_1MS_VAL * 5;
	//uart_Config((u32Ms + (1152 << 2))/ (1152 << 3));

    BOOT_DBG("Freq is %03x slot is %d\n", SYSCTRL->FREQ_SEL, slot);
    /* SCI0 GPIO Remap */
    GPIO_PinRemapConfig(GPIOA, GPIO_Pin_4  | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_14, GPIO_Remap_0);
    /* SCI1 GPIO Remap */
    //GPIO_PinRemapConfig(GPIOC, GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9, GPIO_Remap_0);
    /* SCI2 GPIO Remap */
    //GPIO_PinRemapConfig(GPIOC, GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14, GPIO_Remap_0);

    //while(0 != argc);
    SYSCTRL->SCI_GLF = SYSCTRL->SCI_GLF & ~BIT(29) | BIT(28);
    SCI_ConfigEMV(0x01, 3000000);

    NVIC_Configuration();
	
    /*
    SYSCTRL->WAKE_EN1 = BIT(16);
    SYSCTRL->WAKE_EN2 = 0x00;
    SYSCTRL->FREQ_SEL = (SYSCTRL->FREQ_SEL & 0xFFFF);
    SCI2->SCI_IER = BIT(0);
    *(uint32_t *)(0x4001F000 + 0x0034) |= 0x40;
    __ASM{WFI};
    */
	//uart_SendBuff("This is for Test SCI!\r\n", sizeof("This is for Test SCI!\r\n") - 1);


	s32Recv = -1;
	while ('r' != s32Recv && 'c' != s32Recv)
	{
		BOOT_DBG("Input 'r' for WarmReset, 'c' for ColdReset!\n");
		while (-1 == (s32Recv = uart_RecvChar()));
		BOOT_DBG("%c\n", s32Recv);
	}
    if ('r' == s32Recv)
    {
        tst_SCIWarmReset(slot);
    }
    BOOT_DBG("Test ColdReset!\n");
    
    state = iso7816_device_init();
    if (state)
    {
        BOOT_DBG("device init error %d\n", state);
        while (1);
    }

    while(ICC_ERR_NOCARD == (state = iso7816_init(slot, 0, atr_buf )));
	
    if (state)
    {
        BOOT_DBG("recv ATR error %d\n", state);
        while (1);
    }
	
    BOOT_DBG("recv ATR!\n");
    memcpy(apdu_req.cmd, "\x00\xa4\x04\x00", 4);
    apdu_req.le = 256;
    apdu_req.lc = 14;
    memcpy( apdu_req.data_in, "1PAY.SYS.DDF01", 14);

    state = iso7816_exchange(slot, 1, &apdu_req, &apdu_rsp);

    if (state)
    {
        BOOT_DBG("recv response1 %d\n", state);
        while (1);
    }

        memcpy( apdu_req.cmd, "\xf0\xd4\x00\x00", 4 );
        apdu_req.lc = 32;
        apdu_req.le = 0;
        memcpy( apdu_req.data_in, "1PAY.SYS.DDF01", 14 );

        memcpy( apdu_req.cmd, "\xf0\xd1\x00\x00", 4 );
        apdu_req.lc = 0;
        apdu_req.le = 0;
        apdu_req.cmd[2] = 0;

        state = iso7816_exchange(slot, 1, &apdu_req, &apdu_rsp);

        if (state)
        {
            BOOT_DBG("recv response2 %d\n", state);
            while (1);
        }
    
    BOOT_DBG("ColdReset test finished, reset!\n");
    while (0 == uart_IsSendFinish());
	SYSCTRL->LOCK_R = 0;
	SYSCTRL->SOFT_RST2 = BIT(31);
    while(1) 
    {
		
    }

}

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
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
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





