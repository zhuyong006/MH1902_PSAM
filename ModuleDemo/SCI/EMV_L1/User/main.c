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
#include "test_emv.h"
//#include "test_electrical.h"







#define ALIGN_4(pAddr) ((uint8_t *)((int32_t)((uint8_t *)pAddr + 3) & ~0x03UL))

void NVIC_Configuration(void);
void SCI_Init(void);
uint8_t atr_buf[64];

extern struct emv_core emv_devs[];
#define MH_SMCU_SCU_DES_MASK_BASE       (0x40003600)



extern uint32_t __Vectors;
extern RNG_BUF_ID  uart_SendBuf;
extern RNG_BUF_ID  uart_RecvBuf;
extern RNG_BUF_ID  spi_SendBuf;
extern RNG_BUF_ID  spi_RecvBuf;


//uint8_t au8RecvBuf[UART_RECV_BUF_SIZE + sizeof(RNG_BUF) + 32];
//uint8_t au8SendBuf[UART_SEND_BUF_SIZE + sizeof(RNG_BUF) + 32];






int main(int argc, char *argv[])
{
    uint32_t u32Ms;
    uint8_t  au8Ver[32];

    
    //Enable clock SCI0, SCI1, SCI2, UART0, and GPIO.
    SYSCTRL_APBPeriphClockCmd(SYSCTRL_APBPeriph_SCI0 | SYSCTRL_APBPeriph_UART0 | SYSCTRL_APBPeriph_GPIO | SYSCTRL_APBPeriph_TIMM0, ENABLE);
    SYSCTRL_APBPeriphResetCmd(SYSCTRL_APBPeriph_SCI0 | SYSCTRL_APBPeriph_UART0 | SYSCTRL_APBPeriph_TIMM0, ENABLE);

    
#define USE_EX_12M          1

#if 1 == USE_EX_12M
    SYSCTRL_SYSCLKSourceSelect(SELECT_EXT12M);
#else
    //Set Bit9 use internal clock.
    SYSCTRL_SYSCLKSourceSelect(SELECT_INC12M);
#endif
    
    //card detect
    SYSCTRL->PHER_CTRL |= BIT(16);
    //Choose active level(Low level active).
    SYSCTRL->PHER_CTRL |= BIT(20);

    uart_Config(115200, 0);

    GPIO_GROUP[0].OEN &= ~BIT(2);
    GPIO_GROUP[0].BSRR = BIT(2 + 16);
    GPIO_PinRemapConfig(GPIOA, GPIO_Pin_2, GPIO_Remap_1);
    
    //GPIO_GROUP[0].PUE &= ~(BIT(6) | BIT(8) | BIT(9) | BIT(10) | BIT(14));
    GPIO_GROUP[0].PUE &= ~(BIT(6) | BIT(8) | BIT(9) | BIT(14));
    GPIO_GROUP[0].PUE |= BIT(10);
        
    GPIO_PinRemapConfig(GPIOA, GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_14, GPIO_Remap_0);
		
		GPIO_PinRemapConfig(GPIOA, GPIO_Pin_6, GPIO_Remap_1);
		SYSCTRL->PHER_CTRL &= ~BIT(16);
		
    SYSCTRL->SCI_GLF = SYSCTRL->SCI_GLF & ~BIT(29) | BIT(28); //5V
    //SYSCTRL->SCI_GLF = SYSCTRL->SCI_GLF & ~(0x03 << 28) | (0x02 << 28);   //3V
    //SYSCTRL->SCI_GLF |= BIT(31);
    SYSCTRL->SCI_GLF &= ~0xFFFFF;
    SYSCTRL->SCI_GLF |= SYSCTRL->PCLK_1MS_VAL;
    SYSCTRL->CARD_RSVD |= BIT(0); //High TH(400mA protect)
    //PA6-CARD_DECTECT PA8-CLK PA9-RSTN PA10-IO PA14-VCC_EN
    //GPIO_GROUP[0].PUE |= BIT(10) | BIT(9);
    //GPIO_GROUP[0].PUE &= ~BIT(10);
    SCI_ConfigEMV(0x01, 4000000);

    NVIC_Configuration();
    //SSC->SSC_CR1 &= ~0xFF;

    //electrical_test(0);
#if 1 == USE_EX_12M
        DBG_PRINT("Board use external clock");  
#else
        DBG_PRINT("Board use internal clock");  
#endif

    DBG_PRINT("EMV Lib version 0x%08x\r\n", iso7816_get_version());
    DBG_PRINT("EMV Lib version %s\r\n", iso7816_lib_date());
    DBG_PRINT("EMV Lib version %s\r\n", iso7816_lib_time());
    
		loop_back(0);
    return 0;
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





