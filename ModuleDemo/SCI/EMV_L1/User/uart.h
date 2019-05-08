
#ifndef MHSMCU_UART
    #define MHSMCU_UART

    #ifdef __cplusplus
    extern "C"
    {
    #endif

	#include <stdint.h>
    #include <stdarg.h>
    #include <stdlib.h>
    #include <stdio.h>
    //#define UART_SEND_BUF_SIZE   (512)
    //#define UART_RECV_BUF_SIZE   (16 * 1024)
	#define UART_SEND_BUF_SIZE   (1024)
	#define UART_RECV_BUF_SIZE   (256)

    int printf_GCC(char *fmt, ...);
    
    #define UART_INT_MODE        (1)
    void uart_Config(uint32_t baudrate, uint32_t parity);
    void uart_RecvFlush(void);
    int32_t uart_RecvChar(void);
    int32_t uart_IsSendFinish(void);
    int32_t uart_SendChar(uint8_t ch);
    int32_t uart_SendCharBlock(uint8_t ch);
    int32_t uart_SendBuff(uint8_t *pBuf, uint32_t len);

    #ifdef __cplusplus
    }
    #endif

#endif

