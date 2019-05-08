#ifndef __MHSCPU_LCDI_H
#define __MHSCPU_LCDI_H

#ifdef __cplusplus
    extern "C" {
#endif 


    #include "mhscpu.h"


    #define CMD                  (0)
    #define DAT                  (1)

    #define MODE_6800            (0)
    #define MODE_8080            (1)

    #define LCD_CMD_BUFF_SIZE    (64)
    #define LCD_READ_BUFF_SIZE   (512)
    #define LCD_WRITE_BUFF_SIZE  (512)


    typedef struct
    {
        uint8_t LCD_BusMode;        //Bus mode(8080/6800)
        uint8_t LCD_IntRead;        //Read interrupt Enable
        uint8_t LCD_IntWrite;       //Read interrupt Enable                            
        uint32_t LCD_MaxQTR;        //Max qaurter cylce of read/write.
    } LCD_InitTypeDef;

    //Operate the bus signal
    void LCD_BusRead(uint8_t u8CD);
    void LCD_BusWrite(uint8_t u8CD, uint8_t u8Value);

    void LCD_Read(uint8_t u8CD, uint8_t *pu8Value);
    void LCD_Write(uint8_t u8CD, uint8_t u8Value);

    //Buffer mode only can used in interrupt mode.
    int32_t LCD_ReadBuff(uint8_t opt, uint8_t *pu8Buff, uint32_t u32BuffLen);
    int32_t LCD_WriteBuff(uint8_t opt, uint8_t *pu8Buff, uint32_t u32BuffLen);

    void LCD_Init(LCD_InitTypeDef *pLcdInit);
	void LCD_DMA_WriteBuf(uint8_t *buf,uint32_t lenth);
   void LCD_DMA_WriteBuf_page(uint8_t *buf,uint32_t lenth);
   void LcdDma_Init(uint8_t *base,uint8_t page_lenth);
   void LCD_DMA_WriteBuf_page1(uint8_t *buf);

#ifdef __cplusplus
    }
#endif
	
#endif



