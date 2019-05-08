
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MHSCPU_CONF_H
#define __MHSCPU_CONF_H

    /* Includes ------------------------------------------------------------------*/
    /* Uncomment the line below to enable peripheral header file inclusion */
    #include "mhscpu_uart.h"
    #include "mhscpu_sysctrl.h"
    #include "mhscpu_spi.h"
    #include "mhscpu_wdt.h"
    #include "mhscpu_crc.h"
    #include "mhscpu_timer.h"
    #include "mhscpu_gpio.h"
    #include "mhscpu_rtc.h"
    #include "mhscpu_sensor.h"
    #include "mhscpu_bpk.h"
    #include "mhscpu_trng.h"
    #include "misc.h" 

    #include "string.h"

    
    #define SCI_DBG
    
        #ifdef SCI_DBG
            #if   defined ( __CC_ARM )
                #define DBG_PRINT(format, args...)  printf(format, ##args)

            #elif defined ( __GNUC__ )
                #define DBG_PRINT(format, args...)  printf_GCC(format, ##args)

            #elif defined ( __ICCARM__ )

            #endif

                
        #define DBG_PRINT_HEX(s, buf, len)\
                print_hex(s, buf, len)
    #else
        #define DBG_PRINT(fmt, args...)
        #define DBG_PRINT_HEX(s, buf, len)
    #endif
    
    
    
    #ifdef  USE_FULL_ASSERT

        #define assert_param(expr) ((expr) ? (void)0 : assert_failed((uint8_t *)__FILE__, __LINE__))
        void assert_failed(uint8_t* file, uint32_t line);
    #else
        #define assert_param(expr) ((void)0)
    #endif /* USE_FULL_ASSERT */

#endif 

