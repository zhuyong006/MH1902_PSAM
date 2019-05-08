#include <stdio.h>
#include "mhscpu_sci.h"
#include "iso7816_3.h"
#include "emv_errno.h"
#include "emv_hard.h"
#include "test_emv.h"
#include "test_loopback.h"
#include "uart.h"
static uint8_t atr_buf[64];

static ST_APDU_REQ apdu_req;
static ST_APDU_RSP apdu_rsp;
extern struct emv_core emv_devs[];


int32_t PsamColdRest(uint32_t u32Slot)
{
    int32_t s32Slot, state;
	  int i = 0;
    struct emv_core     *pdev;
    struct emv_atr       su_atr;

    memset(atr_buf, 0, sizeof(atr_buf));
    memset(&apdu_req, 0, sizeof(apdu_req));
    memset(&apdu_rsp, 0, sizeof(apdu_rsp));


		DBG_PRINT("Test ColdReset!\n");

    s32Slot = u32Slot;
    s32Slot = select_slot( s32Slot );
    pdev = &( emv_devs[ s32Slot ] );
    pdev->terminal_ch = s32Slot;

	
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
				for(i=0;i<atr_buf[0];i++)
					DBG_PRINT("0x%02x,",atr_buf[i+1]);
    }
		DBG_PRINT("\n");

    return 0;
}

void ReadRandParm(uint8_t u8Slot)
{
	int32_t state = 0;
	int32_t i = 0;
	memcpy( apdu_req.cmd, "\x00\x84\x00\x00", 4 );
	apdu_req.lc = 0;
	apdu_req.le = 4;
	state = iso7816_exchange(u8Slot, 1, &apdu_req, &apdu_rsp);
	if (state)
  {
		DBG_PRINT("recv response2 %d\n", state);
		while (1);
  }
	DBG_PRINT("recv len %d\n", apdu_rsp.len_out);
	
	for(i=0;i<apdu_rsp.len_out;i++)
	{
		DBG_PRINT("0x%02x,",apdu_rsp.data_out[i]);
	}
	DBG_PRINT("\n");
	DBG_PRINT("SW Status:\n");
	DBG_PRINT("swa:0x%02x\n",apdu_rsp.swa);
	DBG_PRINT("swb:0x%02x\n",apdu_rsp.swb);
}	

#if VERSION_410 == EMV_VERSION

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
        wait_Ns(3);
        //Init emv_devs param.
        iso7816_device_init();
        while (0 != iso7816_detect(u8Slot))
        {
            DBG_PRINT("Wait insert ICC!\n");
            delay_Ms(500);
        }
        DBG_PRINT("Insert IC\n");
        if (0 == (s32Stat = iso7816_init(u8Slot, VCC_5000mV | SPD_1X, atr)))
        {
            //DBG_PRINT("iso7816_init() finished.\n");
        }
        else
        {
            DBG_PRINT("iso7816_init() failed %d.\n", s32Stat);
        }

        lrc = 0;
        rsp.len_out = 0;
        while (0 == iso7816_detect(u8Slot) && 0 == s32Stat)
        {
            if ('b' == uart_RecvChar())
            {
                DBG_PRINT("Break loop!\n");
                break;
            }
            if (rsp.len_out < 6)
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
            //delay_Ms(300);
            if (0 != (s32Stat = iso7816_exchange(u8Slot, AUTO_GET_RSP, &apdu_req, &rsp)))
            {
                DBG_PRINT("Exchange failed %d!\n", s32Stat);
                rsp.len_out = 0;
                break;
            }
            /*
            else
            {
                print_hex("recv:", rsp.data_out, rsp.len_out);
                DBG_PRINT("SWA: %x SWB: %x!\n", rsp.swa, rsp.swb);
            }
            */
            
            if (0x6A == rsp.swa && 0x82 == rsp.swb)
            {
                DBG_PRINT("This test Case finished!\n");
                break;
            }
            lrc = cal_lrc(rsp.data_out, rsp.len_out);
            lrc ^= rsp.swa;
            lrc ^= rsp.swb;
        }
        iso7816_close(u8Slot);
        //DBG_PRINT("Shut down ICC!\n");
        /*
        while('c' != s32Stat)
        {
            DBG_PRINT("Press 'c' to continue\n");
            while (-1UL == (s32Stat = uart_RecvChar()));
            DBG_PRINT("%c\n", s32Stat);
        }
        */
    }
}

#elif VERSION_43A == EMV_VERSION

#define ICBS_TEST       (1)

void loop_back(uint8_t u8Slot)
{
    uint8_t u8Wait = 3, *pu8Str = "Class A(5.0V)";
    int32_t s32Vol, s32Stat = 0;
		int32_t s32Command = 0;
    uint8_t  atr[65];


    s32Vol = VCC_3000mV;
#if 1    
    DBG_PRINT("\r\nchoose voltage:\r\n0:5V\r\n1:3V\r\n2:1.8V\r\n3:1.2V\r\nOther:5V\r\n");
    
		while (-1 == (s32Stat = uart_RecvChar()));
    //s32Stat = '1';
    if ('1' == s32Stat)
    {
        s32Vol = VCC_3000mV;
        pu8Str = "Class B(3.0V)";
        //SYSCTRL->CARD_RSVD = (SYSCTRL->CARD_RSVD & ~(0x0F<<2)) | (0x07); //Bit2:5 default 0x07
    }
    else if ('2' == s32Stat)
    {
        s32Vol = VCC_1800mV;
        pu8Str = "Class C(1.8V)";
    }
    else if ('3' == s32Stat)
    {
        s32Vol = VCC_1200mV;
        pu8Str = "User mode(1.2V)";
    }
    else
    {
        s32Vol = VCC_5000mV;
        pu8Str = "Class A(5.0V)";
    }
    DBG_PRINT("Voltage %s\r\n",pu8Str);
#endif    
    //DBG_PRINT("Vol %d\n\n\n", s32Vol);
    while (1)
    {
        
        #if 1 == ICBS_TEST
        //ICBC will not drawn ICC in next test case.
        u8Wait = wait_ChangeNs(u8Wait);
        #else
        wait_Ns(5);
        #endif
        //Init emv_devs param.
        iso7816_device_init();
        while (0 != iso7816_detect(u8Slot))
        {
            DBG_PRINT("Wait insert ICC!\n");
            delay_Ms(500);
        }
        DBG_PRINT("Insert IC\n");
        if (0 == (s32Stat = iso7816_init(u8Slot, s32Vol | SPD_1X, atr)))
        {
            DBG_PRINT("iso7816_init finished!\n");
        }
        else
        {
            DBG_PRINT("iso7816_init failed %d!\n", s32Stat);
        }

LOOP:		
				DBG_PRINT("input operate cmd(r/c):\n");
				while (-1 == (s32Command = uart_RecvChar()));
				if ('r' == s32Command)
				{
						DBG_PRINT("iso7816 reset card\n");
						PsamColdRest(u8Slot);
				}else if('c' == s32Command){
					  DBG_PRINT("ISO7816 Read Rand Parm\n");
						ReadRandParm(u8Slot);
				}
        
				goto LOOP;
        //Deinit struct power down card and.
        iso7816_close(u8Slot);
        DBG_PRINT("Shut down ICC!\n");

    }

}
#endif
