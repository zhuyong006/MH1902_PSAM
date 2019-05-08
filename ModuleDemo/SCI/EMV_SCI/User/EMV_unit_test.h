



#ifndef EMV_UNIT_TEST
    #define EMV_UNIT_TEST

    #ifdef __cplusplus
    extern "C"
    {
    #endif
    
    #include <stdint.h>
    
    #define EMV_T0_1737             (998)
    
    #define EMV_T0_1707_2           (999)
    #define EMV_T0_1738_0           (1001)
    #define EMV_T0_1738_1           (1002)
    #define EMV_T0_1738_2           (1003)
    
    #define EMV_T1_1770_2           (1004)
    #define EMV_T1_1771             (1005)
    #define EMV_T1_1772             (1006) 
    #define EMV_T1_1774             (1007)
    #define EMV_T1_1777             (1008)
    #define EMV_T1_1778             (1009)
    #define EMV_T1_1784             (10010)
    #define EMV_T1_1809_01          (10011)
    #define EMV_T1_1809_02          (10012)
    #define EMV_T1_1809_03          (10013)
    #define EMV_T1_1809_04          (10014)
    #define EMV_T1_1809_05          (10015)
    #define EMV_T1_1809_06          (10016)
    #define EMV_T1_1809_07          (10017)
    #define EMV_T1_1809_08          (10018)
    #define EMV_T1_1809_09          (10019)
    #define EMV_T1_1809_11          (10020)
    #define EMV_T1_1809_12          (10021)
    #define EMV_T1_1809_13          (10022)
    #define EMV_T1_1809_14          (10023)
    #define EMV_T1_1809_15          (10024)
    
    
    
    #define EMV_UNIT_TEST         (0)
    
    typedef struct
    {
        uint32_t  u32Size;
        uint8_t * pu8Str;
    } EMV_RECV_CMD;
    
    static uint8_t gu8Step = 0;
    
    #if (EMV_T0_1707_2 == EMV_UNIT_TEST)
    #define ATR     "\x3B\x40\x00\x00"
    #define ATR_W   "\x3B\x40\x00\x00"
    
    #define CMD1    "\xA4"
    #define CMD2    "\x61\x15"
    static EMV_RECV_CMD gcmdRecv[] =
    {
        { sizeof(CMD1), CMD1 },
        { sizeof(CMD2), CMD2 }
    };
    
    #elif (EMV_T0_1707_3 == EMV_UNIT_TEST)
    #define ATR     "\x3B\x20\x00\x00"
    #define ATR_W   "\x3B\x20\x00\x00"
    
    #define CMD1    "\xA4"
    #define CMD2    "\x61\x15"
    static EMV_RECV_CMD gcmdRecv[] =
    {
        { sizeof(CMD1), CMD1 },
        { sizeof(CMD2), CMD2 }
    };
    
    #elif (EMV_T0_1737 == EMV_UNIT_TEST)
    #define ATR     "\x3B\xF0\x13\x00\x00\x10\x80"
    #define ATR_W   "\x3B\xF0\x13\x00\x00\x10\x80"
    
    #define CMD1    "\xA4"
    #define CMD2    "\x61\x15"
    static EMV_RECV_CMD gcmdRecv[] =
    {
        { sizeof(CMD1), CMD1 },
        { sizeof(CMD2), CMD2 }
    };
    /*1738*/
    #elif (EMV_T0_1738_2 == EMV_UNIT_TEST)
    #define ATR     "\x3B\x60\x00\x00"
    #define ATR_W   "\x3B\x60\x00\x00"
    #define CMD1    "\xA4"
    #define CMD2    "\x61\x15"
    #define CMD3    "\xC0\x00\x82\x00\x00\x10\x00\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0A\x0B\x0C\x0D\x0E\x0F\x90\x00"
    #define CMD4    "\x82"
    #define CMD5    "\x62\x83"
    //#define CMD5    "\x63\x35"
    //#define CMD5    "\x9F\xFF"
    #define CMD6    "\x61\x05"
    #define CMD7    "\xC0\x00\x70\x00\x00\x00\x90\x00"
    static EMV_RECV_CMD gcmdRecv[] = 
    {
        {sizeof(CMD1), CMD1},
        {sizeof(CMD2), CMD2},
        {sizeof(CMD3), CMD3},
        {sizeof(CMD4), CMD4},
        {sizeof(CMD5), CMD5},
        {sizeof(CMD6), CMD6},
        {sizeof(CMD7), CMD7}
    };
    
    /*1770*/
    #elif (EMV_T_1770_2 == EMV_UNIT_TEST)
    
    
    /*1771*/
    #elif (EMV_T1_1771 == EMV_UNIT_TEST)
    #define ATR    "\x3B\xE0\x00\x00\x81\x31\x20\x01\x71"
    #define ATR_W   ATR
    #define CMD1   "\x00\xE1\x01\xFE\x1E"
    #define CMD2   "\x00\x00\x07\x00\xB2\x07\x04\x00\x90\x00\x26"
    #define CMD3   "\x00\xC1\x01\xFF\x3F"
    #define CMD4   "\x00\x91\x00\x91"
    #define CMD5   "\x00\x40\x07\x00\xB2\x08\x04\x00\x90\x00\x69"
    #define CMD6   "\x00\xA2\x00\xA2"
    #define CMD7   "\x00\x82\x00\x82"
    #define CMD8   "\x00\x00\x07\x00\xB2\x09\x04\x00\x90\x00\x28"
    #define CMD9   "\x00\x81\x00\x81"
    #define CMD10   "\x00\x91\x00\x91"
    static EMV_RECV_CMD gcmdRecv[] = 
    {
        {sizeof(CMD1), CMD1},
        {sizeof(CMD2), CMD2},
        {sizeof(CMD3), CMD3},
        {sizeof(CMD4), CMD4},
        {sizeof(CMD5), CMD5},
        {sizeof(CMD6), CMD6},
        {sizeof(CMD7), CMD7},
        {sizeof(CMD8), CMD8},
        {sizeof(CMD9), CMD9},
        {sizeof(CMD10), CMD10},
    };
    
    /*1772*/
    #elif (EMV_T1_1772 == EMV_UNIT_TEST)
    
    
    /*1774*/
    #elif (EMV_T1_1774 == EMV_UNIT_TEST)
    #define ATR     "\x3B\xE0\x00\x00\x81\x31\x20\x01\x71"
    #define ATR_W   ATR 
    #define CMD1    "\x00\xE1\x01\xFE\x1E"
    #define CMD2    "\x00\x00\x07\xB2\x01\x04\x00\x90\x00\x20\xAA"
    #define CMD3    "\x00\x00\x07\x00\xB2\x01\x04\x00\x90\x00\x8A"
    #define CMD4    "\x00\x91\x00\x91"
    #define CMD5    "\x00\x00\x07\x00\xB2\x01\x04\x00\x90\x00\x20"
    static EMV_RECV_CMD gcmdRecv[] = 
    {
        {sizeof(CMD1), CMD1},
        {sizeof(CMD2), CMD2},
        {sizeof(CMD3), CMD3},
        {sizeof(CMD4), CMD4},
        {sizeof(CMD5), CMD5}
    };
    
    /*1777*/
    #elif (EMV_T1_1777 == EMV_UNIT_TEST)
    #define ATR     "\x3B\xE0\x00\x00\x81\x31\x20\x01\x71"
    #define ATR_W   ATR 
    #define CMD1    "\x00\xE1\x01\xFE\x1E"
    #define CMD2    "\x00\x00\x07\xB2\x01\x04\x00\x90\x00\x20\xAA"
    #define CMD3    "\x00\x92\x00\x92"
    #define CMD4    "\x00\x00\x07\x00\xB2\x01\x04\x00\x90\x00\x20"
    #define CMD5    "\x00\x40\x07\x00\xB2\x02\x04\x00\x90\x00\xC9"
    #define CMD6    "\x00\x81\x00\x81"
    #define CMD7    "\x00\x40\x07\x00\xB2\x02\x04\x00\x90\x00\x63"
    #define CMD8    "\x00\x00\x07\xB2\x03\x04\x00\x90\x00\x23\xAA"
    static EMV_RECV_CMD gcmdRecv[] = 
    {
        {sizeof(CMD1), CMD1},
        {sizeof(CMD2), CMD2},
        {sizeof(CMD3), CMD3},
        {sizeof(CMD4), CMD4},
        {sizeof(CMD5), CMD5},
        {sizeof(CMD6), CMD6},
        {sizeof(CMD7), CMD7},
        {sizeof(CMD8), CMD8}
    };
    
    /*1778*/
    #elif (EMV_T1_1778 == EMV_UNIT_TEST)    
    #define ATR     "\x3B\xE0\x00\x00\x81\x31\x20\x01\x71"
    #define ATR_W   ATR 
    #define CMD1    "\x00\xE1\x01\xFE\x1E"
    #define CMD2    "\x00\x20\x20\x00\xA4\x04\x00\x21\x00\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0A\x0B\x0C\x0D\x0E\x0F\x10\x11\x12\x13\x14\x15\x16\x17\x18\x19\x1A\x9A"
    #define CMD3    "\x00\x91\x00\xAA"
    #define CMD4    "\x00\x40\x09\x1B\x1C\x1D\x1E\x1F\x20\x00\x90\x00\xE2"
    #define CMD5    "\x00\x80\x00\x80"
    #define CMD6    "\x00\x20\x20\x00\xA4\x04\x00\x3A\x20\x21\x22\x23\x24\x25\x26\x27\x28\x29\x2A\x2B\x2C\x2D\x2E\x2F\x30\x31\x32\x33\x34\x35\x36\x37\x38\x39\x3A\xA1"
    #define CMD7    "\x00\x92\x00\x38"
    #define CMD8    "\x00\x60\x20\x3B\x3D\x3E\x3F\x40\x41\x42\x43\x44\x45\x46\x47\x48\x49\x4A\x4B\x4C\x4D\x4E\x4F\x50\x51\x52\x53\x54\x55\x56\x57\x58\x59\x5A\x5B\x47"
    
    static EMV_RECV_CMD gcmdRecv[] = 
    {
        {sizeof(CMD1), CMD1},
        {sizeof(CMD2), CMD2},
        {sizeof(CMD3), CMD3},
        {sizeof(CMD4), CMD4},
        {sizeof(CMD5), CMD5},
        {sizeof(CMD6), CMD6},
        {sizeof(CMD7), CMD7}
    };
    
    /*1784*/
    #elif (EMV_T1_1784 == EMV_UNIT_TEST)
    #define ATR     "\x3B\xE0\x00\x00\x81\x31\x20\x01\x71"
    #define ATR_W   ATR 
    #define CMD1     "\x00\xE1\x01\xFE\x1E"
    #define CMD2     "\x00\x82\x00\x82"
    #define CMD3     "\x00\x20\x20\x00\xA4\x00\x21\x00\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0A\x0B\x0C\x0D\x0E\x0F\x10\x11\x12\x13\x14\x15\x16\x17\x18\x19\x1A\x1B\xAA"
    #define CMD4     "\x00\x20\x20\x00\xA4\x04\x00\x21\x00\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0A\x0B\x0C\x0D\x0E\x0F\x10\x11\x12\x13\x14\x15\x16\x17\x18\x19\x1A\x9A"
    #define CMD5     "\x00\x40\x09\x1B\x1C\x1D\x1E\x1F\x20\x00\x90\x00\xE2"
    #define CMD6     "\x00\x80\x00\x80"
    #define CMD7     "\x00\x80\x00\x80"
    #define CMD8     "\x00\x20\x20\x00\xA4\x00\x3A\x20\x21\x22\x23\x24\x25\x26\x27\x28\x29\x2A\x2B\x2C\x2D\x2E\x2F\x30\x31\x32\x33\x34\x35\x36\x37\x38\x39\x3A\x3B\xAA"
    #define CMD9     "\x00\x20\x20\x00\xA4\x00\x3A\x20\x21\x22\x23\x24\x25\x26\x27\x28\x29\x2A\x2B\x2C\x2D\x2E\x2F\x30\x31\x32\x33\x34\x35\x36\x37\x38\x39\x3A\x3B\x9E"
    #define CMD10    "\x00\x60\x20\x3B\x3D\x3E\x3F\x40\x41\x42\x43\x44\x45\x46\x47\x48\x49\x4A\x4B\x4C\x4D\x4E\x4F\x50\x51\x52\x53\x54\x55\x56\x57\x58\x59\x5A\x5B\xAA"
    #define CMD11    "\x00\x60\x20\x3B\x3C\x3D\x3E\x3F\x40\x41\x42\x43\x44\x45\x46\x47\x48\x49\x4A\x4B\x4C\x4D\x4E\x4F\x50\x51\x52\x53\x54\x55\x56\x57\x58\x59\x5A\x20"
    #define CMD12    "\x00\x00\x02\x90\x00\x92"
    #define CMD13    "\x00\x80\x00\x80"
    static EMV_RECV_CMD gcmdRecv[] = 
    {
        {sizeof(CMD1), CMD1},
        {sizeof(CMD2), CMD2},
        {sizeof(CMD3), CMD3},
        {sizeof(CMD4), CMD4},
        {sizeof(CMD5), CMD5},
        {sizeof(CMD6), CMD6},
        {sizeof(CMD7), CMD7},
        {sizeof(CMD8), CMD8},
        {sizeof(CMD9), CMD9},
        {sizeof(CMD10), CMD10},
        {sizeof(CMD11), CMD11},
        {sizeof(CMD12), CMD12},
        {sizeof(CMD13), CMD13}
    };

    /*1809_01*/
    #elif (EMV_T1_1809_01 == EMV_UNIT_TEST)
    #define ATR     "\x3B\xE0\x00\x00\x81\x31\x20\x01\x71"
    #define ATR_W   ATR 
    #define CMD1    "\x00\xE1\x01\xFE\x1E"
    #define CMD2    "\x00\xC1\x01\xFE\x3E"
    #define CMD3    "\x00\x00\x28\x00\xA4\x00\x20\x00\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0A\x0B\x0C\x0D\x0E\x0F\x10\x11\x12\x13\x14\x15\x16\x17\x18\x19\x1A\x1B\x1C\x1D\x1E\x1F\x20\x21\x22\x23\xAD"
    #define CMD4    "\x00\x92\x00\x92"
    #define CMD5    "\x00\x00\x28\x00\xA4\x04\x00\x20\x00\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0A\x0B\x0C\x0D\x0E\x0F\x10\x11\x12\x13\x14\x15\x16\x17\x18\x19\x1A\x1B\x1C\x1D\x1E\x1F\x20\x21\x22\x8B"
    #define CMD6    "\x00\x40\x07\x11\x70\x33\x44\x55\x90\x00\x94"
    
    static EMV_RECV_CMD gcmdRecv[] = 
    {
        {sizeof(CMD1), CMD1},
        {sizeof(CMD2), CMD2},
        {sizeof(CMD3), CMD3},
        {sizeof(CMD4), CMD4},
        {sizeof(CMD5), CMD5},
        {sizeof(CMD6), CMD6}
    };
    
    /*1809_01*/
    #elif (EMV_T1_1809_02 == EMV_UNIT_TEST)
    
    /*1809_04*/
    #elif (EMV_T1_1809_04 == EMV_UNIT_TEST)
    #define ATR     "\x3B\xE0\x00\x00\x81\x31\x20\x01\x71"
    #define ATR_W   ATR 
    #define CMD1    "\x00\xE1\x01\xFE\x1E"
    #define CMD2    "\x00\xC1\x01\xFE\x3E"
    #define CMD3    "\x88\x00\x28\x00\xA4\x00\x20\x00\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0A\x0B\x0C\x0D\x0E\x0F\x10\x11\x12\x13\x14\x15\x16\x17\x18\x19\x1A\x1B\x1C\x1D\x1E\x1F\x20\x21\x22\x23\x24"
    #define CMD4    "\x00\x92\x00\x92"
    #define CMD5    "\x00\x00\x28\x00\xA4\x04\x00\x20\x00\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0A\x0B\x0C\x0D\x0E\x0F\x10\x11\x12\x13\x14\x15\x16\x17\x18\x19\x1A\x1B\x1C\x1D\x1E\x1F\x20\x21\x22\x8B"
    #define CMD6    "\x00\x40\x07\x11\x70\x33\x44\x55\x90\x00\x94"
    
    static EMV_RECV_CMD gcmdRecv[] = 
    {
        {sizeof(CMD1), CMD1},
        {sizeof(CMD2), CMD2},
        {sizeof(CMD3), CMD3},
        {sizeof(CMD4), CMD4},
        {sizeof(CMD5), CMD5},
        {sizeof(CMD6), CMD6}
    };
    
    /*1809_06*/
    #elif (EMV_T1_1809_06 == EMV_UNIT_TEST)
    #define ATR     "\x3B\xE0\x00\x00\x81\x31\x20\x01\x71"
    #define ATR_W   ATR 
    #define CMD1    "\x00\xE1\x01\xFE\x1E"
    #define CMD2    "\x00\xC1\x01\xFE\x3E"
    #define CMD3    "\x00\x40\x28\x00\xA4\x00\x20\x00\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0A\x0B\x0C\x0D\x0E\x0F\x10\x11\x12\x13\x14\x15\x16\x17\x18\x19\x1A\x1B\x1C\x1D\x1E\x1F\x20\x21\x22\x23\xAD"
    #define CMD4    "\x00\x92\x00\x92"
    #define CMD5    "\x00\x00\x28\x00\xA4\x04\x00\x20\x00\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0A\x0B\x0C\x0D\x0E\x0F\x10\x11\x12\x13\x14\x15\x16\x17\x18\x19\x1A\x1B\x1C\x1D\x1E\x1F\x20\x21\x22\x8B"
    #define CMD6    "\x00\x40\x07\x11\x70\x33\x44\x55\x90\x00\x94"
    
    static EMV_RECV_CMD gcmdRecv[] = 
    {
        {sizeof(CMD1), CMD1},
        {sizeof(CMD2), CMD2},
        {sizeof(CMD3), CMD3},
        {sizeof(CMD4), CMD4},
        {sizeof(CMD5), CMD5},
        {sizeof(CMD6), CMD6}
    };
    
    /*1809_06*/
    #elif (EMV_T1_1809_07 == EMV_UNIT_TEST)
    #define ATR     "\x3B\xE0\x00\x00\x81\x31\x20\x01\x71"
    #define ATR_W   ATR 
    #define CMD1    "\x00\xE1\x01\xFE\x1E"
    #define CMD2    "\x00\xC1\x01\xFE\x3E"
    #define CMD3    "\x00\xC3\x02\x06\x06\xC1"
    #define CMD4    "\x00\x92\x00\x92"
    #define CMD5    "\x00\x00\x28\x00\xA4\x04\x00\x20\x00\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0A\x0B\x0C\x0D\x0E\x0F\x10\x11\x12\x13\x14\x15\x16\x17\x18\x19\x1A\x1B\x1C\x1D\x1E\x1F\x20\x21\x22\x8B"
    #define CMD6    "\x00\x40\x07\x11\x70\x33\x44\x55\x90\x00\x94"
    
    static EMV_RECV_CMD gcmdRecv[] = 
    {
        {sizeof(CMD1), CMD1},
        {sizeof(CMD2), CMD2},
        {sizeof(CMD3), CMD3},
        {sizeof(CMD4), CMD4},
        {sizeof(CMD5), CMD5},
        {sizeof(CMD6), CMD6}
    };
    
    /*1809_06*/
    #elif (EMV_T1_1809_11 == EMV_UNIT_TEST)
    #define ATR     "\x3B\xE0\x00\x00\x81\x31\x20\x01\x71"
    #define ATR_W   ATR 
    #define CMD1    "\x00\xE1\x01\xFE\x1E"
    #define CMD2    "\x00\xC1\x01\xFE\x3E"
    #define CMD3    "\x00\xE3\x01\x06\xE4"
    #define CMD4    "\x00\x92\x00\x92"
    #define CMD5    "\x00\x00\x28\x00\xA4\x04\x00\x20\x00\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0A\x0B\x0C\x0D\x0E\x0F\x10\x11\x12\x13\x14\x15\x16\x17\x18\x19\x1A\x1B\x1C\x1D\x1E\x1F\x20\x21\x22\x8B"
    #define CMD6    "\x00\x40\x07\x11\x70\x33\x44\x55\x90\x00\x94"
    
    static EMV_RECV_CMD gcmdRecv[] = 
    {
        {sizeof(CMD1), CMD1},
        {sizeof(CMD2), CMD2},
        {sizeof(CMD3), CMD3},
        {sizeof(CMD4), CMD4},
        {sizeof(CMD5), CMD5},
        {sizeof(CMD6), CMD6}
    };
    
    
    
    
    #endif

    #ifdef __cplusplus
    }
    #endif
#endif
