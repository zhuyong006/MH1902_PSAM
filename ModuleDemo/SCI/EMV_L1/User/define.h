
#ifndef MHSMCU_DEFINE
    #define MHSMCU_DEFINE

    #ifdef __cplusplus
    extern "C"
    {
    #endif

    #include <stdint.h>

    #define MEM_START       (0x20000000UL)

    #define BIT(n)          (1UL << (n))


    #define RNG_BUF_IS_FULL(pBuf) ((pBuf)->tail == (pBuf)->head)
    #define RNG_BUF_NEXT_HEAD(pBuf) (((pBuf)->head + 1) % ((pBuf)->buf_size + 1))
    #define RNG_BUF_NEXT_TAIL(pBuf) (((pBuf)->tail + 1) % ((pBuf)->buf_size + 1))
    #define RNG_BUF_LEN(pBuf) (((pBuf)->tail + (pBuf)->buf_size - (pBuf)->head) % ((pBuf)->buf_size + 1))

    typedef struct 
    {
        uint16_t head;
        uint16_t tail;
        uint16_t buf_size;
        uint16_t err_stat;
        uint8_t buf[1];     //take place for Data Block
    } RNG_BUF, *RNG_BUF_ID;








    #ifdef __cplusplus
    }
    #endif

#endif

