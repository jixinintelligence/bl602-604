#include <stdint.h>
#include "bl602.h"

#define __STARTUP_CLEAR_BSS             1

/*----------------------------------------------------------------------------
  Linker generated Symbols
 *----------------------------------------------------------------------------*/
extern uint32_t __etext0;
extern uint32_t __etext1;
extern uint32_t __etext2;
extern uint32_t __tcm_code_start__;
extern uint32_t __tcm_code_end__;
extern uint32_t __ocram_data_start__;
extern uint32_t __ocram_data_end__;
extern uint32_t __data_start__;
extern uint32_t __data_end__;
extern uint32_t __copy_table_start__;
extern uint32_t __copy_table_end__;
extern uint32_t __zero_table_start__;
extern uint32_t __zero_table_end__;
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
extern uint32_t __StackTop;

void start_load(void) {
    uint32_t *pSrc, *pDest;
    uint32_t *pTable __attribute__((unused));  

    /* Copy TCM code */
    pSrc  = &__etext0;
    pDest = &__tcm_code_start__;
    for ( ; pDest < &__tcm_code_end__ ; ) {
        *pDest++ = *pSrc++;
    }
    /* BF Add OCRAM data copy */
    pSrc  = &__etext1;
    pDest = &__ocram_data_start__;
    for ( ; pDest < &__ocram_data_end__ ; ) {
        *pDest++ = *pSrc++;
    }
    /* BF Add TCM data copy */
    pSrc  = &__etext2;
    pDest = &__data_start__;
    for ( ; pDest < &__data_end__ ; ) {
        *pDest++ = *pSrc++;
    }
#ifdef __STARTUP_CLEAR_BSS
    /*  Single BSS section scheme.
     *
     *  The BSS section is specified by following symbols
     *    __bss_start__: start of the BSS section.
     *    __bss_end__: end of the BSS section.
     *
     *  Both addresses must be aligned to 4 bytes boundary.
     */
    pDest = &__bss_start__;
    for ( ; pDest < &__bss_end__ ; ) {
        *pDest++ = 0ul;
    }
#endif 
}