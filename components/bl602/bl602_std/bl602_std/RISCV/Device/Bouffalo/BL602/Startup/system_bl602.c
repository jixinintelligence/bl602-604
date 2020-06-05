#include "bl602.h"
#include "system_bl602.h"
#include "bl602_glb.h"
#ifdef BOOTROM
#include "bflb_bootrom.h"
#endif

/*----------------------------------------------------------------------------
  Define clocks
 *----------------------------------------------------------------------------*/
#define  SYSTEM_CLOCK     ( 32000000UL )

/*----------------------------------------------------------------------------
  Vector Table
 *----------------------------------------------------------------------------*/
#define VECT_TAB_OFFSET  0x00 /*!< Vector Table base offset field.
                                   This value must be a multiple of 0x200. */

/*----------------------------------------------------------------------------
  System initialization function
 *----------------------------------------------------------------------------*/

void SystemInit (void)
{
    uint32_t *p;
    uint32_t i=0;

#ifdef BOOTROM
    extern void GLB_Power_On_LDO18_IO(void);
    extern void WDT_Disable(void);
    extern void HBN_Clear_RTC_INT(void);

    BMX_Cfg_Type bmxCfg={
            .timeoutEn=0,
            .errEn=DISABLE,
            .arbMod=BMX_ARB_FIX
    };
    L1C_BMX_Cfg_Type  l1cBmxCfg={
            .timeoutEn=0,
            .errEn=DISABLE,
            .arbMod=L1C_BMX_ARB_FIX,
    };
    
    /* NP boot log Flag */
    p= (uint32_t *)(BFLB_BOOTROM_NP_BOOT_LOG_ADDR);
    *p=0x5A5AA5A5;
    /*diable BMX error incase Sbooten=0xf,while user send vector(core) reset and CPU read deadbeef,
    if not disable this bit, CPU will also get hardfault at the same time*/
    //GLB->bmx_cfg1.BF.bmx_err_en=0;
    //GLB->bmx_cfg1.BF.bmx_timeout_en=0;
    //GLB->bmx_cfg2.BF.bmx_err_addr_dis=1;
    GLB_BMX_Init(&bmxCfg);
    GLB_BMX_Addr_Monitor_Disable();
    //L1C->l1c_config.BF.l1c_bmx_err_en=0;
    //L1C->l1c_config.BF.l1c_bmx_timeout_en=0;
    //L1C->l1c_bmx_err_addr_en.BF.l1c_bmx_err_addr_dis=1;
    L1C_BMX_Init(&l1cBmxCfg);
    L1C_BMX_Addr_Monitor_Disable();
    //L1C->l1c_config.BF.l1c_way_dis=0xf;
    L1C_Set_Way_Disable(0x0f);
    /* Disable Watchdog */
    WDT_Disable();
    /* Clear RTC */
    HBN_Clear_RTC_INT();
    /* Make OCRAM Idle from retention or sleep */
    GLB_Set_OCRAM_Idle();
    /* Disable embedded flash power up*/
    HBN_Set_Embedded_Flash_Pullup(DISABLE);
#endif
    /* Restore default setting*/
    GLB_UART_Sig_Swap_Set(UART_SIG_SWAP_NONE);
    GLB_JTAG_Sig_Swap_Set(JTAG_SIG_SWAP_NONE);
    /* update SystemCoreClock value */
    SystemCoreClockSet(SYSTEM_CLOCK);
    /* CLear all interrupt */
    p=(uint32_t *)(CLIC_HART0_ADDR+CLIC_INTIE);
    for(i=0;i<(IRQn_LAST+3)/4;i++){
        p[i]=0;
    }
    p=(uint32_t *)(CLIC_HART0_ADDR+CLIC_INTIP);
    for(i=0;i<(IRQn_LAST+3)/4;i++){
        p[i]=0;
    }

    /* global IRQ enable */
    __enable_irq();

#ifdef BOOTROM
    /*Power up soc 11 power domain,TODO: This should be optional */
    //AON_Power_On_SOC_11();
    /* Record LDO18 pu flag before power up. This maybe not neccessary but copy from 606*/
    //BL_WR_WORD(BFLB_BOOTROM_AP_BOOT_LOG_ADDR,GLB->ldo18io.BF.pu_ldo18io);
    /* Power up flash power*/
    //GLB_Power_On_LDO18_IO();
#endif
	
}
void System_NVIC_SetPriority(IRQn_Type IRQn, uint32_t PreemptPriority, uint32_t SubPriority)
{
}
