/*
 * Copyright (c) 2020 Bouffalolab.
 *
 * This file is part of
 *     *** Bouffalolab Software Dev Kit ***
 *      (see www.bouffalolab.com).
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of Bouffalo Lab nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <stdio.h>
#include "rfc_bl602.h"
#include "reg/reg_bl_rc2.h"
#include "reg/reg_rf.h"
#include "reg/reg_rf_fsm.h"
#include "reg/reg_rf_epa.h"

#define prefix  "[RFC] "
#define RF_PRIVATE_PRESENT

static uint8_t inited = 0;
static double xtalfreq_MHz = 40.0;

static void wait_us(uint32_t);
static void wait_ms(uint32_t);
static void _set_rfc();
static void _set_rf_channel_sw(uint32_t);
static void _set_rf_channel_hw(uint32_t);
static void _calc_sdm_cw(uint32_t xtalfreq,float freq_MHz,uint32_t *sdm_cw);
static void _calc_sdm_params(uint32_t xtalfreq,uint32_t *lo_center_freq_mhz,uint32_t *lo_sdmin_center,uint32_t *lo_sdmin_1m,uint32_t *lo_sdmin_if);

#ifdef RF_PRIVATE_PRESENT
#include "bl602_rf_private.h"
#endif

#ifndef RF_PRIVATE_PRESENT
enum {
    RF_PU_PD = 0,
    RF_PU_SB = 1,
    RF_PU_LO = 2,
    RF_PU_LOCAL = 3,
    RF_PU_RX = 4,
    RF_PU_TX = 6,
    RF_PU_AUTO = 5
};

typedef struct rf_backup_item_tag 
{
    uint32_t addr;
    uint32_t value;
} rf_backup_item_t;

typedef struct 
{
    // uint32_t freq_MHz;
    uint32_t count;
    uint32_t sdm_cw;
    uint8_t  sdm_dither;
    uint8_t  sdm_bypass;
    uint8_t  vco_freq_cw;
    uint8_t  vco_idac_cw;
} rf_lo_cfg_t;

static rf_lo_cfg_t lo_lut[21]; // 2404:4:2484
static rf_backup_item_t backup_regs[16] = {
    {0x40001004,0},
    {0x4000100C,0},
    {0x4000101C,0},
    {0x40001030,0},
    {0x400010B8,0},
    {0x400010C0,0},
    {0x400010C4,0},
    {0x00000000,0},
    {0x00000000,0},
    {0x00000000,0},
    {0x00000000,0},
    {0x00000000,0},
    {0x00000000,0},
    {0x00000000,0},
    {0x00000000,0},
    {0x00000000,0},
};

static void _save_before_cal()
{
    for (uint32_t i = 0; i < sizeof(backup_regs)/sizeof(backup_regs[0]);i++) {
        if (backup_regs[i].addr != 0) {
            backup_regs[i].value = rfc_read_reg(backup_regs[i].addr);
        }
    }
}

static void _restore_after_cal()
{
    for (uint32_t i = 0; i < sizeof(backup_regs)/sizeof(backup_regs[0]);i++) {
        if (backup_regs[i].addr != 0) {
            rfc_write_reg(backup_regs[i].addr,backup_regs[i].value);
        }
    }
}

static void _set_rf_pu(uint32_t pu_mode)
{
    if (pu_mode == RF_PU_AUTO) {
        rf_pu_ctrl_hw_setf(1);
        return;
    } 
    
    // make sure fbdv is on
    // the system crash if fbdv is off and clkpll selects xtal as reference
    rf_pu_fbdv_setf(1);

    if (pu_mode == RF_PU_LOCAL) {
        // configure pu
        rf_pu_lna_setf(0);
        rf_pu_rmxgm_setf(0);
        rf_pu_rmx_setf(0);
        rf_pu_rbb_setf(0);
        rf_pu_adc_setf(0);
        rf_adc_clk_en_setf(0);
        rf_pu_pkdet_setf(0);
        rf_pu_rosdac_setf(0);
        rf_pu_pwrmx_setf(0);
        rf_pu_pa_setf(0);
        rf_pu_tmx_setf(0);
        rf_pu_tbb_setf(0);
        rf_pu_dac_setf(0);
        rf_pu_rxbuf_setf(0);
        rf_pu_txbuf_setf(0);
        rf_trsw_en_setf(0);
        rf_pu_tosdac_setf(0);
        rf_pu_osmx_setf(1);
        rf_pu_pfd_setf(1);
        rf_pu_fbdv_setf(1);
        rf_pu_vco_setf(1);
        // switch to software pu
        rf_pu_ctrl_hw_setf(0);
    }
    else {

    }
}

uint32_t _fcal_meas(uint32_t cw)
{
    uint32_t count1;

    // configure capcode
    rf_lo_vco_freq_cw_setf(cw);
    wait_us(100);
    // start fcal
    rf_fcal_cnt_start_setf(1);
    wait_us(100);
    // wait fcal done
    while(rf_fcal_cnt_rdy_getf() == 0);
    // read fcal count
    count1 = rf_fcal_cnt_op_getf();
    // stop fcal
    rf_fcal_cnt_start_setf(0);

    return count1;
}

// (1) search cw for lower boundary of channel frequency (2400) (max of cw,min of count)
// (2) search cw for each channel (2404:4:2484)
static void _fcal_main(void)
{
    uint32_t k;
    uint32_t count1 = 0xa6a0;
    uint32_t count2 = 0xa6e0;
    uint32_t ncycles = 0x855;
    uint32_t fbdv_div = 4;
    uint32_t fbdv_div_sdm = fbdv_div*(1<<22);
    uint32_t target_count;
    uint32_t bw,cw,cnt,cnt_last,cw_binary;
    uint32_t channel_freq;
    double   vco_freq;
    
    _save_before_cal();

    // configure PU for lo calibration
    _set_rf_pu(RF_PU_LOCAL);

    // turn off hardware control
    rf_adda_ctrl_hw_setf(0);
    rf_rbb_pkdet_out_rstn_ctrl_hw_setf(0);
    rf_rbb_pkdet_en_ctrl_hw_setf(0);
    rf_sdm_ctrl_hw_setf(0);
    rf_inc_fcal_ctrl_en_hw_setf(0);
    rf_inc_acal_ctrl_en_hw_setf(0);
    rf_lo_ctrl_hw_setf(0);
    rf_trxcal_ctrl_hw_setf(0);
    rf_rbb_bw_ctrl_hw_setf(0);
    rf_lna_ctrl_hw_setf(0);
    rf_tx_gain_ctrl_hw_setf(0);
    rf_rx_gain_ctrl_hw_setf(0);
    rf_pu_ctrl_hw_setf(0);

    // configure for lo fcal
    rf_fcal_en_setf(1);
    rf_lo_fbdv_sel_fb_clk_setf(0);
    rf_fcal_div_setf(ncycles);      // configure number cycles of reference clock
    rf_lo_sdmin_setf(fbdv_div_sdm); // configure fbdv divider (0x1000000 = 4.0 when sdm is bypassed)
    rf_lo_sdm_bypass_setf(1);
    rf_lo_sdm_rstb_setf(0);
    rf_lo_fbdv_rst_setf(1);
    wait_us(100);
    rf_lo_sdm_rstb_setf(1);
    rf_lo_fbdv_rst_setf(0);
    wait_us(100);

    // scan vco freq
    for (cw = 0; cw < 256; cw += 8) {
        cnt = _fcal_meas(cw);
        vco_freq = fbdv_div*xtalfreq_MHz*cnt/ncycles;
        printf(prefix "capcode %02lx, cnt %lx, vco freq %.2f MHz\r\n",cw,cnt,vco_freq);
    }

    // build count table
    bw = 8;
    cw = 1 << (bw - 1);
    for (k = 1; k < bw; k++){
        cnt = _fcal_meas(cw);
        
        printf(prefix "binary search: fmeas %lx,result %lx\r\n",cw,cnt);

        if (cnt >= count1 && cnt <= count2) {
            break;
        }
        else if (cnt < count1) {
            cw -= 1 << (bw-1-k);
        }
        else if (cnt > count2) {
            cw += 1 << (bw-1-k);
        }
    }
    cw = (cw + 1 > 255) ? 255 : cw + 1;
    cw_binary = cw;

    if (cw_binary < 64) {
        printf(prefix "fcal binary search error\r\n");
        while (1);
    }

    // build lut
    k = 0; cw = cw_binary + 3;
    for (k = 0,channel_freq = 2404; channel_freq <= 2484; channel_freq += 4, k += 1) {
        vco_freq = (4.0/3.0)*channel_freq;
        target_count = (int)(vco_freq/fbdv_div/xtalfreq_MHz*ncycles);
        cw = cw + 3;
        do {
            cnt_last = cnt;
            cnt = _fcal_meas(--cw);
        } while (cnt < target_count);

        // if (cnt - target_count <= target_count - cnt_last) {
        //     lo_lut[k].vco_freq_cw = cw;
        // }
        // else {
        //     lo_lut[k].vco_freq_cw = cw + 1;
        // }
        lo_lut[k].vco_freq_cw = cw + 1;
        lo_lut[k].sdm_cw = (int)(0.5+4.0/3.0*channel_freq/xtalfreq_MHz*(1<<22));
        lo_lut[k].count  = target_count;
        // lo_lut[k].freq_MHz = channel_freq;

        printf(prefix "channel %ld (%02ld): sdmin %08lx,c_tgt %04lx,c(%lx)=%04lx,c(%lx)=%04lx,vco_freq_cw %02x\r\n",
                channel_freq,k,lo_lut[k].sdm_cw,
                target_count,
                cw+1,cnt_last,
                cw,cnt,
                lo_lut[k].vco_freq_cw);
    }

    // restore registers
    _restore_after_cal();
}

#endif

static void wait_us(uint32_t us)
{
    volatile uint32_t n;
    us = (us >= (1<<24)-1) ? (1<<24)-1 : us;
    // if (us < 2) {
    //     return;
    // }
    // else 
    {
        n = us << 4;
        while(n--);
    }
}

__attribute__((unused)) static void wait_ms(uint32_t ms)
{
    wait_us(1000*ms);
}

uint32_t rfc_read_reg(uint32_t addr)
{
    return *(volatile uint32_t*)addr;
}

void rfc_write_reg(uint32_t addr,uint32_t value)
{
    *(volatile uint32_t*)addr = value;
}

void rfc_write_reg_bits(uint32_t addr, uint8_t high_bit, uint8_t low_bit, uint32_t wdata)
{
    uint32_t rval = 0;
    uint32_t wval = 0;
    uint8_t  bitsize = high_bit - low_bit + 1;
    uint32_t mask_bit = ~(((1 << bitsize) - 1) << low_bit);

    if (low_bit > high_bit) {
        return;
    }

    rval = rfc_read_reg(addr);
    wval = wdata & ((1 << bitsize) - 1);
    wval = (rval & mask_bit) | ( wval << low_bit);

    rfc_write_reg(addr,wval);
}

#ifndef RF_PRIVATE_PRESENT
static void _run_cal(void)
{

    _fcal_main();
}

static void _set_rf(void)
{
    #if 0
    // configure capcode
    rfc_write_reg_bits(0x4000f884,27,22,60);
    rfc_write_reg_bits(0x4000f884,21,16,60);
    #endif

    // enable/disable incremental calibration
    rf_acal_en_setf(0);
    rf_fcal_en_setf(0);
    rf_inc_acal_ctrl_en_hw_setf(0);
    rf_inc_fcal_ctrl_en_hw_setf(0);
    rf_inc_acal_sten_setf(0);
    rf_inc_fcal_sten_setf(0);
    rf_acal_inc_en_setf(0);
    rf_fcal_inc_en_setf(0);

    // disable LO halfstep due to overflow issue
    rf_lo_fbdv_halfstep_en_tx_setf(0);
    rf_lo_fbdv_halfstep_en_rx_setf(0);
    rf_lo_fbdv_halfstep_en_setf(0);
}
#endif

#define SET_RFG(index,rfg) rf_tbb_ind_gc##index##_setf((rfg))
#define SET_DG(index,dg) rf_tx_dvga_gain_qdb_gc##index##_setf((dg))

static void _set_rfc(uint32_t xtalfreq_hz)
{
    uint32_t rf_fsm_lo_time = 52*80; // in 80M cycles
    // uint32_t rf_fsm_dly_30_time = 30*50; // 30us in 50M cycles
    uint32_t lo_center_freq_mhz,lo_sdmin_center,lo_sdmin_1m,lo_sdmin_if;
    uint32_t fcal_cw;
    uint32_t acal_cw;
    uint32_t lo_cw;
    uint32_t rfg_index, dg;

    xtalfreq_MHz = xtalfreq_hz/1000/1000;

    _calc_sdm_params(xtalfreq_hz,&lo_center_freq_mhz,&lo_sdmin_center,&lo_sdmin_1m,&lo_sdmin_if);

    // configure sdm params
    rf_lo_center_freq_mhz_setf(lo_center_freq_mhz);
    rf_lo_sdmin_center_setf(lo_sdmin_center);
    rf_lo_sdmin_1m_setf(lo_sdmin_1m);
    rf_lo_sdmin_if_setf(lo_sdmin_if);

    // configure fcal table
    //   f_cw_2408 , a_cw_2408, f_cw_2404, a_cw_2404
    //   f_cw_2416 , a_cw_2416, f_cw_2412, a_cw_2412
    //   ...
    //   f_cw_2424 , a_cw_2424, f_cw_2420, a_cw_2420
    //   f_cw_2432 , a_cw_2432, f_cw_2428, a_cw_2428
    //   f_cw_2440 , a_cw_2440, f_cw_2436, a_cw_2436
    //   f_cw_2448 , a_cw_2448, f_cw_2444, a_cw_2444
    //   f_cw_2456 , a_cw_2456, f_cw_2452, a_cw_2452
    //   f_cw_2464 , a_cw_2464, f_cw_2460, a_cw_2460
    //   f_cw_2472 , a_cw_2472, f_cw_2468, a_cw_2468
    //   f_cw_2480 , a_cw_2480, f_cw_2476, a_cw_2476
    //                          f_cw_2484, a_cw_2484
    lo_cw = 0;
    for (int k = 0,freq = 2404; freq <= 2484; freq += 4, k += 1) {
        #ifdef RF_PRIVATE_PRESENT
        fcal_cw = rf_pri_get_vco_freq_cw(freq);
        acal_cw = rf_pri_get_vco_idac_cw(freq);
        if (freq == 2404) fcal_cw += 1;
        #else
        fcal_cw = lo_lut[k].vco_freq_cw;
        acal_cw = 6; // lo_lut[k].vco_idac_cw;
        #endif
        
        printf("%d: %02lx | %lx\r\n", freq, fcal_cw,  acal_cw);

        if ((k % 2) == 0) {
            lo_cw = (fcal_cw << 8) | acal_cw;
        }
        else {
            lo_cw = (((fcal_cw << 8) | acal_cw) << 16) | lo_cw;
            *(volatile uint32_t*)(0x4000113c+((k>>1)<<2)) = lo_cw;
            printf("                     %08lx\r\n", lo_cw);
        }

        if (freq == 2484) {
            *(volatile uint32_t*)(0x4000113c+((k>>1)<<2)) = lo_cw;
            printf("                     %08lx\r\n", lo_cw);
        }
    }
    
    // configure power table
    #ifdef RF_PRIVATE_PRESENT
    rf_pri_query_txgain_table( 0,&rfg_index,&dg); SET_RFG( 0,rfg_index); SET_DG( 0,dg);
    rf_pri_query_txgain_table( 1,&rfg_index,&dg); SET_RFG( 1,rfg_index); SET_DG( 1,dg);
    rf_pri_query_txgain_table( 2,&rfg_index,&dg); SET_RFG( 2,rfg_index); SET_DG( 2,dg);
    rf_pri_query_txgain_table( 3,&rfg_index,&dg); SET_RFG( 3,rfg_index); SET_DG( 3,dg);
    rf_pri_query_txgain_table( 4,&rfg_index,&dg); SET_RFG( 4,rfg_index); SET_DG( 4,dg);
    rf_pri_query_txgain_table( 5,&rfg_index,&dg); SET_RFG( 5,rfg_index); SET_DG( 5,dg);
    rf_pri_query_txgain_table( 6,&rfg_index,&dg); SET_RFG( 6,rfg_index); SET_DG( 6,dg);
    rf_pri_query_txgain_table( 7,&rfg_index,&dg); SET_RFG( 7,rfg_index); SET_DG( 7,dg);
    rf_pri_query_txgain_table( 8,&rfg_index,&dg); SET_RFG( 8,rfg_index); SET_DG( 8,dg);
    rf_pri_query_txgain_table( 9,&rfg_index,&dg); SET_RFG( 9,rfg_index); SET_DG( 9,dg);
    rf_pri_query_txgain_table(10,&rfg_index,&dg); SET_RFG(10,rfg_index); SET_DG(10,dg);
    rf_pri_query_txgain_table(11,&rfg_index,&dg); SET_RFG(11,rfg_index); SET_DG(11,dg);
    rf_pri_query_txgain_table(12,&rfg_index,&dg); SET_RFG(12,rfg_index); SET_DG(12,dg);
    rf_pri_query_txgain_table(13,&rfg_index,&dg); SET_RFG(13,rfg_index); SET_DG(13,dg);
    rf_pri_query_txgain_table(14,&rfg_index,&dg); SET_RFG(14,rfg_index); SET_DG(14,dg);
    rf_pri_query_txgain_table(15,&rfg_index,&dg); SET_RFG(15,rfg_index); SET_DG(15,dg);
    #endif

    // configure rf_fsm related params
    rf_rc_state_dbg_en_setf(0);             // turn off debug port
    rf_fsm_st_dbg_en_setf(0);               // turn off debug port
    rf_fsm_lo_time_setf(rf_fsm_lo_time);    // config lo state wait time
    rf_fsm_ctrl_en_setf(1);                 // enable rfc fsm

    // configure tx gain control
    rf_tx_gain_ctrl_hw_setf(1);
    rf_tx_dvga_gain_ctrl_hw_setf(1);
    rf_tx_dvga_gain_qdb_setf(0);

    // configure rf compensation paramas
    // turn off trxcal if ROSDAC table changes vs gain to prevent pkdet false triggers 
    // caused by the transient response of ROSDAC 
    rf_trxcal_ctrl_hw_setf(1); 
    
    // configure rx gain control
    rf_rx_gain_ctrl_hw_setf(1);
}

// static void _set_rfc_channel(void)
// {
// }

static void _set_rfc_powercontrol(uint32_t pc_mode)
{
    rf_pa_ib_fix_setf(0);
    rf_tmx_cs_setf(6);

    if (pc_mode == RFC_PC_AUTO) {
        rf_pa_vbcore_11b_setf(0xf);
        rf_pa_vbcore_11n_setf(0xa);
        rf_pa_vbcore_11g_setf(0xa);

        rf_pa_vbcas_11b_setf(5);
        rf_pa_vbcas_11n_setf(5);
        rf_pa_vbcas_11g_setf(5);
        
        rf_pa_iet_11b_setf(7);
        rf_pa_iet_11n_setf(5);
        rf_pa_iet_11g_setf(5);

        rf_pa_vbcore_setf(0xf);  // register control for BLE 
        rf_pa_vbcas_setf(5);     // register control for BLE
        rf_pa_iet_setf(7);       // register control for BLE 
    }
    else if (pc_mode == RFC_PC_WLAN_11B) {
        rf_pa_vbcore_setf(0xf);
        rf_pa_vbcas_setf(5);
        rf_pa_iet_setf(7);   
    }
    else if (pc_mode == RFC_PC_WLAN_11G) {
        rf_pa_vbcore_setf(0xa);
        rf_pa_vbcas_setf(5);
        rf_pa_iet_setf(5);   
    }
    else if (pc_mode == RFC_PC_WLAN_11N) {
        rf_pa_vbcore_setf(0xa);
        rf_pa_vbcas_setf(5);
        rf_pa_iet_setf(5);
    }
    else if (pc_mode == RFC_PC_BT_BLE) {
        rf_pa_vbcore_setf(0xf);
        rf_pa_vbcas_setf(5);
        rf_pa_iet_setf(7);
    }

    // gain setting
    rf_gc_tbb_boost_setf(0);
    rf_gc_tbb_setf(6);
    rf_gc_tmx_setf(5);
    rf_dac_bias_sel_setf(1);
}

static void _calc_sdm_cw(uint32_t xtalfreq, float freq_MHz, uint32_t *sdm_cw) 
{
    float xtal_MHz = xtalfreq/1000000;
    *sdm_cw = (uint32_t)(0.5+freq_MHz*4/3/xtal_MHz*(1<<22));
}

static void _calc_sdm_params(uint32_t xtalfreq,uint32_t *lo_center_freq_mhz,uint32_t *lo_sdmin_center,uint32_t *lo_sdmin_1m,uint32_t *lo_sdmin_if)
{
    float center_freq_MHz = xtalfreq == 38400000 ? 2448 : 
                            xtalfreq == 40000000 ? 2430 : 2448;
    
    *lo_center_freq_mhz = (uint32_t)center_freq_MHz;
    _calc_sdm_cw(xtalfreq,center_freq_MHz,lo_sdmin_center);
    _calc_sdm_cw(xtalfreq,1.0,lo_sdmin_1m);
    _calc_sdm_cw(xtalfreq,1.25,lo_sdmin_if);
}

void _print_channel_info()
{
    int i;

    printf("Channel information:\r\n");
    printf("    lo_sdmin_hw        %lx\r\n",rf_lo_sdmin_hw_getf());
    printf("    lo_sdmbypass_hw    %ld\r\n",rf_lo_sdm_bypass_hw_getf());
    printf("    lo_vco_idac_cw_hw  %lx\r\n",rf_lo_vco_idac_cw_hw_getf());
    printf("    lo_vco_freq_cw_hw  %lx\r\n",rf_lo_vco_freq_cw_hw_getf());
    printf("    lo_unlocked        %ld\r\n",rf_lo_unlocked_getf());
    printf("    lo_halfstep_en_hw  %ld\r\n",rf_lo_fbdv_halfstep_en_hw_getf());
    printf("    lo_slipped_up      ");
    for (i = 0; i < 8; i++) {
        wait_ms(1);
        printf("%ld", rf_lo_slipped_up_getf() ? (uint32_t)1 : (uint32_t)0);
    }
    printf("\r\n");
    printf("    lo_slipped_dn      ");
    for (i = 0; i < 8; i++) {
        wait_ms(1);
        printf("%ld", rf_lo_slipped_dn_getf() ? (uint32_t)1 : (uint32_t)0);
    }
    printf("\r\n");
}

static void _set_rf_channel_sw(uint32_t channel_freq)
{
    
    uint32_t sdmin = (uint32_t)(channel_freq*4.0/3/xtalfreq_MHz*(1<<22));

    rf_lo_ctrl_hw_setf(0);
    rf_sdm_ctrl_hw_setf(0);
    rf_pu_vco_setf(1);
    rf_pu_fbdv_setf(1);
    rf_pu_pfd_setf(1);
    rf_pu_osmx_setf(1);
    rf_pu_ctrl_hw_setf(0);

    // set channel
    // rf_lo_fbdv_halfstep_en_setf(1);
    #ifdef RF_PRIVATE_PRESENT
    rf_lo_vco_freq_cw_setf(rf_pri_get_vco_freq_cw(channel_freq));
    rf_lo_vco_idac_cw_setf(rf_pri_get_vco_idac_cw(channel_freq));
    rf_lo_osmx_cap_setf(rf_pri_get_vco_freq_cw(channel_freq) >> 4);
    #else
    int32_t k = (int32_t)((channel_freq-2404)/4 + 0.5);
    k = k < 0 ? 0 : k;
    k = k > sizeof(lo_lut)/sizeof(lo_lut[0])-1 ? sizeof(lo_lut)/sizeof(lo_lut[0]) - 1 : k;
    rf_lo_vco_freq_cw_setf(lo_lut[k].vco_freq_cw);
    rf_lo_vco_idac_cw_setf(6);
    rf_lo_osmx_cap_setf(lo_lut[k].vco_freq_cw >> 4);
    #endif

    rf_lo_sdmin_setf(sdmin);
    rf_lo_sdm_bypass_setf(0);

    // wait channel lock
    do {
        rf_lo_fbdv_rst_setf(1);
        wait_us(10);
        rf_lo_fbdv_rst_setf(0);
        wait_us(50);
        rf_lo_pfd_rst_csd_setf(1);
        wait_us(10);
        rf_lo_pfd_rst_csd_setf(0);
        wait_us(50);

        // if (rf_lo_slipped_dn_getf() || rf_lo_slipped_up_getf()) {
        //     printf(".");
        // }
        // else {
        //     printf("\r\n");
        //     break;
        // }
    } while (0);
}

#if 0
static void _set_rf_channel_sw0(uint32_t channel_freq)
{
    int32_t k = (int32_t)((channel_freq-2404)/4 + 0.5);
    uint32_t sdmin = (uint32_t)(channel_freq*4.0/3/xtalfreq_MHz*(1<<22));
    uint32_t vco_freq_cw;
    uint32_t vco_idac_cw;

    k = k < 0 ? 0 : k;
    k = k > sizeof(lo_lut)/sizeof(lo_lut[0])-1 ? sizeof(lo_lut)/sizeof(lo_lut[0]) - 1 : k;

    rf_lo_vco_freq_cw_setf(rf_lo_vco_freq_cw_hw_getf());
    rf_lo_vco_idac_cw_setf(rf_lo_vco_idac_cw_hw_getf());
    rf_lo_fbdv_halfstep_en_setf(1);
    rf_lo_sdmin_setf(rf_lo_sdmin_hw_getf());
    rf_lo_sdm_bypass_setf(0);

    rf_lo_ctrl_hw_setf(0);

    // wait channel lock
    do {
        rf_lo_fbdv_rst_setf(1);
        wait_us(10);
        rf_lo_fbdv_rst_setf(0);
        wait_us(50);
        rf_lo_pfd_rst_csd_setf(1);
        wait_us(10);
        rf_lo_pfd_rst_csd_setf(0);
        wait_us(50);
    } while (0);
}

static void _set_rf_channel_hw(uint32_t channel_freq)
{
    rf_ch_ind_wifi_setf(channel_freq);

    // clear LO ready
    rf_fsm_lo_rdy_rst_setf(1); wait_us(100);
    rf_fsm_lo_rdy_rst_setf(0); wait_us(10);

    // toggle rf_fsm_ctrl to reset rf_fsm
    rf_fsm_ctrl_en_setf(0); wait_us(10);
    rf_fsm_ctrl_en_setf(1); wait_us(10);

    // set rf_fsm state using debug port
	rf_fsm_st_dbg_setf(RFC_FSM_SB); wait_us(10);
	rf_fsm_st_dbg_en_setf(1);       wait_us(10);
    
    #if 0
    printf("pu_fbdv %ld,pu_vco %ld,pu_pfd %ld,pu_osmx %ld\r\n",
        rf_pu_fbdv_hw_getf(),rf_pu_vco_hw_getf(),rf_pu_pfd_hw_getf(),rf_pu_osmx_getf());
    printf("sdmin %08lx\r\n",rf_lo_sdmin_hw_getf());
    rf_lo_ctrl_hw_setf(0);  wait_us(10);
    rf_lo_sdmin_setf(0x14599998); wait_us(100);
    rf_lo_fbdv_rst_setf(1); wait_us(100);
    rf_lo_fbdv_rst_setf(0); wait_us(10);
    rf_lo_ctrl_hw_setf(1);  wait_us(10);
    #endif 

    /* Controlled by HW */
    rf_fsm_st_dbg_setf(RFC_FSM_LO); wait_us(300);
    printf("*************************** (1) *****************\r\n");
    _print_channel_info();

    /* Controlled by SW */
    rf_lo_vco_freq_cw_setf(rf_lo_vco_freq_cw_hw_getf());
    rf_lo_vco_idac_cw_setf(rf_lo_vco_idac_cw_hw_getf());
    rf_lo_sdmin_setf(rf_lo_sdmin_hw_getf());
    rf_lo_sdm_bypass_setf(rf_lo_sdm_bypass_hw_getf());
    rf_lo_fbdv_halfstep_en_setf(rf_lo_fbdv_halfstep_en_hw_getf());

    rf_lo_ctrl_hw_setf(0);  wait_us(100);

    // rf_lo_pfd_rst_csd_setf(1);
    // rf_lo_fbdv_rst_setf(1); wait_us(100);
    // rf_lo_fbdv_rst_setf(0); wait_us(10);
    // wait_us(100);
    // rf_lo_pfd_rst_csd_setf(0);
    // wait_us(100);

    rf_lo_pfd_rst_csd_setf(1);
    rf_lo_fbdv_rst_setf(1);
    rf_lo_fbdv_rst_setf(0);
    wait_us(100);
    rf_lo_pfd_rst_csd_setf(0);
    wait_us(100);

    printf("*************************** (2) *****************\r\n");
    _print_channel_info();

    rf_lo_ctrl_hw_setf(1);  wait_us(10);

// // _set_rf_channel_sw0(channel_freq);
//     printf("*************************** (2-B) *****************\r\n");
//     _print_channel_info();

    rf_fsm_st_dbg_setf(RFC_FSM_SB); wait_us(300);
    printf("*************************** (3) *****************\r\n");
    _print_channel_info();
    rf_lo_fbdv_halfstep_en_tx_setf(0);
    rf_lo_fbdv_halfstep_en_rx_setf(0);
    // // // // rf_lo_ctrl_hw_setf(0);  wait_us(10);
    // // // // rf_lo_fbdv_halfstep_en_setf(0);
    // // // // rf_lo_fbdv_halfstep_en_tx_setf(0);
    // // // // rf_lo_fbdv_halfstep_en_rx_setf(0);
    // // // // rf_lo_ctrl_hw_setf(1);  wait_us(10);
    
    rf_fsm_st_dbg_setf(RFC_FSM_LO); wait_us(300);
    printf("*************************** (4) *****************\r\n");
    _print_channel_info();

    // // now debug port is released
    // rf_fsm_st_dbg_en_setf(0);        wait_us(100);

    // wait_us(100);
    rf_lo_ctrl_hw_setf(0);
    rf_lo_fbdv_rst_setf(1);
    rf_lo_fbdv_rst_setf(0);
    rf_lo_ctrl_hw_setf(1);

    rf_lo_ctrl_hw_setf(0);
    rf_lo_pfd_rst_csd_setf(1); wait_us(100);
    rf_lo_pfd_rst_csd_setf(0);
    rf_lo_ctrl_hw_setf(1);

    printf("*************************** (5) *****************\r\n");
    _print_channel_info();

    printf("\r\n\r\n");
}
#endif

static void _set_rf_channel_hw(uint32_t channel_freq)
{
    rf_ch_ind_wifi_setf(channel_freq);

    // clear LO ready
    rf_fsm_lo_rdy_rst_setf(1); wait_us(100);
    rf_fsm_lo_rdy_rst_setf(0); wait_us(10);

    // toggle rf_fsm_ctrl to reset rf_fsm
    rf_fsm_ctrl_en_setf(0); wait_us(10);
    rf_fsm_ctrl_en_setf(1); wait_us(10);

    // set rf_fsm state using debug port
	rf_fsm_st_dbg_setf(RFC_FSM_SB); wait_us(10);
	rf_fsm_st_dbg_en_setf(1);       wait_us(10);

    // set LO state
    rf_fsm_st_dbg_setf(RFC_FSM_LO); wait_us(300);

    // now debug port is released
    rf_fsm_st_dbg_en_setf(0); wait_us(100);
}

void rfc_config_channel(uint32_t channel_freq)
{
    uint8_t ncf_on;
    uint8_t ncf_alpha = 1;
    int32_t  ncf_freq_hz;
    int8_t  ncf_nrmfc = 0;
    double  ncf_fs_hz = 40 * 1000 * 1000;
    
    rfif_int_lo_unlocked_mask_setf(1);

    rf_lo_ctrl_hw_setf(1);
    rf_sdm_ctrl_hw_setf(1);
    rf_pu_ctrl_hw_setf(1);
    _set_rf_channel_hw(channel_freq);
    _print_channel_info();

    #ifdef RF_PRIVATE_PRESENT
    rf_pri_update_param(channel_freq);
    rf_pri_get_notch_param(channel_freq,&ncf_on,&ncf_freq_hz);
    ncf_nrmfc = (int8_t)((ncf_freq_hz/ncf_fs_hz)*(1<<8) + 0.5);
    rfc_rxdfe_set_notch0(ncf_on,ncf_alpha,ncf_nrmfc);
    #endif

    rfif_int_lo_unlocked_mask_setf(0);
}

void rfc_config_channel_sw(uint32_t channel_freq)
{
    uint8_t ncf_on;
    uint8_t ncf_alpha = 1;
    int32_t  ncf_freq_hz;
    int8_t  ncf_nrmfc = 0;
    double  ncf_fs_hz = 40 * 1000 * 1000;

    rf_lo_ctrl_hw_setf(0);
    rf_sdm_ctrl_hw_setf(0);
    rf_pu_ctrl_hw_setf(0);
    _set_rf_channel_sw(channel_freq);
    _print_channel_info();

    #ifdef RF_PRIVATE_PRESENT
    rf_pri_update_param(channel_freq);
    rf_pri_get_notch_param(channel_freq,&ncf_on,&ncf_freq_hz);

    ncf_nrmfc = (int8_t)((ncf_freq_hz/ncf_fs_hz)*(1<<8) + 0.5);
    rfc_rxdfe_set_notch0(ncf_on,ncf_alpha,ncf_nrmfc);
    #endif
}

#define READ_DG(index) (((int8_t)(rf_tx_dvga_gain_qdb_gc##index##_getf() << 1)) >> 1)

void _check_config()
{
    printf("txvector powerlevel[:2] table\r\n");
    printf("  0:  %ld, %d\r\n", rf_tbb_ind_gc0_getf(),READ_DG(0));
    printf("  1:  %ld, %d\r\n", rf_tbb_ind_gc1_getf(),READ_DG(1));
    printf("  2:  %ld, %d\r\n", rf_tbb_ind_gc2_getf(),READ_DG(2));
    printf("  3:  %ld, %d\r\n", rf_tbb_ind_gc3_getf(),READ_DG(3));
    printf("  4:  %ld, %d\r\n", rf_tbb_ind_gc4_getf(),READ_DG(4));
    printf("  5:  %ld, %d\r\n", rf_tbb_ind_gc5_getf(),READ_DG(5));
    printf("  6:  %ld, %d\r\n", rf_tbb_ind_gc6_getf(),READ_DG(6));
    printf("  7:  %ld, %d\r\n", rf_tbb_ind_gc7_getf(),READ_DG(7));
    printf("  8:  %ld, %d\r\n", rf_tbb_ind_gc8_getf(),READ_DG(8));
    printf("  9:  %ld, %d\r\n", rf_tbb_ind_gc9_getf(),READ_DG(9));
    printf("  a:  %ld, %d\r\n", rf_tbb_ind_gc10_getf(),READ_DG(10));
    printf("  b:  %ld, %d\r\n", rf_tbb_ind_gc11_getf(),READ_DG(11));
    printf("  c:  %ld, %d\r\n", rf_tbb_ind_gc12_getf(),READ_DG(12));
    printf("  d:  %ld, %d\r\n", rf_tbb_ind_gc13_getf(),READ_DG(13));
    printf("  e:  %ld, %d\r\n", rf_tbb_ind_gc14_getf(),READ_DG(14));
    printf("  f:  %ld, %d\r\n", rf_tbb_ind_gc15_getf(),READ_DG(15));

    printf("rf gain table\r\n");
    printf("  0:  dac %ld, tbbb %ld, tbb %ld, tmx %ld\r\n", rf_gain_ctrl0_dac_bias_sel_getf(),rf_gain_ctrl0_gc_tbb_boost_getf(),rf_gain_ctrl0_gc_tbb_getf(),rf_gain_ctrl0_gc_tmx_getf());
    printf("  1:  dac %ld, tbbb %ld, tbb %ld, tmx %ld\r\n", rf_gain_ctrl1_dac_bias_sel_getf(),rf_gain_ctrl1_gc_tbb_boost_getf(),rf_gain_ctrl1_gc_tbb_getf(),rf_gain_ctrl1_gc_tmx_getf());
    printf("  2:  dac %ld, tbbb %ld, tbb %ld, tmx %ld\r\n", rf_gain_ctrl2_dac_bias_sel_getf(),rf_gain_ctrl2_gc_tbb_boost_getf(),rf_gain_ctrl2_gc_tbb_getf(),rf_gain_ctrl2_gc_tmx_getf());
    printf("  3:  dac %ld, tbbb %ld, tbb %ld, tmx %ld\r\n", rf_gain_ctrl3_dac_bias_sel_getf(),rf_gain_ctrl3_gc_tbb_boost_getf(),rf_gain_ctrl3_gc_tbb_getf(),rf_gain_ctrl3_gc_tmx_getf());
    printf("  4:  dac %ld, tbbb %ld, tbb %ld, tmx %ld\r\n", rf_gain_ctrl4_dac_bias_sel_getf(),rf_gain_ctrl4_gc_tbb_boost_getf(),rf_gain_ctrl4_gc_tbb_getf(),rf_gain_ctrl4_gc_tmx_getf());
    printf("  5:  dac %ld, tbbb %ld, tbb %ld, tmx %ld\r\n", rf_gain_ctrl5_dac_bias_sel_getf(),rf_gain_ctrl5_gc_tbb_boost_getf(),rf_gain_ctrl5_gc_tbb_getf(),rf_gain_ctrl5_gc_tmx_getf());
    printf("  6:  dac %ld, tbbb %ld, tbb %ld, tmx %ld\r\n", rf_gain_ctrl6_dac_bias_sel_getf(),rf_gain_ctrl6_gc_tbb_boost_getf(),rf_gain_ctrl6_gc_tbb_getf(),rf_gain_ctrl6_gc_tmx_getf());
    printf("  7:  dac %ld, tbbb %ld, tbb %ld, tmx %ld\r\n", rf_gain_ctrl7_dac_bias_sel_getf(),rf_gain_ctrl7_gc_tbb_boost_getf(),rf_gain_ctrl7_gc_tbb_getf(),rf_gain_ctrl7_gc_tmx_getf());

    printf("tx_gain_ctrl_hw : %ld\r\n",rf_tx_gain_ctrl_hw_getf());
    printf("rx_gain_ctrl_hw : %ld\r\n",rf_rx_gain_ctrl_hw_getf());
    printf("trxcal_ctrl_hw  : %ld\r\n",rf_trxcal_ctrl_hw_getf());
    printf("lo_ctrl_hw      : %ld\r\n",rf_lo_ctrl_hw_getf());
    printf("lna_ctrl_hw     : %ld\r\n",rf_lna_ctrl_hw_getf());
    printf("pu_ctrl_hw      : %ld\r\n",rf_pu_ctrl_hw_getf());

    printf("pa setting\r\n");
    printf("   11b: vbcas %ld vbcore %ld iet %ld\r\n",rf_pa_vbcas_11b_getf(),rf_pa_vbcore_11b_getf(),rf_pa_iet_11b_getf());
    printf("   11g: vbcas %ld vbcore %ld iet %ld\r\n",rf_pa_vbcas_11g_getf(),rf_pa_vbcore_11g_getf(),rf_pa_iet_11g_getf());
    printf("   11n: vbcas %ld vbcore %ld iet %ld\r\n",rf_pa_vbcas_11n_getf(),rf_pa_vbcore_11n_getf(),rf_pa_iet_11n_getf());
    printf("   half_on_wifi %ld\r\n",rf_pa_half_on_wifi_getf());
    printf("   half_on_ble %ld\r\n",rf_pa_half_on_getf());
}

void rfc_init(uint32_t xtalfreq_hz)
{
    rf_bbmode_4s_setf(0);
    rf_bbmode_4s_en_setf(1);

    /* Init RF core */
#ifdef RF_PRIVATE_PRESENT
    switch (xtalfreq_hz)
    {
    case 24 * 1000 * 1000:
        rf_pri_xtalfreq(E_RF_XTAL_24M);
        break;
    case 26 * 1000 * 1000:
        rf_pri_xtalfreq(E_RF_XTAL_26M);
        break;
    case 32 * 1000 * 1000:
        rf_pri_xtalfreq(E_RF_XTAL_32M);
        break;
    case 384 * 100 * 1000:
        rf_pri_xtalfreq(E_RF_XTAL_38M4);
        break;
    case 40 * 1000 * 1000:
        rf_pri_xtalfreq(E_RF_XTAL_40M);
        break;
    case 52 * 1000 * 1000:
        rf_pri_xtalfreq(E_RF_XTAL_52M);
        break;
    default:
        rf_pri_xtalfreq(E_RF_XTAL_40M);
        break;
    }
    
    rf_pri_init(!inited);
    inited = 1;
#else
    if (inited) {
        return;
    }
    
    _set_rf();
    _run_cal();
    inited = 1;
#endif
    
    /* Init RF controller */
    _set_rfc(xtalfreq_hz);
    
    /* Release RF force */
    rf_bbmode_4s_en_setf(0);

    /* RF Optimized Settings */
    if (1) {
        rf_adc_gt_rm_setf(1);
        rf_rx_pf_i_en_setf(0);
        rf_rx_pf_q_en_setf(0);
    }

    /* Enable RF incremental cal */
#if CFG_RF_ICAL
    rf_fcal_inc_en_setf(1);
    rf_acal_inc_en_setf(1);
    rf_roscal_inc_en_setf(1);
    rf_fsm_t2r_cal_mode_setf(1);
#else
    rf_fcal_inc_en_setf(0);
    rf_acal_inc_en_setf(0);
    rf_roscal_inc_en_setf(0);
    rf_fsm_t2r_cal_mode_setf(0);
#endif

    _check_config();
    printf("rf controller init done\r\n");
}

void rfc_reset()
{
    inited = 0;
    rfc_init(xtalfreq_MHz * 1000 * 1000);
}


/*
 * TXDFE IMPLEMENTATIONS
 ****************************************************************************************
 */
void rfc_txdfe_start()
{
    rf_tx_dfe_en_4s_setf(1);
    rf_tx_dfe_en_4s_en_setf(1);
}

void rfc_txdfe_stop()
{
    rf_tx_dfe_en_4s_setf(0);
	rf_tx_dfe_en_4s_en_setf(0);
}

void rfc_txdfe_mux(int8_t signal_source)
{
    rf_tx_test_sel_setf(signal_source);
}

void rfc_txdfe_set_dvga(int8_t dvga_qdb)
{
    if (dvga_qdb < -48 || dvga_qdb > +24) {
        printf(prefix "dvga_qdb out of range -48~+24,skip\r\n");
        return;
    }
    rf_tx_dvga_gain_qdb_setf(dvga_qdb);
}

void rfc_txdfe_set_iqgaincomp(uint8_t en,uint16_t coeff)
{
    rf_tx_iqc_gain_en_setf(en);
	rf_tx_iqc_gain_setf(coeff);
}

void rfc_txdfe_set_iqphasecomp(uint8_t en,int16_t coeff)
{
	rf_tx_iqc_phase_en_setf(en);
	rf_tx_iqc_phase_setf(coeff);
}


void rfc_txdfe_set_dccomp(int16_t dcc_i,int16_t dcc_q)
{
    rf_tx_dac_os_i_setf(dcc_i);
    rf_tx_dac_os_q_setf(dcc_q);
}

void rfc_txdfe_set_iqswap(uint8_t swap_on)
{
    rf_tx_dac_iq_swap_setf(swap_on);
}


/*
 * RXDFE IMPLEMENTATIONS
 ****************************************************************************************
 */
void rfc_rxdfe_start()
{
    rf_rx_dfe_en_4s_setf(1);
    rf_rx_dfe_en_4s_en_setf(1);
}

void rfc_rxdfe_stop()
{
    rf_rx_dfe_en_4s_setf(0);
	rf_rx_dfe_en_4s_en_setf(0);
}

void rfc_rxdfe_set_iqgaincomp(uint8_t en,uint16_t coeff)
{
    rf_rx_iqc_gain_en_setf(en);
	rf_rx_iqc_gain_setf(coeff);
}

void rfc_rxdfe_set_iqphasecomp(uint8_t en,int16_t coeff)
{
	rf_rx_iqc_phase_en_setf(en);
	rf_rx_iqc_phase_setf(coeff);
}

void rfc_rxdfe_set_dccomp(int16_t dcc_i,int16_t dcc_q)
{
    rf_rx_adc_os_i_setf(dcc_i);
    rf_rx_adc_os_q_setf(dcc_q);
}

void rfc_rxdfe_set_iqswap(uint8_t swap_on)
{
    rf_rx_adc_iq_swap_setf(swap_on);
}

void rfc_rxdfe_set_notch0(uint8_t en,uint8_t alpha,int8_t nrmfc)
{
    rf_rx_notch0_alpha_setf(alpha);
    rf_rx_notch0_nrmfc_setf(nrmfc);
    rf_rx_notch0_en_setf(en);
}

void rfc_rxdfe_set_notch1(uint8_t en,uint8_t alpha,int8_t nrmfc)
{
    rf_rx_notch1_alpha_setf(alpha);
    rf_rx_notch1_nrmfc_setf(nrmfc);
    rf_rx_notch1_en_setf(en);
}


/*
 * SG IMPLEMENTATIONS
 ****************************************************************************************
 */
void rfc_sg_start(uint32_t inc_step,uint32_t gain_i,uint32_t gain_q,uint32_t addr_i,uint32_t addr_q)
{
    rf_singen_en_setf(0);
	rf_singen_inc_step0_setf(inc_step);
	rf_singen_clkdiv_n_setf(0); // work clock
	rf_singen_mode_i_setf(RFC_SG_SINGLE_TONE); // 0: single tone, 1: two tone, 2: ramp
	rf_singen_mode_q_setf(RFC_SG_SINGLE_TONE);
	rf_singen_gain_i_setf(gain_i);
	rf_singen_gain_q_setf(gain_q);
    // if (complex) {
    // rf_singen_start_addr0_i_setf(0);
    // rf_singen_start_addr0_q_setf(768);
    // }
    // else {
    // rf_singen_start_addr0_i_setf(0);
    // rf_singen_start_addr0_q_setf(0);
    // }
    rf_singen_start_addr0_i_setf(addr_i);
    rf_singen_start_addr0_q_setf(addr_q);
	rf_singen_en_setf(1);
}

/**
 * This function only stops signal generator. TxDFE mux and RF state control are left for 
 * the code using this function. 
 */
void rfc_sg_stop()
{
    // ~~restore txdfe mux to default path (baseband)~~
    // stop signal generator
    rf_singen_en_setf(0);
}

/*
 * PM IMPLEMENTATIONS
 ****************************************************************************************
 */

uint32_t rfc_pm_start(uint32_t insel,int32_t freq_cw,uint32_t acclen,uint32_t rshift,
                     int32_t *raw_acc_i,int32_t *raw_acc_q)
{
    int32_t freqshift_en;
    int32_t freqshift_cw;
    int32_t iqacc_i;
    int32_t iqacc_q;
    uint64_t pwr64;
    uint32_t pwr_hi,pwr_lo;

    // int32_t  adc_rate;
    // adc_rate = insel == PM_ADC ? 80 : insel == PM_IQCOMP ? 40 : 16;
    // if (rf_reg->dfe_ctrl_2.BF.rx_adc_low_pow_en) {
    //     adc_rate = adc_rate >> 1;
    // }
    // freqshift_en = freq_MHz != 0;
    // freqshift_cw = (int32_t)(2*freq_MHz/adc_rate*(1<<19)+0.5);

    // turn off and configure power meter
    rf_rx_pm_en_setf(0);

    // configure freqshift
    freqshift_en = freq_cw != 0;
    freqshift_cw = freq_cw;

    // make sure freq is aligned to singen
    // if (freqshift_en) {
    //     if (insel == RFC_PM_MUX_TO_IQCOMP || rf_rx_adc_low_pow_en_getf()) {
    //         freqshift_cw <<= 1;
    //     }
    // }

    rf_rx_pm_start_ofs_setf(1024);
    rf_rx_pm_acc_len_setf(acclen);
    rf_rx_pm_freqshift_en_setf(freqshift_en);
    rf_rx_pm_freqshift_cw_setf(freqshift_cw);
    rf_rx_pm_in_sel_setf(insel);
    rf_rx_pm_en_setf(1);

    wait_us(100);

    iqacc_i = rf_rx_pm_iqacc_i_getf(); // 25BIT
    iqacc_q = rf_rx_pm_iqacc_q_getf(); // 25BIT
    iqacc_i = iqacc_i << 7 >> 7;
    iqacc_q = iqacc_q << 7 >> 7;

    if (raw_acc_i) {
        *raw_acc_i = iqacc_i;
    }
    if (raw_acc_q) {
        *raw_acc_q = iqacc_q;
    }

    iqacc_i = iqacc_i >> rshift;
    iqacc_q = iqacc_q >> rshift;

    pwr64 = (int64_t)iqacc_i*(int64_t)iqacc_i + (int64_t)iqacc_q*(int64_t)iqacc_q;
    pwr_hi = (uint32_t)(pwr64 >> 32);
    pwr_lo = (uint32_t)(pwr64 & 0xffffffff);

    if (pwr_hi != 0) {
        printf(prefix "overflow occurred\r\n");
    }
    
    return pwr_lo;
}

void rfc_pm_stop()
{
    rf_rx_pm_en_setf(0);
	rf_rx_pm_freqshift_en_setf(0);
}

/*
 * HWCTRL DECLARATIONS
 ****************************************************************************************
 */
void rfc_hwctrl_txrfgain(uint8_t hwctrl_on)
{
    rf_tx_gain_ctrl_hw_setf(hwctrl_on);
}

void rfc_hwctrl_rxgain(uint8_t hwctrl_on)
{
    rf_rx_gain_ctrl_hw_setf(hwctrl_on);
}

void rfc_hwctrl_txdvga(uint8_t hwctrl_on)
{
    rf_tx_dvga_gain_ctrl_hw_setf(hwctrl_on);
}

void rfc_hwctrl_calparam(uint8_t hwctrl_on)
{
    rf_trxcal_ctrl_hw_setf(hwctrl_on);
}

void rfc_fsm_force(uint8_t state)
{
    if (state == RFC_FSM_FORCE_OFF) {
        rf_fsm_st_dbg_en_setf(0);
    }
    else {
        rf_fsm_st_dbg_setf(state); wait_us(20);
        rf_fsm_st_dbg_en_setf(1);
    }
}

void rfc_coex_force_to(uint32_t force_enable, uint32_t bbmode)
{
    // reset rf_fsm, fpga_fsm, state_cci
    // fpga_rf_fsm_en_setf(0);
    rf_fsm_ctrl_en_setf(0);

    wait_us(10);

    // force coex
    rf_bbmode_4s_setf(bbmode);
    rf_bbmode_4s_en_setf(force_enable ? 1 : 0);

    wait_us(10);
    
    // start fsm
    // fpga_rf_fsm_en_setf(1);
    rf_fsm_ctrl_en_setf(1);
}

void rfc_config_power(uint32_t mode,uint32_t tbb_boost,uint32_t tbb,uint32_t tmx)
{
    _set_rfc_powercontrol(mode);
    rf_gc_tbb_boost_setf(tbb_boost);
    rf_gc_tbb_setf(tbb);
    rf_gc_tmx_setf(tmx);
}

void rfc_config_bandwidth(uint32_t mode)
{
    if (mode == RFC_BW_20M) {
        // dac
        rfc_write_reg_bits(0x4000e41c,9,9,0); // clkpll_en_div2_480m

        // adc
        rf_adc_clk_div_sel_setf(1);
        rfckg_rxclk_div2_mode_setf(0);

        // rbb bw
        rf_rbb_bw_setf(2);

        // rmxgm
        rf_rmxgm_10m_mode_en_setf(0);
    }
    else if (mode == RFC_BW_10M) {
        // dac
        rfc_write_reg_bits(0x4000e41c,9,9,1); // clkpll_en_div2_480m

        // adc
        rf_adc_clk_div_sel_setf(0);
        rfckg_rxclk_div2_mode_setf(1);
        
        // rbb bw
        rf_rbb_bw_setf(1);

        // rmxgm
        rf_rmxgm_10m_mode_en_setf(1);
    }
}

uint32_t rfc_get_power_level(uint32_t formatmod, int32_t power)
{
    uint32_t power_level;

    #ifdef RF_PRIVATE_PRESENT
    if (formatmod == RFC_FORMATMOD_11B) {
        power_level = rf_pri_get_txgain_index(power,E_RF_MODE_11B);
    }
    else if (formatmod == RFC_FORMATMOD_11G) {
        power_level = rf_pri_get_txgain_index(power,E_RF_MODE_11G);
    }
    else {
        power_level = rf_pri_get_txgain_index(power,E_RF_MODE_11N);
    }
    #else
    return 0;
    #endif

    return (power_level << 2);
}

struct rfc_test_tag {
    /** test read 1 */
    uint32_t pkdet_out_raw            : 1;  //[0]
    uint32_t dig_xtal_clk_dbg         : 1;  //[1]
    uint32_t clk_ble_16m_dbg          : 1;  //[2]
    uint32_t clk_rc_dbg0              : 1;  //[3]
    uint32_t clk_adcpow_dbg           : 1;  //[4]
    uint32_t clk_fetx_dbg             : 1;  //[5]
    uint32_t clk_ferx_dbg             : 1;  //[6]
    uint32_t clkpll_postdiv_outclk_dbg: 1;  //[7]
    uint32_t clk_soc_480m_dbg         : 1;  //[8]
    uint32_t clk_soc_240m_dbg         : 1;  //[9]
    uint32_t clk_soc_192m_dbg         : 1;  //[10]
    uint32_t clk_soc_160m_dbg         : 1;  //[11]
    uint32_t clk_soc_120m_dbg         : 1;  //[12]
    uint32_t clk_soc_96m_dbg          : 1;  //[13]
    uint32_t clk_soc_80m_dbg          : 1;  //[14]
    uint32_t clk_soc_48m_dbg          : 1;  //[15]
    uint32_t clk_soc_32m_dbg          : 1;  //[16]
    uint32_t pad_pkdet_out            : 1;  //[17]
    uint32_t pad_agc_ctrl             :10;  //[27:18]
    uint32_t rf_pkdet_rst_hw          : 1;  //[28]
    uint32_t rf_cbw_wifi              : 2;  //[30:29]
    uint32_t lo_unlocked              : 1;  //[31]

    /** test read 2 */
    uint32_t fsm_pu_txbuf             : 1;  //[0]
    uint32_t fsm_pu_rxbuf             : 1;  //[1]
    uint32_t fsm_pu_tosdac            : 1;  //[2]
    uint32_t fsm_pu_dac               : 1;  //[3]
    uint32_t fsm_trsw_en              : 1;  //[4]
    uint32_t fsm_pu_adc               : 1;  //[5]
    uint32_t fsm_pu_pkdet             : 1;  //[6]
    uint32_t fsm_pu_rbb               : 1;  //[7]
    uint32_t fsm_pu_rmx               : 1;  //[8]
    uint32_t fsm_pu_rmxgm             : 1;  //[9]
    uint32_t fsm_pu_lna               : 1;  //[10]
    uint32_t clk_rc_dbg2              : 1;  //[11]
    uint32_t rf_lna_ind_hw            : 4;  //[15:12]
    uint32_t rf_rbb_ind_hw            : 4;  //[19:16]
    uint32_t rf_tx_pow_lvl_hw         : 4;  //[23:20]
    uint32_t rf_rc_lo_rdy             : 1;  //[24]
    uint32_t rf_fsm_state             : 3;  //[27:25]
    uint32_t rf_rc_state              : 3;  //[30:28]
    uint32_t clk_rc_dbg               : 1;  //[31]

};


void rfc_dump()
{
    struct rfc_test_tag *p_test_read;
    uint32_t test_read[2];
    static char* rc_state_str[8] = {
        "RC_IDLE",
        "RC_RX2ON",
        "RC_TX2ON",
        "RC_RXON",
        "RC_TX2PAON",
        "RC_TXPAON",
        "UNKNOWN",
        "UNKNOWN"
    };
    static char* rf_state_str[8] = {
        "RF_PD",
        "RF_SB",
        "RF_LO",
        "RF_RX",
        "RF_TX",
        "RF_T2RI",
        "RF_R2T",
        "RF_MS"
    };


    rf_test_sel_setf(1);
    test_read[0] = rf_test_read_getf();

    rf_test_sel_setf(2);
    test_read[1] = rf_test_read_getf();

    p_test_read = (struct rfc_test_tag*)test_read;

    printf("******************************** [RFC DUMP START] *****************************\r\n");
    printf("  rc_state      : %s\r\n", rc_state_str[p_test_read->rf_rc_state]);
    printf("  rf_state      : %s\r\n", rf_state_str[p_test_read->rf_fsm_state]);
    printf("  fsm_pu_lna    : %d\r\n", p_test_read->fsm_pu_lna   );
    printf("  fsm_pu_rmxgm  : %d\r\n", p_test_read->fsm_pu_rmxgm );
    printf("  fsm_pu_rmx    : %d\r\n", p_test_read->fsm_pu_rmx   );
    printf("  fsm_pu_rbb    : %d\r\n", p_test_read->fsm_pu_rbb   );
    printf("  fsm_pu_pkdet  : %d\r\n", p_test_read->fsm_pu_pkdet );
    printf("  fsm_pu_adc    : %d\r\n", p_test_read->fsm_pu_adc   );
    printf("  fsm_trsw_en   : %d\r\n", p_test_read->fsm_trsw_en  );
    printf("  fsm_pu_dac    : %d\r\n", p_test_read->fsm_pu_dac   );
    printf("  fsm_pu_tosdac : %d\r\n", p_test_read->fsm_pu_tosdac);
    printf("  fsm_pu_rxbuf  : %d\r\n", p_test_read->fsm_pu_rxbuf );
    printf("  fsm_pu_txbuf  : %d\r\n", p_test_read->fsm_pu_txbuf );
    
    printf("\r\n\r\n");
    printf("  tx_gain_ctrl_hw : %ld\r\n", rf_tx_gain_ctrl_hw_getf());
    printf("  rx_gain_ctrl_hw : %ld\r\n", rf_rx_gain_ctrl_hw_getf());
    printf("  trxcal_ctrl_hw  : %ld\r\n", rf_trxcal_ctrl_hw_getf());
    printf("  lo_ctrl_hw      : %ld\r\n", rf_lo_ctrl_hw_getf());
    printf("  lna_ctrl_hw     : %ld\r\n", rf_lna_ctrl_hw_getf());
    printf("  pu_ctrl_hw      : %ld\r\n", rf_pu_ctrl_hw_getf());

    printf("\r\n\r\n");
    printf("  rf_lo_vco_freq_cw_hw : %ld\r\n", rf_lo_vco_freq_cw_hw_getf());
    printf("  rf_lo_vco_idac_cw_hw : %ld\r\n", rf_lo_vco_idac_cw_hw_getf());
    printf("  rf_lo_sdmin_hw       : %ld\r\n", rf_lo_sdmin_hw_getf());
    printf("  rf_ch_ind_wifi       : %ld\r\n", rf_ch_ind_wifi_getf());


    if (rf_lo_unlocked_getf()) {
        printf("rf_lo_unlocked_getf = 1\r\n");
        printf("rf_lo_unlocked_getf (after csd reset) = ");
        for (int i = 0; i < 8; i++) {
            rf_lo_pfd_rst_csd_setf(1);
            wait_us(10);
            rf_lo_pfd_rst_csd_setf(0);
            wait_us(10);
            printf("%ld", rf_lo_unlocked_getf());
        }
        printf("\r\n");
    }
    else {
        printf("rf_lo_unlocked_getf = 0\r\n");
    }

    printf("******************************** [RFC DUMP END]   *****************************\r\n");


}
