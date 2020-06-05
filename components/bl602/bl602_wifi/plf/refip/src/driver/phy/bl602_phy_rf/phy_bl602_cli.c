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
#include <string.h>
#include <stdint.h>

#include <cli.h>
#include "FreeRTOS.h"
#include "task.h"
#include "bl_config.h"

#include "reg_macbypass.h"
#include "reg_mdm_stat.h"
#include "reg_mdm_cfg.h"
#include "reg_mdmdsss_cfg.h"
#include "reg_riu.h"
#include "reg/reg_bl_rc2.h"
#include "reg/reg_rf.h"
#include "reg/reg_rf_fpga.h"
#include "reg/reg_rf_fsm.h"
#include "reg/reg_rf_epa.h"

#include "phy.h"
#include "rfc_bl602.h"
#include "bl602_rf_private.h"
#include "bl602_rf_calib_data.h"

#include "phy_hal.h"

#define PRINT_PREFIX  "[phy_cli]"
#define RESULT_PREFIX "done "

enum {
    MACBYP_OFF = 0,
    MACBYP_ON = 1
};

typedef enum {
    MACBYP_IDLE = 0,
    MACBYP_RX   = 1,
    MACBYP_TX_SINGLE = 2,
    MACBYP_TX_BURST = 3,
    MACBYP_TX_CONT = 4
} MacbypassMode_t;

typedef enum {
    MACBYP_PLD_PRBS = 0,
    MACBYP_PLD_ALLONE = 1,
    MACBYP_PLD_RAMP = 2,
    MACBYP_PLD_01 = 3,
    MACBYP_PLD_10 = 4
} MacbypassPldType_t;

typedef enum {
    NONHT = 0,
    NONHT_DUP = 1,
    HT_MM = 2,
    HT_GF = 3
} FormatMode_t;

typedef enum {
    LM11B_MCS1 = 0x0,
    LM11B_MCS2 = 0x1,
    LM11B_MCS3 = 0x2,
    LM11B_MCS4 = 0x3,
    LM_MCS0    = 0xb, // LEG_6=0xb,
    LM_MCS1    = 0xf, // LEG_9=0xf,
    LM_MCS2    = 0xa, // LEG_12=0xa,
    LM_MCS3    = 0xe, // LEG_18=0xe,
    LM_MCS4    = 0x9, // LEG_24=0x9,
    LM_MCS5    = 0xd, // LEG_36=0xd,
    LM_MCS6    = 0x8, // LEG_48=0x8,
    LM_MCS7    = 0xc  // LEG_54=0xc
} LegRate_t;

typedef enum {
    SHORT_PREAMBLE = 0,
    LONG_PREAMBLE = 1
} PreambleType_t;

 
typedef struct  {
    const char * name;
    int32_t    * value;
} arg_t;

typedef struct {
    uint32_t macbyp_mode;
    uint32_t macbyp_nframes_in_burst;
    uint32_t macbyp_ifs_us;
    uint32_t macbyp_pld_type;

    uint32_t pwrlevel;
    uint32_t smoothing;
    uint32_t continuoustx;
    uint32_t formatmod;
    uint32_t lm_length;
    uint32_t lm_rate;
    uint32_t ht_length;
    uint32_t ht_mcs;
    uint32_t ofdm_shortgi;
    uint32_t dsss_preamble_type;

    uint32_t fe_ofdm_filter_sel;
    uint32_t fe_dsss_filter_sel;
    uint32_t fe_dsss_gain;
    uint32_t fe_ofdm_gain;
    int32_t  fe_dvga_qdb;
} TxCfg_t;

static uint8_t s_txvec[16];
static TxCfg_t s_txcfg;
static int32_t s_dsss_coeff0[20] = {0,-1,-1,1,1,-2,-4,2,20,42,52,42,20,2,-4,-2,1,1,-1,-1};
static int32_t s_dsss_coeff1[20] = {0,0,0,0,-1,-1,2,10,23,35,40,35,23,10,2,-1,-1,0,0,0};
struct phy_cfg_tag s_phycfg;
static uint8_t s_rate_table_11g[8] = {LM_MCS0,LM_MCS1,LM_MCS2,LM_MCS3,LM_MCS4,LM_MCS5,LM_MCS6,LM_MCS7};
static uint8_t s_rate_table_11n[8] = {0,1,2,3,4,5,6,7};
static uint8_t s_nbmdm_on = 0;
static uint8_t s_txpwr = 0;

static inline void wait_us(uint32_t us)
{
    volatile uint32_t n = us*32;
    while (n--);
}

static inline void wait_ms(uint32_t ms)
{
    wait_us(1000*ms);
}

static void macbyp_frameperburst_setf(uint32_t frameperburst)
{
    uint32_t addr = MACBYP_CTRL_ADDR + 0x10;
    REG_PL_WR(addr,frameperburst);
}

static void txcfg_reset(TxCfg_t *cfg)
{
    /* macbypass config */
    s_txcfg.macbyp_mode = MACBYP_TX_BURST;
    s_txcfg.macbyp_nframes_in_burst = 0;
    s_txcfg.macbyp_ifs_us = 2000;
    s_txcfg.macbyp_pld_type = MACBYP_PLD_PRBS;
    
    /* tx config */
    s_txcfg.pwrlevel = 0;
    s_txcfg.smoothing = 1;
    s_txcfg.continuoustx = 0;
    s_txcfg.formatmod = NONHT;
    s_txcfg.lm_rate = LM_MCS7;
    s_txcfg.lm_length = 1000;
    s_txcfg.ht_mcs = 7;
    s_txcfg.ht_length = 4096;
    s_txcfg.dsss_preamble_type = LONG_PREAMBLE;

    /* fe config */
    s_txcfg.fe_ofdm_filter_sel = 0xffff;
    s_txcfg.fe_dsss_filter_sel = 0;
    s_txcfg.fe_dsss_gain = 32;
    s_txcfg.fe_ofdm_gain = 32;
    s_txcfg.fe_dvga_qdb = 0;
}

static void txvec_build(TxCfg_t *cfg)
{
    uint8_t agg = 0;
    uint8_t doze_not_allowed = 1;
    uint8_t disambiguity = 1;
    uint16_t service = 0;
    uint8_t sounding = 0;
    uint8_t feccoding = 0;
    uint8_t chbw = 0;
    uint8_t ness = 0;
    uint8_t nsts = 1;

    /* reset txvec */
    for (int i = 0; i < sizeof(s_txvec)/sizeof(s_txvec[0]); i++) {
        s_txvec[i] = 0;
    }

    /* txvec0: power control */
    s_txvec[0] = cfg->pwrlevel;
    /* txvec1 */
    s_txvec[1] = (0
        | chbw              << 6
        | cfg->continuoustx << 4
        | cfg->smoothing    << 3
        | feccoding         << 1
        | sounding          << 0
    );
    /* txvec2: antenna set */
    s_txvec[2] = 1; // 
    /* txvec3: spatial map matrix index */
    s_txvec[3] = 0;
    /* txvec4 */
    s_txvec[4] = (0
        | cfg->dsss_preamble_type << 7
        | cfg->ht_mcs             << 0
    );
    /* txvec5 */
    s_txvec[5] = (0
        | nsts           << 5
        | ness           << 3
        | cfg->formatmod << 0
    );
    /* txvec6 */
    s_txvec[6] = cfg->lm_length & 0xff;
    /* txvec7 */
    s_txvec[7] = (0
        | cfg->lm_rate << 4
        | ((cfg->lm_length >> 8) & 0xf) << 0
    );
    /* txvec8 */
    s_txvec[8] = (service >> 0) & 0xff;
    /* txvec9 */
    s_txvec[9] = (service >> 8) & 0xff;
    /* txvec10 */
    s_txvec[10] = (cfg->ht_length >> 0) & 0xff;
    /* txvec11 */
    s_txvec[11] = (cfg->ht_length >> 8) & 0xff;
    /* txvec12 */
    s_txvec[12] = (0
        | disambiguity      << 7
        | cfg->ofdm_shortgi << 6
        | doze_not_allowed  << 5
        | agg               << 4
        | ((cfg->ht_length >> 16) & 0xf)  << 0
    );
    /* txvec13: nTx=0,stbc=0 (no STBC) */
    s_txvec[13] = 0;
    /* txvec14: partial AID */
    s_txvec[14] = 0;
    /* group ID, parital AID */
    s_txvec[15] = 0;
}

static void txvec_program()
{
    macbyp_txv0_setf(s_txvec[0]);
    macbyp_txv1_setf(s_txvec[1]);
    macbyp_txv2_setf(s_txvec[2]);
    macbyp_txv3_setf(s_txvec[3]);
    macbyp_txv4_setf(s_txvec[4]);
    macbyp_txv5_setf(s_txvec[5]);
    macbyp_txv6_setf(s_txvec[6]);
    macbyp_txv7_setf(s_txvec[7]);
    macbyp_txv8_setf(s_txvec[8]);
    macbyp_txv9_setf(s_txvec[9]);
    macbyp_txv10_setf(s_txvec[10]);
    macbyp_txv11_setf(s_txvec[11]);
    macbyp_txv12_setf(s_txvec[12]);
    macbyp_txv13_setf(s_txvec[13]);
    macbyp_txv14_setf(s_txvec[14]);
    macbyp_txv15_setf(s_txvec[15]);
}

static void phy_start_transmit(TxCfg_t *cfg)
{
    int32_t *dsss_coeff;

    // configure TxVector
    txvec_build(cfg);
    txvec_program();

    // configure FE registers
    rc2_txhbf20coeffsel_setf(cfg->fe_ofdm_filter_sel);

    dsss_coeff = (cfg->fe_dsss_filter_sel == 0) ? s_dsss_coeff0 : s_dsss_coeff1;
    REG_PL_WR(0x44c03030,0
        | (dsss_coeff[0] & 0x7f) << 0
        | (dsss_coeff[1] & 0x7f) << 8
        | (dsss_coeff[2] & 0x7f) << 16
        | (dsss_coeff[3] & 0x7f) << 24
    );
    REG_PL_WR(0x44c03034,0
        | (dsss_coeff[4] & 0x7f) << 0
        | (dsss_coeff[5] & 0x7f) << 8
        | (dsss_coeff[6] & 0x7f) << 16
        | (dsss_coeff[7] & 0x7f) << 24
    );
    REG_PL_WR(0x44c03038,0
        | (dsss_coeff[8] & 0x7f) << 0
        | (dsss_coeff[9] & 0x7f) << 8
        | (dsss_coeff[10] & 0x7f) << 16
        | (dsss_coeff[11] & 0x7f) << 24
    );
    REG_PL_WR(0x44c0303c,0
        | (dsss_coeff[12] & 0x7f) << 0
        | (dsss_coeff[13] & 0x7f) << 8
        | (dsss_coeff[14] & 0x7f) << 16
        | (dsss_coeff[15] & 0x7f) << 24
    );
    REG_PL_WR(0x44c03040,0
        | (dsss_coeff[16] & 0x7f) << 0
        | (dsss_coeff[17] & 0x7f) << 8
        | (dsss_coeff[18] & 0x7f) << 16
        | (dsss_coeff[19] & 0x7f) << 24
    );
    
    riu_tx20diggainlin0_setf(cfg->fe_ofdm_gain);
    rc2_txdsssdiggainlin0_setf(cfg->fe_dsss_gain | 0x80); // 0x80=enable dsss digital gain

    // // configure RF
    // rfc_txdfe_set_dvga(cfg->fe_dvga_qdb);
    // rfc_hwctrl_txdvga(0);

    // configure macbypass
    macbyp_frameperburst_setf(cfg->macbyp_nframes_in_burst);
    macbyp_pre_tx_delay_setf(cfg->macbyp_ifs_us*30); // mpif clock is 30M
    macbyp_payload_setf(cfg->macbyp_pld_type);

    // start macbypass
    macbyp_ctrl_pack(0,cfg->macbyp_mode,0,MACBYP_ON);
}

// static void phy_stop_transmit()
// {
//     macbyp_ctrl_pack(0,MACBYP_IDLE,0,MACBYP_OFF);
// }

static void phy_mdm_reset()
{
    mdm_mdmswreset_setf(1);
    mdm_agcswreset_setf(1);
    mdm_dspswreset_setf(1);

    wait_ms(1);

    mdm_mdmswreset_setf(0);
    mdm_agcswreset_setf(0);
    mdm_dspswreset_setf(0);
}

static void phy_macbyp_reset()
{
    macbyp_ctrl_pack(0,MACBYP_IDLE,0,0);
}

void phy_macbyp_clr_status()
{
    // macbyp_ctrl_pack(0,MACBYP_IDLE,1,1);    // mpif clk is enabled if mode != idle or if bypassed
    uint32_t value;
    uint32_t bypassed;

    value = *(volatile uint32_t *)MACBYP_CTRL_ADDR;
    bypassed = value & 1;

    // make sure mpif clk is enabled
    if (!bypassed) {
        value |= 1;
    }

    // set regb_clr_status
    value |= 1 << 4;
    *(volatile uint32_t *)MACBYP_CTRL_ADDR = value;

    // clr regb_clr_status
    value &= ~(1 << 4);
    *(volatile uint32_t *)MACBYP_CTRL_ADDR = value;

    // restore regb_bypass
    if (!bypassed) {
        value &= ~(1);
        *(volatile uint32_t *)MACBYP_CTRL_ADDR = value;
    }
}

void phy_macbyp_rx_start()
{
    macbyp_ctrl_pack(0,MACBYP_RX,0,MACBYP_ON);
}

float calc_ppm_ofdm(uint16_t rxv_freqoff)
{
    int16_t freqoff;
    float ppm;
    
    freqoff = (int16_t)rxv_freqoff;
    ppm = -1 * freqoff * 20.0 / 2440;   // 602 as the center
    ppm = ppm/(s_nbmdm_on+1);

    return ppm;
}

float calc_ppm_dsss(uint8_t rxv_freqoff)
{
    int8_t freqoff;
    float ppm;

    freqoff = (int8_t)rxv_freqoff;
    ppm = freqoff * 0.7;  // 602 as the center

    return ppm;
}

void phy_macbyp_rx_print_status()
{
    static uint32_t stat_frame_ok;  
    __attribute__((unused)) static uint32_t stat_frame_bad; 
    static uint32_t stat_rxend;     
    __attribute__((unused)) static uint32_t stat_rxerr;     
    static uint32_t stat_evm;       
    static uint32_t stat_rssi;      

    stat_frame_ok  = macbyp_stat_frame_ok_get();
    stat_frame_bad = macbyp_stat_frame_bad_get();
    stat_rxend     = macbyp_stat_rxend_get();
    stat_rxerr     = macbyp_stat_rxerror_get();
    stat_evm       = macbyp_evm_get();
    stat_rssi      = macbyp_rssi_get();
    float ppm_dsss = calc_ppm_dsss((uint8_t)((stat_evm >> 16) & 0xff));
    float ppm_ofdm = calc_ppm_ofdm((uint16_t)((stat_evm >> 16) & 0xffff));
    uint16_t gain_status = stat_rssi >> 16;
    int gain_lna = gain_status & 0xf;
    int gain_rbb = (gain_status >> 4) & 0x1f;
    int gain_dg  = (gain_status >> 9) & 0x7f;
    int rssi     = (int8_t)(stat_rssi & 0xff);

    gain_dg = (gain_dg > 63) ? gain_dg - 128 : gain_dg;

    printf("ok %5ld,end %5ld,rssi %4d,lna %d,rbb %2d,dg %4d,ppm_ofdm %+5.1f,ppm_dsss %+5.1f\r\n",
        stat_frame_ok,
        stat_rxend,
        rssi,
        gain_lna,
        gain_rbb,
        gain_dg,
        ppm_ofdm,
        ppm_dsss
    );
}

void phy_macbyp_rx_print_status_at()
{
    static uint32_t stat_frame_ok;  
    __attribute__((unused)) static uint32_t stat_frame_bad; 
    static uint32_t stat_rxend;     
    __attribute__((unused)) static uint32_t stat_rxerr;     
    static uint32_t stat_evm;       
    static uint32_t stat_rssi;      

    stat_frame_ok  = macbyp_stat_frame_ok_get();
    stat_frame_bad = macbyp_stat_frame_bad_get();
    stat_rxend     = macbyp_stat_rxend_get();
    stat_rxerr     = macbyp_stat_rxerror_get();
    stat_evm       = macbyp_evm_get();
    stat_rssi      = macbyp_rssi_get();
    float ppm_dsss = calc_ppm_dsss((uint8_t)((stat_evm >> 16) & 0xff));
    float ppm_ofdm = calc_ppm_ofdm((uint16_t)((stat_evm >> 16) & 0xffff));
    uint16_t gain_status = stat_rssi >> 16;
    int gain_lna = gain_status & 0xf;
    int gain_rbb = (gain_status >> 4) & 0x1f;
    int gain_dg  = (gain_status >> 9) & 0x7f;
    int rssi     = (int8_t)(stat_rssi & 0xff);

    gain_dg = (gain_dg > 63) ? gain_dg - 128 : gain_dg;

    // printf("ok %5ld,end %5ld,rssi %4d,lna %d,rbb %2d,dg %4d,ppm_ofdm %+5.1f,ppm_dsss %+5.1f\r\n",
    //     stat_frame_ok,
    //     stat_rxend,
    //     rssi,
    //     gain_lna,
    //     gain_rbb,
    //     gain_dg,
    //     ppm_ofdm,
    //     ppm_dsss
    // );

    printf("{\"rxok\":%ld,\"rxend\":%ld,\"rssi\":%d,",stat_frame_ok,stat_rxend,rssi);
    printf("\"lna\":%d,\"rbb\":%d,\"dg\":%d,",gain_lna,gain_rbb,gain_dg);
    printf("\"ppm_ofdm\":%.1f,\"ppm_dsss\":%.1f}\r\n",ppm_ofdm,ppm_dsss);
}

/*
static void phy_macbyp_rx_start()
{
	REG_PL_WR(0x54c60000,0x000);
	wait_us(100);
	REG_PL_WR(0x54c60000,0x101);
	wait_us(100);
}

static void phy_macbyp_rx_print()
{
	int32_t num_fcs_ok;
	int32_t num_fcs_bad;
	int32_t num_rx_end;
	int32_t num_rx_err;
	int32_t rssi;

	num_fcs_ok  = REG_PL_RD(0x54c60080);
	num_fcs_bad = REG_PL_RD(0x54c60084);
	num_rx_end  = REG_PL_RD(0x54c60088);
	num_rx_err  = REG_PL_RD(0x54c6008c);
	rssi        = REG_PL_RD(0x54c60094);

	rssi = (rssi & 0xff) << 24 >> 24;

	printf("FCS OK %5ld, RX END %5ld, RSSI %ld\r\n",num_fcs_ok,num_rx_end,rssi);
}

static void phy_app_rx()
{
    int k = 0;

    phy_macbyp_reset();
    phy_macbyp_rx_start();

    while (1) {
    	wait_ms(1000);
    	printf("check %20d: ",k++);
    	phy_macbyp_rx_print();
    }
}
*/

static void tx_stop()
{
    // stop mdm
    phy_mdm_reset();
    phy_macbyp_reset();
    
    // disable signal generator
    rfc_sg_stop();

    // stop and resotre txdfe settings
    rfc_txdfe_set_dvga(0);
    rfc_hwctrl_txdvga(1);
    rfc_txdfe_mux(RFC_TXDFE_IN_MUX_TO_BB);
    rfc_txdfe_stop();
}

static int32_t argname_compare(char *cmd,const char* tgt_cmd)
{
    return strncmp(cmd,tgt_cmd,strlen(tgt_cmd)) == 0;
}

__attribute__((unused)) static int32_t argvalue_read(const char* cmd,int32_t *value)
{
    int32_t status = -1;

    char* s = (char*)cmd;

    while ((*s) && (*s != '=')) {
        s++;
    }

    if (*s && *(s+1)) {
        status = 0;
        *value = (int32_t)atoi(s+1);

        printf("...atof(%s)=%ld",s+1,*value);
    }

    return status;
}
#if 0
static int32_t argvalue_reads(const char* cmd,char *value)
{
    int32_t status = -1;

    char* s = (char*)cmd;

    while ((*s) && (*s != '=')) {
        s++;
    }

    if (*s && *(s+1)) {
        status = 0;
        strcpy(value,s+1);
    }

    return status;
}
#endif

static void print_args(arg_t *arglist, int size_arglist, const char* func_name)
{
    int j;

    printf("args of %s:\r\n",func_name);
    for (j = 0; j < size_arglist; j++) {
        printf("  * %s = %ld\r\n",arglist[j].name,*arglist[j].value);
    }
}

static void parse_args(arg_t *arglist, int size_arglist, int argc, char **argv, const char* func_name)
{
    int i,j;

    for (i = 1; i < argc; i++) {
        printf("parse %s...",argv[i]);
        for (j = 0; j < size_arglist; j++) {
            if (argname_compare(argv[i],arglist[j].name)) {
                printf("argument match detected %d",j);
                argvalue_read(argv[i],arglist[j].value);
            }
        }
        printf("\r\n");
    }

    print_args(arglist,size_arglist,func_name);
}

static void cmd_init(char *buf, int len, int argc, char **argv)
{   
    uint32_t xtalfreq;
    uint32_t xtalfreq_valid;

    if (argc == 1) {
        printf("phy_init usage:\r\n");
        printf("  phy_init 40000000");
        return;
    }

    xtalfreq = (uint32_t)atoi(argv[1]);
    xtalfreq_valid = 0;

    if (xtalfreq == 24*1000*1000) xtalfreq_valid = 1;
    if (xtalfreq == 26*1000*1000) xtalfreq_valid = 1;
    if (xtalfreq == 32*1000*1000) xtalfreq_valid = 1;
    if (xtalfreq == 384*100*1000) xtalfreq_valid = 1;
    if (xtalfreq == 40*1000*1000) xtalfreq_valid = 1;
    if (xtalfreq == 52*1000*1000) xtalfreq_valid = 1;

    if (xtalfreq_valid) {
        printf("start rf/rc/phy init\r\n");
        rfc_init(xtalfreq);
        phy_mdm_reset();
        phy_init(&s_phycfg);
    }
    else {
        printf("unsupported xtalfreq\r\n");
    }
}

static void cmd_channel(char *buf, int len, int argc, char **argv)
{
    uint32_t chanfreq;

    if (argc == 1) {
        printf("phy_channel usage:\r\n");
        printf("  phy_channel 2472");
        return;
    }

    chanfreq = (uint32_t)atoi(argv[1]);

    printf("switch channel to %ld MHz\r\n",chanfreq);
    phy_set_channel(PHY_BAND_2G4, PHY_CHNL_BW_20, 0,chanfreq, chanfreq, 0);
}

static void cmd_rf_channel(char *buf, int len, int argc, char **argv)
{
    uint32_t chanfreq;

    if (argc == 1) {
        printf("rf_channel usage:\r\n");
        printf("  rf_channel 2472");
        return;
    }

    chanfreq = (uint32_t)atoi(argv[1]);

    printf("switch channel to %ld MHz\r\n",chanfreq);
    rfc_config_channel(chanfreq);
}

static void cmd_rf_channel_sw(char *buf, int len, int argc, char **argv)
{
    uint32_t chanfreq;

    if (argc == 1) {
        printf("rf_channel_sw usage:\r\n");
        printf("  rf_channel_sw 2472");
        return;
    }

    chanfreq = (uint32_t)atoi(argv[1]);

    printf("switch channel to %ld MHz\r\n",chanfreq);
    rfc_config_channel_sw(chanfreq);
}

static void cmd_txsin_start(char *buf, int len, int argc, char **argv)
{
    int32_t freq_hz = INT32_MIN;
    int32_t gain_dbfs = 0;
    int32_t complex_flag = 1;
    int32_t gain;
    int32_t nshift;
    int32_t natt;
    int32_t incstep;
    int32_t addr_i,addr_q;
    int32_t conj_flag;
    int32_t att_table_q7[6] = {128, 114, 102, 91, 81, 72};

    // int32_t i;
    // if (argc > 1) {
    //     for (i = 1; i < argc; i++) {
    //         printf("processing argv[%ld]: %s\r\n",i,argv[i]);
    //         if (argname_compare(argv[i],"freq")) {
    //             argvalue_read(argv[i],&freq_hz);
    //             printf("    freq=%ld\r\n",freq_hz);
    //         }
    //         else if (argname_compare(argv[i],"dbfs")) {
    //             argvalue_read(argv[i],&gain_dbfs);
    //             printf("    dbfs=%ld\r\n",gain_dbfs);
    //         }
    //         else if (argname_compare(argv[i],"cplx")) {
    //             argvalue_read(argv[i],&complex_flag);
    //             printf("    cplx=%ld\r\n",complex_flag);
    //         }
    //     }
    // }

    if (argc == 1) {
        printf("txsin_start usage:\r\n");
        printf("  (1) txsin_start incstep=1 dbfs=-6 cplx=1\r\n");
        printf("  (2) txsin_start freqhz=1  dbfs=-6 cplx=1\r\n");
        return;
    }

    arg_t args[] = {
        {"dbfs",&gain_dbfs},
        {"cplx",&complex_flag},
        {"incstep",&incstep},
        {"freqhz",&freq_hz}
    };

    parse_args(args,sizeof(args)/sizeof(args[0]),argc,argv,__func__);

    // gain  = 1024;
    // nshift = (int32_t)(-gain_dbfs / 6 + 0.5);
    // if (nshift == 0) {
    //     gain = 1023;
    // }
    // else {
    //     gain = 1024 >> nshift;
    // }
    nshift = (int32_t)(-gain_dbfs / 6);
    natt = -gain_dbfs - 6*nshift;
    natt = natt < 0 ? 0 : natt > 5 ? 5 : natt; // make sure natt in range
    gain = ((1024 >> nshift) * att_table_q7[natt]) >> 7;
    gain = gain > 1023 ? 1023 : gain;
    
    conj_flag = complex_flag ? ((freq_hz!=INT32_MIN) ? freq_hz < 0 : incstep < 0) : 0;

    if (freq_hz != INT32_MIN) {
        freq_hz = (freq_hz < 0) ? -freq_hz : freq_hz;
        incstep = (int32_t)(((float)freq_hz)*1024/80e6);
        printf("convert freqhz to incstep: %ld => %ld\r\n",freq_hz,incstep);
    }
    else {
        incstep = (incstep < 0) ? -incstep : incstep;
    }

    if (complex_flag) {
        addr_i = 0;
        addr_q = conj_flag ? 256 : 768;
    }
    else {
        addr_i = 0;
        addr_q = 0;
    }

    rfc_hwctrl_txdvga(0);
    rfc_hwctrl_calparam(0);
    
    // make sure rf circuit is in TX state
    rfc_fsm_force(RFC_FSM_TX);

    // configure and start txdfe
    rfc_txdfe_set_dvga(0);
    rfc_txdfe_start();
    rfc_txdfe_mux(RFC_TXDFE_IN_MUX_TO_SINGEN);
    
    // start signal generator
    rfc_sg_start(incstep,gain,gain,addr_i,addr_q);

    printf("txsin start\r\n");
}

static void cmd_txsin_stop(char *buf, int len, int argc, char **argv)
{
    // disable signal generator
    rfc_sg_stop();

    // stop and resotre txdfe settings
    rfc_txdfe_mux(RFC_TXDFE_IN_MUX_TO_BB);
    rfc_txdfe_stop();

    // restore hwctrl
    rfc_hwctrl_txdvga(1);
    rfc_hwctrl_calparam(1);

    // reset fsm
    //rf_fsm_force(RF_STATE_SB);
    //rf_fsm_force(RF_STATE_FORCE_OFF);

    printf("txsin stop\r\n");
}

/*
  tx11b_start mcs=1 len=1000 filter=1 dg=32 dvga=-8 ifs=100
 */
static void cmd_tx11b_start(char *buf, int len, int argc, char **argv)
{
    int32_t mcs = 1;
    int32_t payload_len = 200;
    int32_t filter_sel = 0;
    int32_t diggain_dsss = 32;
    int32_t dvga = 0;
    int32_t ifs_us = 100;
    int32_t pwrlvl = 0;
    int32_t continuous = 0;

    if (argc == 1) {
        printf("tx11b_start usage:\r\n");
        printf("  tx11b_start mcs=[1|2|3|4] len=1000 filter=1 dg=32 dvga=0 ifs=100 continuous=0\r\n");
        return;
    }

    // parse args
    arg_t args[] = {
        {"mcs",&mcs},
        {"len",&payload_len},
        {"filter",&filter_sel},
        {"dg",&diggain_dsss},
        {"dvga",&dvga},
        {"ifs",&ifs_us},
        {"pwrlvl",&pwrlvl},
        {"continuous",&continuous}
    };

    parse_args(args,sizeof(args)/sizeof(args[0]),argc,argv,__func__);

    if (mcs < 1 || mcs > 4) {
        printf("mcs error\r\n");
        return;
    }

    // configure txcfg
    txcfg_reset(&s_txcfg);
    s_txcfg.macbyp_mode = MACBYP_TX_BURST;
    s_txcfg.macbyp_nframes_in_burst = 0;
    s_txcfg.macbyp_ifs_us = ifs_us;
    s_txcfg.formatmod = NONHT;
    s_txcfg.lm_rate = LM11B_MCS1 + (mcs - 1);
    s_txcfg.lm_length = payload_len;
    s_txcfg.fe_dsss_filter_sel = filter_sel;
    s_txcfg.fe_dsss_gain = diggain_dsss;
    s_txcfg.fe_dvga_qdb = dvga;
    s_txcfg.pwrlevel = pwrlvl << 2;
    s_txcfg.continuoustx = continuous;

    // reset
    tx_stop();
    phy_macbyp_reset();
    phy_mdm_reset();

    // configure and start transmit
    phy_start_transmit(&s_txcfg);
}

/*
  tx11g_start mcs=3 len=1000 filter=1 dg=32 dvga=-8 ifs=100 pwrlvl=10
 */
static void cmd_tx11g_start(char *buf, int len, int argc, char **argv)
{
    int32_t mcs = 1;
    int32_t payload_len = 1000;
    int32_t filter_sel = 0;
    int32_t diggain = 32;
    int32_t dvga = 0;
    int32_t ifs_us = 100;
    int32_t pwrlvl = 0;
    int32_t continuous = 0;

    if (argc == 1) {
        printf("tx11g_start usage:\r\n");
        printf("  tx11g_start mcs=[0~7] len=4096 filter=1 dg=32 dvga=0 ifs=100 continuous=0\r\n");
        return;
    }

    // parse args
    arg_t args[] = {
        {"mcs",&mcs},
        {"len",&payload_len},
        {"filter",&filter_sel},
        {"dg",&diggain},
        {"dvga",&dvga},
        {"ifs",&ifs_us},
        {"pwrlvl",&pwrlvl},
        {"continuous",&continuous}
    };

    parse_args(args,sizeof(args)/sizeof(args[0]),argc,argv,__func__);

    if (mcs < 0 || mcs > 7) {
        printf("mcs error\r\n");
        return;
    }

    // configure txcfg
    txcfg_reset(&s_txcfg);
    s_txcfg.macbyp_mode = MACBYP_TX_BURST;
    s_txcfg.macbyp_nframes_in_burst = 0;
    s_txcfg.macbyp_ifs_us = ifs_us;
    s_txcfg.formatmod = NONHT;
    s_txcfg.lm_rate = s_rate_table_11g[mcs];
    s_txcfg.lm_length = payload_len;
    s_txcfg.fe_ofdm_filter_sel = filter_sel ? 0x00ff : 0x0000;
    s_txcfg.fe_ofdm_gain = diggain;
    s_txcfg.fe_dvga_qdb = dvga;
    s_txcfg.pwrlevel = pwrlvl << 2;
    s_txcfg.continuoustx = continuous;

    // reset
    tx_stop();    
    phy_macbyp_reset();
    phy_mdm_reset();

    // configure and start transmit
    phy_start_transmit(&s_txcfg);
}

/*
  tx11n_start mcs=3 len=1000 filter=1 dg=32 dvga=-8 ifs=100
 */
static void cmd_tx11n_start(char *buf, int len, int argc, char **argv)
{
    int32_t mcs = 1;
    int32_t payload_len = 4096;
    int32_t filter_sel = 0;
    int32_t diggain = 32;
    int32_t dvga = 0;
    int32_t ifs_us = 100;
    int32_t pwrlvl = 0;
    int32_t smoothing = 0;
    int32_t continuous = 0;

    if (argc == 1) {
        printf("tx11n_start usage:\r\n");
        printf("  tx11n_start mcs=[0~7] len=4096 filter=1 dg=32 dvga=0 ifs=100 continuous=0\r\n");
        return;
    }

    // parse args
    arg_t args[] = {
        {"mcs",&mcs},
        {"len",&payload_len},
        {"filter",&filter_sel},
        {"dg",&diggain},
        {"dvga",&dvga},
        {"ifs",&ifs_us},
        {"pwrlvl",&pwrlvl},
        {"smoothing",&smoothing},
        {"continuous",&continuous}
    };

    parse_args(args,sizeof(args)/sizeof(args[0]),argc,argv,__func__);

    if (mcs < 0 || mcs > 7) {
        printf("mcs error\r\n");
        return;
    }

    // configure txcfg
    txcfg_reset(&s_txcfg);
    s_txcfg.macbyp_mode = MACBYP_TX_BURST;
    s_txcfg.macbyp_nframes_in_burst = 0;
    s_txcfg.macbyp_ifs_us = ifs_us;
    s_txcfg.formatmod = HT_MM;
    s_txcfg.ht_mcs = s_rate_table_11n[mcs];
    s_txcfg.ht_length = payload_len;
    s_txcfg.fe_ofdm_filter_sel = filter_sel ? 0xff00 : 0x0000;
    s_txcfg.fe_ofdm_gain = diggain;
    s_txcfg.fe_dvga_qdb = dvga;
    s_txcfg.pwrlevel = pwrlvl << 2;
    s_txcfg.smoothing = smoothing;
    s_txcfg.continuoustx = continuous;

    // reset
    tx_stop();
    phy_mdm_reset();
    phy_macbyp_reset();

    // configure and start transmit
    phy_start_transmit(&s_txcfg);
}

static void cmd_tx_stop(char *buf, int len, int argc, char **argv)
{
    tx_stop();
}

static void cmd_txdvga_force(char *buf, int len, int argc, char **argv)
{
    int32_t dvga;

    if (argc > 1) {
        dvga = (int32_t)atoi(argv[1]);
        rfc_hwctrl_txdvga(0);
        rfc_txdfe_set_dvga(dvga);
    }
}

static void cmd_txdvga_auto(char *buf, int len, int argc, char **argv)
{
    rfc_hwctrl_txdvga(1);
}

static void cmd_txpwr_set(char *buf, int len, int argc, char **argv)
{
    int32_t pwrlvl;

    if (argc > 1) {
        pwrlvl = (int32_t)atoi(argv[1]);
        pwrlvl = (pwrlvl < 0) ? 0 : pwrlvl;
        pwrlvl = (pwrlvl >15) ? 15: pwrlvl;
        s_txpwr = pwrlvl;
    }
}


// static void cmd_txiqcomp_force(char *buf, int len, int argc, char **argv)
// {
//     int32_t gain = 0x400;
//     int32_t phase = 0;
//     if (argc > 1) {
//         gain = (int32_t)atoi(argv[1]);
//         if (argc > 2) {
//             phase = (int32_t)atoi(argv[2]);
//         }
//         rfc_txdfe_set_iqcomp(1,gain,1,phase);
//         rfc_hwctrl_calparam(0);
//     }
// }

static void cmd_txiqgain_force(char *buf, int len, int argc, char **argv)
{
    int32_t gain = 0x400;
    int32_t en = 0;

    if (argc < 3) {
        printf("txiqgain_force en coeff\r\n");
        return;
    }
    
    en = (int32_t)atoi(argv[1]);
    gain = (int32_t)atoi(argv[2]);
    rfc_txdfe_set_iqgaincomp(en,gain);
    rfc_hwctrl_calparam(0);
}

static void cmd_txiqphase_force(char *buf, int len, int argc, char **argv)
{
    int32_t phase = 0x0;
    int32_t en = 0;

    if (argc < 3) {
        printf("txiqphase_force en coeff\r\n");
        return;
    }
    
    en = (int32_t)atoi(argv[1]);
    phase = (int32_t)atoi(argv[2]);
    rfc_txdfe_set_iqphasecomp(en,phase);
    rfc_hwctrl_calparam(0);
}

static void cmd_txdcc_force(char *buf, int len, int argc, char **argv)
{
    int32_t dcc_i = 0;
    int32_t dcc_q = 0;

    if (argc > 1) {
        dcc_i = (int32_t)atoi(argv[1]);
        if (argc > 2) {
            dcc_q = (int32_t)atoi(argv[2]);
        }
        rfc_txdfe_set_dccomp(dcc_i,dcc_q);
        rfc_hwctrl_calparam(0);
    }
}

static void cmd_rxdcc_force(char *buf, int len, int argc, char **argv)
{
    int32_t dcc_i = 0;
    int32_t dcc_q = 0;

    if (argc > 1) {
        dcc_i = (int32_t)atoi(argv[1]);
        if (argc > 2) {
            dcc_q = (int32_t)atoi(argv[2]);
        }
        rfc_rxdfe_set_dccomp(dcc_i,dcc_q);
        rfc_hwctrl_calparam(0);
    }
}

// static void cmd_rxiqcomp_force(char *buf, int len, int argc, char **argv)
// {
//     int32_t gain = 0x400;
//     int32_t phase = 0;
//     if (argc > 1) {
//         gain = (int32_t)atoi(argv[1]);
//         if (argc > 2) {
//             phase = (int32_t)atoi(argv[2]);
//         }
//         rfc_rxdfe_set_iqcomp(1,gain,1,phase);
//         rfc_hwctrl_calparam(0);
//     }
// }

static void cmd_rxiqgain_force(char *buf, int len, int argc, char **argv)
{
    int32_t gain = 0x400;
    int32_t en = 0;

    if (argc < 3) {
        printf("rxiqgain_force en coeff\r\n");
        return;
    }
    
    en = (int32_t)atoi(argv[1]);
    gain = (int32_t)atoi(argv[2]);
    rfc_rxdfe_set_iqgaincomp(en,gain);
    rfc_hwctrl_calparam(0);
}

static void cmd_rxiqphase_force(char *buf, int len, int argc, char **argv)
{
    int32_t phase = 0x0;
    int32_t en = 0;

    if (argc < 3) {
        printf("rxiqphase_force en coeff\r\n");
        return;
    }
    
    en = (int32_t)atoi(argv[1]);
    phase = (int32_t)atoi(argv[2]);
    rfc_rxdfe_set_iqphasecomp(en,phase);
    rfc_hwctrl_calparam(0);
}

static void cmd_calparam_auto(char *buf, int len, int argc, char **argv)
{
    rfc_hwctrl_calparam(1);
}

static void cmd_rxnotch0_set(char *buf, int len, int argc, char **argv)
{
    int32_t en = 0;
    int32_t alpha = 0;
    int32_t nrmfc = 0;
    double fc = 0;
    double fs = 40e6;

    if (argc < 4) {
        printf("rxnotch0 en alpha fc_hz\r\n");
        return;
    }
    
    en = (int32_t)atoi(argv[1]);
    alpha = (int32_t)atoi(argv[2]);
    fc = atof(argv[3]);
    nrmfc = (int32_t)(fc/fs*(1<<8) + 0.5);
    rfc_rxdfe_set_notch0(en,alpha,nrmfc);
}

static void cmd_rxnotch1_set(char *buf, int len, int argc, char **argv)
{
    int32_t en = 0;
    int32_t alpha = 0;
    int32_t nrmfc = 0;
    double fc = 0;
    double fs = 40e6;

    if (argc < 4) {
        printf("rxnotch1 en alpha fc_hz\r\n");
    }
    
    en = (int32_t)atoi(argv[1]);
    alpha = (int32_t)atoi(argv[2]);
    fc = atof(argv[3]);
    nrmfc = (int32_t)(fc/fs*(1<<8) + 0.5);
    rfc_rxdfe_set_notch1(en,alpha,nrmfc);
}

void rxpm_stop()
{
    rfc_pm_stop();
    rfc_rxdfe_stop();
    rfc_fsm_force(RFC_FSM_FORCE_OFF);
    // erf_config_gain_ctrl(1);    // make sure rxgain of erf is hardware controlled
    wait_us(500);
}

int32_t insel = 0;
int32_t acclen = 8192;
int32_t rshift = 10;
int32_t freqcw = INT32_MIN;
int32_t freqhz = INT32_MIN;
double fs = 80e6;
uint32_t pwr = 0;

static void cmd_rxpm_start(char *buf, int len, int argc, char **argv)
{
    if (argc == 1) {
        printf("rxpm_start usage:\r\n");
        printf("  (1) rxpm_start insel=[0|1|2] acclen=8192 rshift=10 freqcw=1024\r\n");
        printf("  (2) rxpm_start insel=[0|1|2] acclen=8192 rshift=10 freqhz=1e6\r\n");
        return;
    }

    freqcw = INT32_MIN;
    freqhz = INT32_MIN;

    arg_t args[] = {
        {"insel",&insel},
        {"acclen",&acclen},
        {"rshift",&rshift},
        {"freqcw",&freqcw},
        {"freqhz",&freqhz},
    };
    parse_args(args,sizeof(args)/sizeof(args[0]),argc,argv,__func__);

    if (freqhz != INT32_MIN) {
        freqcw = (int32_t)(2.0*freqhz/fs*(1<<19) + 0.5);
    }

    // make sure rxgain of erf is correct
    // erf_config_gain_ctrl(0);
    // erf_config_rxgain(E_RF_GC_RRF_54DB,E_RF_GC_RBB1_6DB,E_RF_GC_RBB2_6DB);
    // wait_us(500);

    // make sure rf is in rx state
    rfc_fsm_force(RFC_FSM_RX); 
    wait_us(500); 

    // start rxdfe
    rfc_rxdfe_start();

    // start power meter
    pwr = rfc_pm_start(insel,freqcw,acclen,rshift,0,0);

    // printf(RESULT_PREFIX "%ld\r\n",pwr);
    // printf("rxpm done... %ld\r\n",pwr);
    printf("{\"rxpm_pwr\":%ld,\"rxpm_insel\":%ld,\"rxpm_acclen\":%ld",pwr,insel,acclen);
    printf(",\"rxpm_rshift\":%ld,\"rxpm_freqcw\":%ld,\"rxpm_freqhz\":%ld}\r\n",rshift,freqcw,freqhz);
 
    rxpm_stop();
}

static void cmd_rxpm_stop(char *buf, int len, int argc, char **argv)
{
    rxpm_stop();
}

static void cmd_rxpm_query(char *buf, int len, int argc, char **argv)
{
    printf("{\"rxpm_pwr\":%ld,\"rxpm_insel\":%ld,\"rxpm_acclen\":%ld",pwr,insel,acclen);
    printf(",\"rxpm_rshift\":%ld,\"rxpm_freqcw\":%ld,\"rxpm_freqhz\":%ld}\r\n",rshift,freqcw,freqhz);
}

static void cmd_rxpm_sweep(char *buf, int len, int argc, char **argv)
{
    int32_t fstart = (int32_t)-20e6;
    int32_t fstop  = (int32_t)20e6;
    int32_t fstep  = (int32_t)1e6;
    int32_t insel  = 0;
    int32_t acclen = 2048;
    int32_t rshift = 8;
    int32_t freqcw;
    double fs = 80e6;
    int32_t freqhz;
    uint32_t pwr = 0;
    int32_t acc_i,acc_q;
    __attribute__((unused)) uint32_t lna,rbb1,rbb2;

    if (argc < 4) {
        printf("rxpm_sweep usage:\r\n");
        printf("  rxpm_sweep fstart=-20000000 fstop=20000000 fstep=1000000 insel=[0|1|2]\r\n");
        return;
    }

    arg_t args[] = {
        {"fstart",&fstart},
        {"fstop", &fstop},
        {"fstep", &fstep},
        {"insel", &insel},
        {"acclen", &acclen},
        {"rshift", &rshift}
    };
    parse_args(args,sizeof(args)/sizeof(args[0]),argc,argv,__func__);

    // // make sure rxgain of erf is correct
    // erf_config_gain_ctrl(0);
    // erf_config_rxgain(E_RF_GC_RRF_54DB,E_RF_GC_RBB1_6DB,E_RF_GC_RBB2_6DB);
    // wait_us(500);

    // erf_read_rxgain(&lna,&rbb1,&rbb2);
    // printf("RF gain : %ld/%ld/%ld\r\n",lna,rbb1,rbb2);

    // make sure rf is in rx state
    rfc_fsm_force(RFC_FSM_RX); 
    wait_us(500); 

    // start rxdfe
    rfc_rxdfe_start();

    printf("rxpm sweep result:\r\n");
    for (freqhz = fstart; freqhz <= fstop; freqhz += fstep) {
        wait_ms(1);
        freqcw = (int32_t)(2.0*freqhz/fs*(1<<19) + 0.5);
        pwr = rfc_pm_start(insel,freqcw,acclen,rshift,&acc_i,&acc_q);
        printf("  * freqhz=%+10ld,pwr=%10ld,acc=(%10ld,%10ld)\r\n",freqhz,pwr,acc_i>>rshift,acc_q>>rshift);
    }

    // rxpm_stop();
}

static void cmd_dump_phy(char *buf, int len, int argc, char **argv)
{

}

static void cmd_dump_rf(char *buf, int len, int argc, char **argv)
{
    printf("RF state\r\n");
    
    printf("  control : pu %ld,adda %ld, sdm %ld, lo %ld, lna %ld, txg %ld, rxg %ld\r\n",
        rf_pu_ctrl_hw_getf(),rf_adda_ctrl_hw_getf(),rf_sdm_ctrl_hw_getf(),rf_lo_ctrl_hw_getf(),
        rf_lna_ctrl_hw_getf(),rf_tx_gain_ctrl_hw_getf(),rf_rx_gain_ctrl_hw_getf());
    printf("            inc_fcal %ld, inc_acal %ld, trxcal %ld\r\n",
        rf_inc_fcal_ctrl_en_hw_getf(),rf_inc_acal_ctrl_en_hw_getf(),rf_trxcal_ctrl_hw_getf());
    
    printf("  pucr1_hw: sfreg %ld, pfd   %ld, fbdv  %ld, vco %ld, osmx   %ld\r\n",
        rf_pu_sfreg_hw_getf(),rf_pu_pfd_hw_getf(),rf_pu_fbdv_hw_getf(),rf_pu_vco_hw_getf(),rf_pu_osmx_hw_getf());
    printf("            trsw  %ld, txbuf %ld, rxbuf %ld\r\n",
        rf_trsw_en_hw_getf(),rf_pu_txbuf_hw_getf(),rf_pu_rxbuf_hw_getf());
    printf("            dac   %ld, tbb   %ld, tmx   %ld, pa  %ld, tosdac %ld\r\n",
        rf_pu_dac_hw_getf(),rf_pu_tbb_hw_getf(),rf_pu_tmx_hw_getf(),rf_pu_pa_hw_getf(),rf_pu_tosdac_hw_getf());
    printf("            adc   %ld, adcclk %ld, rbb  %ld, rmx %ld, rmxgm  %ld, lna %ld, adda_ldo %ld\r\n",
        rf_pu_adc_hw_getf(),rf_adc_clk_en_hw_getf(),rf_pu_rbb_hw_getf(),rf_pu_rmx_hw_getf(),rf_pu_rmxgm_hw_getf(),
        rf_pu_lna_hw_getf(),rf_pu_adda_ldo_hw_getf());

    printf("  calibration: tiq  %ld, riq  %ld, lol    %ld, tos    %ld, ros %ld, i_ros %ld\r\n",
        rf_tiqcal_en_getf(),rf_riqcal_en_getf(),rf_lo_leakcal_en_getf(),rf_toscal_en_getf(),
        rf_roscal_en_getf(),rf_roscal_inc_en_getf());
    printf("               acal %ld, fcal %ld, i_acal %ld, i_fcal %ld\r\n",
        rf_acal_en_getf(),rf_fcal_en_getf(),rf_acal_inc_en_getf(),rf_fcal_inc_en_getf());


    printf("  power: tbb %ld, tbb boost %ld, tmx %ld, dg %ld\r\n", 
        rf_gc_tbb_hw_getf(),
        rf_gc_tbb_boost_hw_getf(),
        rf_gc_tmx_hw_getf(),
        rf_tx_dvga_gain_ctrl_hw_getf());

    printf("  pa: \r\n");
    printf("      pa_ib_fix_hw:   %ld\r\n",rf_pa_ib_fix_hw_getf());
    printf("      pa_half_on_hw:  %ld\r\n",rf_pa_half_on_hw_getf());
    printf("      pa_vbcas_hw:    %ld\r\n",rf_pa_vbcas_hw_getf());
    printf("      pa_vbcore_hw:   %ld\r\n",rf_pa_vbcore_hw_getf());
    printf("      pa_iet_hw:      %ld\r\n",rf_pa_iet_hw_getf());
    printf("      pa_etb_en_hw:   %ld\r\n",rf_pa_etb_en_hw_getf());

}

static void cmd_dump_rfc(char *buf, int len, int argc, char **argv)
{
    printf("RF Controller state\r\n");
    // printf("  rf_tx_pow_lvl_hw = %lf\r\n",rfc_tx_pow)
}

#if 0
static void cmd_debug_clkpll(char *buf, int len, int argc, char **argv)
{
    REG_PL_WR(0x40000128,0x0f030b03);
    REG_PL_WR(0x40001220,0x10002010);
    REG_PL_WR(0x40001054,0x00800004);
    REG_PL_WR(0x400000d8,0x80000002);
    REG_PL_WR(0x400000e0,0x1);
}
#endif

static void cmd_dummy(char *buf, int len, int argc, char **argv)
{
    printf("not available\r\n");
}

static void cmd_reg_wr(char *buf, int len, int argc, char **argv)
{
    uint32_t addr;
    uint32_t value;
    int base = 16;
    char *endptr;

    if (argc < 3) {
        return;
    }

    // addr = (uint32_t)strtol(argv[1],&endptr,base);
    addr = (uint32_t)strtoul(argv[1],&endptr,base);
    if (endptr == argv[1]) {
        printf("%s: no digits found!\r\n",__func__);
        return;
    }

    // value = (uint32_t)strtol(argv[2],&endptr,base);
    value = (uint32_t)strtoul(argv[2],&endptr,base);
    if (endptr == argv[2]) {
        printf("%s: no digits found!\r\n",__func__);
        return;
    }

    printf("program external rf %08lx to addr %08lx\r\n",value,addr);
    rfc_write_reg(addr,value);
}

static void cmd_reg_rd(char *buf, int len, int argc, char **argv)
{
    uint32_t start_addr;
    uint32_t num_words;
    uint32_t read_value;
    int base = 16;
    char *endptr;

    if (argc < 2) {
        return;
    }
    else {
        start_addr = strtoul(argv[1],&endptr,base);
        if (endptr == argv[1]) {
            printf("%s: no digits found!\r\n",__func__);
            return;
        }
    }

    if (argc < 3) {
        num_words = 1;
    }
    else {
        num_words = atoi(argv[2]);
    }

    for (int i = 0; i < num_words; i++) {
        if (i % 8 == 0) {
            printf("\r\n%08lx:  ", start_addr + 4*i);
        }

        read_value = rfc_read_reg(start_addr + 4*i);
        printf("%08lx  ",read_value);
    }
    printf("\r\n");
}

static void cmd_rx_start(char *buf, int len, int argc, char **argv)
{
    int32_t time_prev = xTaskGetTickCount();
    int32_t time_pkt  = xTaskGetTickCount();
    __attribute__((unused)) int32_t time_last_print = xTaskGetTickCount();
    int32_t time_break = 45*1000;
    int32_t time_out = 2*1000;
    __attribute__((unused)) int32_t time_print = 800;
    int32_t time_curr;
    int32_t frame_ok = 0;
    int test_on_going = 0;

    if (argc < 3) {
        printf("rx_start usage:");
        printf("  rx_start clr_sec stop_sec\r\n");
        printf("rx start using default time (clr_stat_time=%ld,timeout_time=%ld)\r\n",time_out/1000,time_break/1000);
    }
    else {
        time_out = atoi(argv[1])*1000;
        time_break = atoi(argv[2])*1000;
    }

    // reset
    phy_mdm_reset();
    phy_macbyp_reset();

    // clear
    phy_macbyp_clr_status();

    // start
    phy_macbyp_rx_start();


    printf("start receiving\r\n");

    while (1) {
        time_curr = xTaskGetTickCount();

        if (time_curr - (int32_t)time_pkt > time_break) {
            printf("rx timeout...exit\r\n");
            break;
        }

        // if (test_on_going && (time_curr - (int32_t)time_prev > time_out)) {
        if (time_curr - (int32_t)time_prev > time_out) {
            if (test_on_going) {
                printf("rx status cleared...\r\n");
                test_on_going = 0;
                frame_ok = 0;
            }
            time_prev = time_curr;
            phy_mdm_reset();
            phy_macbyp_reset();
            phy_macbyp_clr_status();
            phy_macbyp_rx_start();
        }

        if ((macbyp_stat_frame_ok_get() > frame_ok)) {// && (time_curr - time_prev > time_print)) {
            test_on_going = 1;
            frame_ok = macbyp_stat_frame_ok_get();
            time_prev = xTaskGetTickCount();
            time_pkt  = xTaskGetTickCount();
            phy_macbyp_rx_print_status();
        }
    }
}

static void cmd_rx_start_at(char *buf, int len, int argc, char **argv)
{
    // reset
    phy_mdm_reset();
    phy_macbyp_reset();

    // clear
    phy_macbyp_clr_status();

    // start
    phy_macbyp_rx_start();

    printf("at_rx_start ok\r\n");
}

static void cmd_rx_poll_stat_at(char *buf, int len, int argc, char **argv)
{
    phy_macbyp_rx_print_status_at();
    printf("at_rx_poll ok\r\n");
}

static void cmd_rx_clear_stat_at(char *buf, int len, int argc, char **argv)
{
    // clear
    phy_macbyp_clr_status();
    printf("at_rx_clr ok\r\n");
}

static void cmd_enhance_on(char *buf, int len, int argc, char **argv)
{
    s_nbmdm_on = 1;
    printf("wifi enhance mode on\r\n");

    mdm_mdmconf_setf(1);
    rfc_config_bandwidth(RFC_BW_10M);
}

static void cmd_enhance_off(char *buf, int len, int argc, char **argv)
{
    s_nbmdm_on = 0;
    printf("wifi enhance mode off\r\n");
    mdm_mdmconf_setf(0);
    rfc_config_bandwidth(RFC_BW_20M);
}

#if 0
static void cmd_rx_pf_on(char *buf, int len, int argc, char **argv)
{
    printf("rx pulse filter on\r\n");
    rf_rx_pf_i_en_setf(1);
    rf_rx_pf_q_en_setf(1);
}

static void cmd_rx_pf_off(char *buf, int len, int argc, char **argv)
{
    printf("rx pulse filter off\r\n");
    rf_rx_pf_i_en_setf(0);
    rf_rx_pf_q_en_setf(0);
}
#endif

static void cmd_rf_pkdet(char *buf, int len, int argc, char **argv)
{
    uint32_t pkdetvth;
    if (argc == 1) {
        pkdetvth = rf_rbb_pkdet_vth_getf();
        printf("pkdetvth = %ld\r\n",pkdetvth);
        return;
    }

    pkdetvth = atoi(argv[1]);
    rf_rbb_pkdet_vth_setf(pkdetvth);
}

static void cmd_rf_filterbw(char *buf, int len, int argc, char **argv)
{
    uint32_t cw;
    if (argc == 1) {
        return;
    }

    cw = atoi(argv[1]);
    rf_rbb_cap1_fc_i_setf(cw);
    rf_rbb_cap1_fc_q_setf(cw);
    rf_rbb_cap2_fc_i_setf(cw);
    rf_rbb_cap2_fc_q_setf(cw);

    printf("set analog filter fc capcode to %ld\r\n",cw);
}

static void cmd_rf_pa_halfon_wifi(char *buf, int len, int argc, char **argv)
{
    uint32_t on;
    if (argc == 1) {
        return;
    }

    on = atoi(argv[1]);
    if (on) {
        rf_pa_half_on_wifi_setf(1);
        printf("WiFi PA half on enabled\r\n");
    }
    else {
        rf_pa_half_on_wifi_setf(0);
        printf("WiFi PA half on disabled\r\n");
    }    
}

static void cmd_rf_pa_halfon(char *buf, int len, int argc, char **argv)
{
    uint32_t on;
    if (argc == 1) {
        return;
    }

    on = atoi(argv[1]);
    if (on) {
        rf_pa_half_on_setf(1);
        printf("PA half on (BLE/SW control) enabled\r\n");
    }
    else {
        rf_pa_half_on_setf(0);
        printf("PA half on (BLE/SW control) disabled\r\n");
    }    
}

static void cmd_rf_trxcal_ctrl(char *buf, int len, int argc, char **argv)
{
    uint32_t on;

    if (argc > 1) {
        on = atoi(argv[1]);
        rf_trxcal_ctrl_hw_setf(on);  
    }

    printf("trxcal_ctrl_hw : %ld\r\n", rf_trxcal_ctrl_hw_getf());
}

static void cmd_rf_txgain_ctrl(char *buf, int len, int argc, char **argv)
{
    uint32_t on;

    if (argc > 1) {
        on = atoi(argv[1]);
        rf_tx_gain_ctrl_hw_setf(on);  
    }

    printf("txgain_ctrl_hw : %ld\r\n", rf_tx_gain_ctrl_hw_getf());
}

static void cmd_rf_caldata_dump(char *buf, int len, int argc, char **argv)
{
    int i;
    printf("[CALIBRATION_DATA_START]\r\n");
    printf("{");

    printf("\"lo_fcal\": [");
    for (i = 0; i < E_RF_CHANNEL_NUM; i++) {
        printf("%d",rf_calib_data->lo[i].fcal);
        if (i < E_RF_CHANNEL_NUM - 1) printf(",");
    }
    printf("]");

    printf(",\\\r\n");

    printf("\"lo_acal\": [");
    for (i = 0; i < E_RF_CHANNEL_NUM; i++) {
        printf("%d",rf_calib_data->lo[i].acal);
        if (i < E_RF_CHANNEL_NUM - 1) printf(",");
    }
    printf("]");

    printf(",\\\r\n");
    printf("\"rbb_i\": %d", rf_calib_data->cal.rbb_cap1_fc_i);

    printf(",\\\r\n");
    printf("\"rbb_q\": %d", rf_calib_data->cal.rbb_cap1_fc_q);

    printf(",\\\r\n");

    printf("\"ros_i\": [");
    for (i = 0; i < E_RF_RXCAL_GAIN_CNT; i++) {
        printf("%d",rf_calib_data->rxcal[i].rosdac_i);
        if (i < E_RF_RXCAL_GAIN_CNT - 1) printf(",");
    }
    printf("]");

    printf(",\\\r\n");

    printf("\"ros_q\": [");
    for (i = 0; i < E_RF_RXCAL_GAIN_CNT; i++) {
        printf("%d",rf_calib_data->rxcal[i].rosdac_q);
        if (i < E_RF_RXCAL_GAIN_CNT - 1) printf(",");
    }
    printf("]");

    printf(",\\\r\n");

    printf("\"tos_i\": [");
    for (i = 0; i < E_RF_TXCAL_GAIN_CNT; i++) {
        printf("%d",rf_calib_data->txcal[i].tosdac_i);
        if (i < E_RF_TXCAL_GAIN_CNT - 1) printf(",");
    }
    printf("]");

    printf(",\\\r\n");

    printf("\"tos_q\": [");
    for (i = 0; i < E_RF_TXCAL_GAIN_CNT; i++) {
        printf("%d",rf_calib_data->txcal[i].tosdac_q);
        if (i < E_RF_TXCAL_GAIN_CNT - 1) printf(",");
    }
    printf("]");

    printf(",\\\r\n");

    printf("\"tiq_g\": [");
    for (i = 0; i < E_RF_TXCAL_GAIN_CNT; i++) {
        printf("%d",rf_calib_data->txcal[i].tx_iq_gain_comp);
        if (i < E_RF_TXCAL_GAIN_CNT - 1) printf(",");
    }
    printf("]");

    printf(",\\\r\n");

    printf("\"tiq_p\": [");
    for (i = 0; i < E_RF_TXCAL_GAIN_CNT; i++) {
        printf("%d",(rf_calib_data->txcal[i].tx_iq_phase_comp >= 512 ? rf_calib_data->txcal[i].tx_iq_phase_comp - 1024 : rf_calib_data->txcal[i].tx_iq_phase_comp));
        if (i < E_RF_TXCAL_GAIN_CNT - 1) printf(",");
    }
    printf("]");
    printf("}");
    printf("\r\n");
    
    printf("[CALIBRATION_DATA_END]\r\n");
}

static void cmd_rf_cal_rerun(char *buf, int len, int argc, char **argv)
{
    rfc_reset();
    rfc_init(40*1000*1000);
}

static uint32_t tbb_boost = 0;
static uint32_t tbb = 6;
static uint32_t tmx = 5;
static uint32_t mode;

static void cmd_rf_power_auto(char *buf, int len, int argc, char **argv)
{
    printf("rf tx power hardware control on\r\n");
    mode = RFC_PC_AUTO;
    rfc_hwctrl_txrfgain(1);
    rfc_config_power(mode,tbb_boost,tbb,tmx);
}

static void cmd_rf_power_force(char *buf, int len, int argc, char **argv)
{
    // mode, tbb_boost, tbb, tmx
    // if (argc < 2) {
    //     printf("rx_power_force usage:\r\n");
    //     printf("  rf_power_force mode=<11b|11n|11g|ble> tbb_boost=0 tbb=6 tmx=5\r\n");
    //     return;
    // }
    // arg_t args[] = {
    //     {"tbb_boost",&tbb_boost},
    //     {"tbb",&tbb},
    //     {"tmx",&tmx},
    // };
    // parse_args(args,sizeof(args)/sizeof(args[0]),argc-1,&argv[1],__func__);
    // printf("new tx power: mode=%s,tbb_boost=%ld,tbb=%ld,tmx=%ld\r\n",tbb_boost,tbb,tmx);

    if (argc == 1) {
        printf("rx_power_force usage:\r\n");
        printf("  rf_power_force 11b|11n|11g|ble tbb_boost tbb tmx\r\n");
        return;
    }

    if (strcmp("11b",argv[1]) == 0) {
        mode = RFC_PC_WLAN_11B;
    }
    else if (strcmp("11g",argv[1]) == 0) {
        mode = RFC_PC_WLAN_11G;
    }
    else if (strcmp("11n",argv[1]) == 0) {
        mode = RFC_PC_WLAN_11N;
    }
    else if (strcmp("ble",argv[1]) == 0) {
        mode = RFC_PC_BT_BLE;
    }

    if (argc > 2) tbb_boost = (uint32_t)atoi(argv[2]);
    if (argc > 3) tbb       = (uint32_t)atoi(argv[3]);
    if (argc > 4) tmx       = (uint32_t)atoi(argv[4]);

    rfc_hwctrl_txrfgain(0);
    rfc_config_power(mode,tbb_boost,tbb,tmx);
}

// One param per update, no correctness check
//   agc_param_update param_name param_value
static void cmd_agc_param_update(char *buf, int len, int argc, char **argv)
{
    char *param_name;
    int32_t param_value;

    if (argc < 3) {
        return;
    }

    param_name = argv[1];
    param_value = atoi(argv[2]);
    
    if (strcmp("ENUP_QDB",param_name) == 0) {
        riu_rampupgapqdb_setf(param_value);
    }
    else if (strcmp("ENUP_NDL",param_name) == 0) {
        riu_rampupndlindex_setf(param_value);
    }
    else if (strcmp("ENDN_QDB",param_name) == 0) {
        riu_rampdngapqdb_setf(param_value);
    }
    else if (strcmp("ENDN_NDL",param_name) == 0) {
        riu_rampdnndlindex_setf(param_value);
    }
    else if (strcmp("DSSSDET",param_name) == 0) {
        rc2_evt4op2_setf(param_value);
    }
    else if (strcmp("PKDET",param_name) == 0) {
        riu_evt0op1_setf(param_value);
    }
    else if (strcmp("DISDET",param_name) == 0) {
        if (param_value) {
            riu_evt3op1_setf(25);
        }
        else {
            riu_evt3op1_setf(0);
        }
    }
    else if (strcmp("ADCSATDET",param_name) == 0) {
        if (param_value) {
            riu_evt1op3_setf(20);
        }
        else {
            riu_evt1op3_setf(0);
        }
    }
    else if (strcmp("RAMPUPDET",param_name) == 0) {
        if (param_value) {
            riu_evt2op2_setf(23);
        }
        else {
            riu_evt2op2_setf(0);
        }
    }
    else if (strcmp("ADCSAT_THR",param_name) == 0) {
        riu_sathighthrdbv_setf((uint8_t)(param_value + 4 + 64));
        riu_satlowthrdbv_setf((uint8_t)(param_value + 64));
        riu_satthrdbv_setf((uint8_t)(param_value + 1 + 64));
        printf("ADCSAT THR = %ld\r\n", param_value);
    }
    else if (strcmp("ADCSAT_TRIGGER_THR",param_name) == 0) {
        riu_satlowthrdbv_setf((uint8_t)(param_value + 64));
        printf("ADCSAT Trigger Threshold = %ld\r\n", param_value);
    }
    else if (strcmp("ADCSAT_HOLD_THR",param_name) == 0) {
        riu_sathighthrdbv_setf((uint8_t)(param_value + 64));
        printf("ADCSAT Hold Threshold = %ld\r\n", param_value);
    }
    else if (strcmp("ADCSAT_HIGH_THR",param_name) == 0) {
        riu_satthrdbv_setf((uint8_t)(param_value + 64));
        printf("ADCSAT High Threshold = %ld\r\n", param_value);
    }
    else if (strcmp("INBDPOWSUP",param_name) == 0) {
        riu_inbdpowsupthrdbm_setf((uint8_t)(256+param_value));
        printf("INBDPOWSUP = %d (%d)\r\n", (uint8_t)param_value, (int8_t)(param_value));
    }
    else if (strcmp("INBDPOWINF",param_name) == 0) {
        riu_inbdpowinfthrdbm_setf((uint8_t)(256+param_value));
        printf("INBDPOWINF = %d (%d)\r\n", (uint8_t)param_value, (int8_t)(param_value));
    }
    else if (strcmp("INBDPOWADJ_THR",param_name) == 0) {
        rc2_inbdpow_adj_thr_dbm_setf(param_value+256);
        printf("INBDPOWADJTHR = %d (%d)\r\n", (uint8_t)param_value, (int8_t)(param_value));
    }
    else if (strcmp("INBDPOWADJ_SUP",param_name) == 0) {
        rc2_inbdpowsupthr_adj_en_setf(param_value);
        printf("INBDPOWADJ_SUP = %s\r\n", param_value ? "ON" : "OFF");
    }
    else if (strcmp("INBDPOWADJ_INF",param_name) == 0) {
        rc2_inbdpowinfthr_adj_en_setf(param_value);
        printf("INBDPOWADJ_INF = %s\r\n", param_value ? "ON" : "OFF");
    }
    else if (strcmp("ADCPOWSUP",param_name) == 0) {
        riu_adcpowsupthrdbm_setf((uint8_t)(256+param_value));
        printf("ADCPOWSUP = %d (%d)\r\n", (uint8_t)param_value, (int8_t)(param_value));
    }
    else if (strcmp("ADCPOWINF",param_name) == 0) {
        riu_adcpowinfthrdbm_setf((uint8_t)(256+param_value));
        printf("ADCPOWINF = %d (%d)\r\n", (uint8_t)param_value, (int8_t)(param_value));
    }
    else if (strcmp("ADCPOWSUP_DBV",param_name) == 0) {
        riu_adcpowsupthrdbv_setf((uint8_t)(256+param_value));
        printf("ADCPOWSUP = %d (%d)\r\n", (uint8_t)param_value, (int8_t)(param_value));
    }
    else if (strcmp("ADCPOWINF_DBV",param_name) == 0) {
        riu_adcpowinfthrdbv_setf((uint8_t)(256+param_value));
        printf("ADCPOWINF = %d (%d)\r\n", (uint8_t)param_value, (int8_t)(param_value));
    }

    

    printf("agc param %s update done\r\n",param_name);
}

#if 0
// #define PRINT_PARAM_S8(x) printf("%s = %d\r\n",#x,(int8_t)( ## x ##_getf() ))
// #define PRINT_PARAM_S8(x) printf("%s = %d\r\n", #x , ## x ##_getf())
static void cmd_agc_param_dump(char *buf, int len, int argc, char **argv)
{
    // PRINT_PARAM_S8(riu_adcpowsupthrdbm);
    // PRINT_PARAM_S8(riu_adcpowinfthrdbm);
    // PRINT_PARAM_S8(riu_adcpowsupthrdbv);
    // PRINT_PARAM_S8(riu_adcpowinfthrdbv);

    // PRINT_PARAM_S8(riu_inbdpowsupthrdbm);
    // PRINT_PARAM_S8(riu_inbdpowinfthrdbm);
    
    // PRINT_PARAM_S8(riu_sathighthr);
    // PRINT_PARAM_S8(riu_satlowthr);
    // PRINT_PARAM_S8(riu_satthr);

    // // PRINT_PARAM_S8(rc2_inbdpow_adj_thr_dbm);
    // // PRINT_PARAM_S8(rc2_inbdpowsupthr_adj_en);
    // // PRINT_PARAM_S8(rc2_inbdpowinfthr_adj_en);
}
#endif

#if 0
extern void rf_pri_config_mode(uint32_t mode);
extern void rf_pri_config_channel(uint32_t channel_index);
extern void rf_pri_singen_config(uint32_t fcw,uint32_t start_addr_i,uint32_t start_addr_q);
extern void rf_pri_singen_start(void);
extern void rf_pri_save_state_for_cal();
extern void rf_pri_restore_state_for_cal();
extern void rf_pri_config_txgain(uint32_t gc_tbb_boost,uint32_t gc_tmx,uint32_t gc_tbb);
extern void rf_pri_wait_us(uint32_t us);
extern void rf_pri_wait_ms(uint32_t ms);

static int channel4cal = E_RF_CHANNEL_2440M;

void bruteforce_fcal()
{
}

void bruteforce_acal()
{
}

void bruteforce_tiq()
{
    rf_pri_config_channel(channel4cal);
    rf_pri_config_mode(E_RF_MODE_TXCAL);

    // turn on tiq calibration circuits
    rf_lo_leakcal_en_setf(1);
    rf_tiqcal_en_setf(1);
    rf_pwr_det_en_setf(1);
    rf_pa_pwrmx_dac_pn_switch_setf(1);

    // turn off tiq calibration circuits
    rf_lo_leakcal_en_setf(0);
    rf_tiqcal_en_setf(0);
    rf_pwr_det_en_setf(0);
    rf_pa_pwrmx_dac_pn_switch_setf(0);
}

void bruteforce_riq()
{
}

void bruteforce_ros()
{
}

void bruteforce_rbw()
{
}
#endif

static void cmd_fem_ctrl_on(char *buf, int len, int argc, char **argv)
{
    hal_fem_gpio_on();

    // FEM GPIO0
    rf_epa_rx2on_vector_00_setf(0);
    rf_epa_rxon_vector_00_setf(0);
    rf_epa_tx2on_vector_00_setf(1);
    rf_epa_tx2paon_vector_00_setf(1);
    rf_epa_txpaon_vector_00_setf(1);

    // FEM GPIO1
    rf_epa_rx2on_vector_01_setf(0);
    rf_epa_rxon_vector_01_setf(0);
    rf_epa_tx2on_vector_01_setf(0);
    rf_epa_tx2paon_vector_01_setf(0);
    rf_epa_txpaon_vector_01_setf(1);

    printf("FEM control on\r\n");
}

static void cmd_fem_ctrl_off(char *buf, int len, int argc, char **argv)
{
    hal_fem_gpio_off();
    printf("FEM control off\r\n");
}


// STATIC_CLI_CMD_ATTRIBUTE makes this(these) command(s) static
const static struct cli_command cmds_phy[] STATIC_CLI_CMD_ATTRIBUTE = {
    { "phy_init", "init rf driver",                         cmd_init},
    { "phy_channel", "configure work channel frequency",    cmd_channel},
    { "rf_channel", "configure rf channel (hw control)",    cmd_rf_channel},
    { "rf_channel_sw","configure rf channel (sw control)",  cmd_rf_channel_sw},
    { "rf_power_force","configure rf tx power",             cmd_rf_power_force},
    { "rf_power_auto", "tx power is hardware controlled",   cmd_rf_power_auto},
    { "rf_tos","set rf tos",                                cmd_dummy},
    { "rf_ros","set rf ros",                                cmd_dummy},
    { "rf_gain","read or set rf gain",                      cmd_dummy},
    { "rf_pkdet","read or set rf pkdet vth",                cmd_rf_pkdet},
    { "rf_filterbw","set rf analog filter bw",              cmd_rf_filterbw},
    { "rf_halfpa_wifi", "set rf pa half on for wifi",       cmd_rf_pa_halfon_wifi},
    { "rf_halfpa", "set rf pa half on for reg ctrl or ble", cmd_rf_pa_halfon},
    { "rf_trxcal_ctrl", "rf_trxcal_ctrl",                   cmd_rf_trxcal_ctrl},
    { "rf_txgain_ctrl", "rf_txgain_ctrl",                   cmd_rf_txgain_ctrl},
    { "rfcal_dump", "dump rf calibration data",             cmd_rf_caldata_dump},
    { "rfcal_run", "re-run rf calibration",                 cmd_rf_cal_rerun},
    { "txpwr_set", "set tx power level",                    cmd_txpwr_set},
    { "tx11b_start", "start transmit 11b frames",           cmd_tx11b_start},
    { "tx11b_stop", "start transmit 11b frames",            cmd_tx_stop},
    { "tx11g_start", "start transmit 11g frames",           cmd_tx11g_start},
    { "tx11g_stop", "start transmit 11g frames",            cmd_tx_stop},
    { "tx11n_start", "start transmit 11n frames",           cmd_tx11n_start},
    { "tx11n_stop", "start transmit 11n frames",            cmd_tx_stop},
    { "txsin_start", "transmit single tone",                cmd_txsin_start},
    { "txsin_stop", "stop transmit single tone",            cmd_txsin_stop},
    { "tx_stop", "stop transmit",                           cmd_tx_stop},
    { "txdvga_force", "force tx dvga gain (qdB)",           cmd_txdvga_force},
    { "txdvga_auto", "auto tx dvga gain",                   cmd_txdvga_auto},
    { "txdgofdm_force", "force ofdm digital gain (linear)", cmd_dummy},
    { "txdgdsss_force", "force dsss digital gain (linear)", cmd_dummy},
    { "txiqgain_force", "force tx iqgaincomp parameters",   cmd_txiqgain_force},
    { "txiqphase_force", "force tx iqphasecomp parameters", cmd_txiqphase_force},
    { "txdcc_force", "force tx dccomp parameters",          cmd_txdcc_force},
    { "rxdcc_force", "force rx dccomp parameters",          cmd_rxdcc_force},
    { "rxiqgain_force", "force rx iqgaincomp parameters",   cmd_rxiqgain_force},
    { "rxiqphase_force", "force rx iqgaincomp parameters",  cmd_rxiqphase_force},
    { "calparam_auto", "auto config calibration parameters",cmd_calparam_auto},
    { "rxnotch0", "config rx notch filter parameters",      cmd_rxnotch0_set},
    { "rxnotch1", "config rx notch filter parameters",      cmd_rxnotch1_set},
    { "rxpm_start", "start rx power meter",                 cmd_rxpm_start},
    { "rxpm_stop", "stop rx power meter",                   cmd_rxpm_stop},
    { "rxpm_sweep", "sweep using rx power meter",           cmd_rxpm_sweep},
    { "rxpm_query", "sweep using rx power meter",           cmd_rxpm_query},
    { "sram_play", "play waveform from sram",               cmd_dummy}, // TODO
    { "sram_dump", "dump waveform to sram",                 cmd_dummy}, // TODO
    { "rx_start", "start receiver",                         cmd_rx_start},
    { "at_rx_start", "start receiver for auto test",        cmd_rx_start_at},
    { "at_rx_poll","pull receiver state for auto test",     cmd_rx_poll_stat_at},
    { "at_rx_clr","clear receiver state for auto test",     cmd_rx_clear_stat_at},
    { "at_rx_restart","restart receiver state for auto test",cmd_rx_start_at},
    #if 0
    { "rxpf_on","rx pulse filter on",                       cmd_rx_pf_on},
    { "rxpf_off","rx pulse filter off",                     cmd_rx_pf_off},
    #endif
    { "reg_rd","register read",                             cmd_reg_rd},
    { "reg_wr","register write",                            cmd_reg_wr},
    { "enhance_on", "wifi enhance mode on",                 cmd_enhance_on},
    { "enhance_off", "wifi enhance mode off",               cmd_enhance_off},
    { "dump_phy", "dump current state",                     cmd_dump_phy},
    { "dump_rf",  "dump rf state",                          cmd_dump_rf},
    { "dump_rfc",  "dump rfc state",                        cmd_dump_rfc},
    { "agc_param_update", "update agc param",               cmd_agc_param_update},
    { "fem_ctrl_on", "turn on fem control",                 cmd_fem_ctrl_on},
    { "fem_ctrl_off", "turn off fem control",               cmd_fem_ctrl_off},
    #if 0
    { "agc_param_dump", "dump agc param",                   cmd_dummy}
    { "debug_clkpll","debug clock pll",                     cmd_debug_clkpll}
    #endif
};    

int phy_cli_register(void)
{
    // static command(s) do NOT need to call aos_cli_register_command(s) to register.
    // However, calling aos_cli_register_command(s) here is OK but is of no effect as cmds_user are included in cmds list.
    // XXX NOTE: Calling this *empty* function is necessary to make cmds_user in this file to be kept in the final link.
    //return aos_cli_register_commands(cmds_user, sizeof(cmds_user)/sizeof(cmds_user[0]));
    return 0;
}
