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

//#define RF_IN_SDK
#ifndef RF_IN_SDK

#include "bl602_rf_private.h"
#include "bl602_rf_calib_data.h"
#include "rf_reg.h"
#include "pds_reg.h"
#include "aon_reg.h"
#include "hbn_reg.h"
#include "bl602_common.h"
#include "bflb_platform.h"
// #define  rf_pri_printf MSG

#else

#include <stdio.h>
#include "bl602_rf_private.h"
#include "bl602_common.h"
#include "bl602_rf_calib_data.h"
// #include "bl602_rf.h"
#include "rf_reg.h"
#include "bl602_pds.h"
#include "bl602_aon.h"
#include "bl602_hbn.h"
#include "bl602.h"
// #define rf_pri_printf printf

#undef  MSG
#define MSG printf

#endif

#ifndef RF_DBG_LOG_DISABLE
#define rf_pri_printf    MSG
#define rf_pri_printw    MSG
#define rf_pri_printe    MSG
#else
#define rf_pri_printf    (...)
#define rf_pri_printw    MSG
#define rf_pri_printe    MSG
#endif

/** @addtogroup  BL602_Peripheral_Driver
 *  @{
 */

/** @addtogroup  RF_PRIVATE
 *  @{
 */

/** @defgroup  RF_PRIVATE_Private_Macros
 *  @{
 */
#define BL602_DBG_RF (0)

#define CNT_TIMES           (40)
#define NUM_CHANNELS        (21)
#define NUM_WIFI_CHANNELS   (14)

#define XTAL24M (0)
#define XTAL26M (0)
#define XTAL32M (0)
#define XTAL38P4M (0)
#define XTAL40M (1)
#define XTAL52M (0)
#define xtalType (0)

/*@} end of group RF_PRIVATE_Private_Macros */

/** @defgroup  RF_PRIVATE_Private_Types
 *  @{
 */

/*@} end of group RF_PRIVATE_Private_Types */

/** @defgroup  RF_PRIVATE_Private_Variables
 *  @{
 */
static uint32_t state_rf_fsm_ctrl_hw;
static uint32_t state_rfctrl_hw_en;
static uint32_t state_rfcal_ctrlen;
static uint32_t state_pucr1;
static uint32_t state_fbdv;
static uint32_t state_sdm1;
static uint32_t state_sdm2;
static uint32_t state_rbb3;
static uint32_t state_adda1;
static uint32_t state_dfe_ctrl_0;
static uint32_t state_dfe_ctrl_3;
static uint32_t state_dfe_ctrl_6;
static uint32_t state_dfe_ctrl_7;
static uint32_t state_trx_gain1;
static uint32_t state_singen_ctrl0;
static uint32_t state_singen_ctrl2;
static uint32_t state_singen_ctrl3;
static uint32_t state_sram_ctrl0;
static uint32_t state_sram_ctrl1;
static uint32_t state_sram_ctrl2;
static uint32_t state_rf_resv_reg_1;
static uint32_t state_pa1;
static uint32_t state_ten_ac;
static uint32_t state_rfif_dfe_ctrl0;
static uint32_t state_tbb;
static uint32_t state_vco2;

static uint32_t init_fast=0;

static uint32_t channel_div_table[E_RF_CHANNEL_NUM] = {
#if XTAL38P4M
    0x14F00000 , /* Channel 2412 MHz */
    0x14FB1C71 , /* Channel 2417 MHz */
    0x150638E3 , /* Channel 2422 MHz */
    0x15115555 , /* Channel 2427 MHz */
    0x151C71C7 , /* Channel 2432 MHz */
    0x15278E38 , /* Channel 2437 MHz */
    0x1532AAAA , /* Channel 2442 MHz */
    0x153DC71C , /* Channel 2447 MHz */
    0x1548E38E , /* Channel 2452 MHz */
    0x15540000 , /* Channel 2457 MHz */
    0x155F1C71 , /* Channel 2462 MHz */
    0x156A38E3 , /* Channel 2467 MHz */
    0x15755555 , /* Channel 2472 MHz */
    0x15900000 , /* Channel 2484 MHz */
    0x15C00000 , /* Channel 2505.6 MHz */
#endif
#if XTAL24M
    0x21800000 , /* Channel 2412 MHz */
    0x2191C71C , /* Channel 2417 MHz */
    0x21A38E38 , /* Channel 2422 MHz */
    0x21B55555 , /* Channel 2427 MHz */
    0x21C71C71 , /* Channel 2432 MHz */
    0x21D8E38E , /* Channel 2437 MHz */
    0x21EAAAAA , /* Channel 2442 MHz */
    0x21FC71C7 , /* Channel 2447 MHz */
    0x220E38E3 , /* Channel 2452 MHz */
    0x22200000 , /* Channel 2457 MHz */
    0x2231C71C , /* Channel 2462 MHz */
    0x22438E38 , /* Channel 2467 MHz */
    0x22555555 , /* Channel 2472 MHz */
    0x22800000 , /* Channel 2484 MHz */
    0x22CCCCCC , /* Channel 2505.6 MHz */
#endif
#if XTAL26M
    0x1EEC4EC4 , /* Channel 2412 MHz */
    0x1EFCB7CB , /* Channel 2417 MHz */
    0x1F0D20D2 , /* Channel 2422 MHz */
    0x1F1D89D8 , /* Channel 2427 MHz */
    0x1F2DF2DF , /* Channel 2432 MHz */
    0x1F3E5BE5 , /* Channel 2437 MHz */
    0x1F4EC4EC , /* Channel 2442 MHz */
    0x1F5F2DF2 , /* Channel 2447 MHz */
    0x1F6F96F9 , /* Channel 2452 MHz */
    0x1F800000 , /* Channel 2457 MHz */
    0x1F906906 , /* Channel 2462 MHz */
    0x1FA0D20D , /* Channel 2467 MHz */
    0x1FB13B13 , /* Channel 2472 MHz */
    0x1FD89D89 , /* Channel 2484 MHz */
    0x201F81F8 , /* Channel 2505.6 MHz */
#endif
#if XTAL32M
    0x19200000 , /* Channel 2412 MHz */
    0x192D5555 , /* Channel 2417 MHz */
    0x193AAAAA , /* Channel 2422 MHz */
    0x19480000 , /* Channel 2427 MHz */
    0x19555555 , /* Channel 2432 MHz */
    0x1962AAAA , /* Channel 2437 MHz */
    0x19700000 , /* Channel 2442 MHz */
    0x197D5555 , /* Channel 2447 MHz */
    0x198AAAAA , /* Channel 2452 MHz */
    0x19980000 , /* Channel 2457 MHz */
    0x19A55555 , /* Channel 2462 MHz */
    0x19B2AAAA , /* Channel 2467 MHz */
    0x19C00000 , /* Channel 2472 MHz */
    0x19E00000 , /* Channel 2484 MHz */
    0x1A199999 , /* Channel 2505.6 MHz */
#endif
#if XTAL40M
    // 0x14199999 , /* Channel 2412 MHz */
    // 0x14244444 , /* Channel 2417 MHz */
    // 0x142EEEEE , /* Channel 2422 MHz */
    // 0x14399999 , /* Channel 2427 MHz */
    // 0x14444444 , /* Channel 2432 MHz */
    // 0x144EEEEE , /* Channel 2437 MHz */
    // 0x14599999 , /* Channel 2442 MHz */
    // 0x14644444 , /* Channel 2447 MHz */
    // 0x146EEEEE , /* Channel 2452 MHz */
    // 0x14799999 , /* Channel 2457 MHz */
    // 0x14844444 , /* Channel 2462 MHz */
    // 0x148EEEEE , /* Channel 2467 MHz */
    // 0x14999999 , /* Channel 2472 MHz */
    // 0x14B33333 , /* Channel 2484 MHz */
    // 0x14E147AE , /* Channel 2505.6 MHz */
    0x14088889,
    0x14111111,
    0x1419999A,
    0x14222222,
    0x142AAAAB,
    0x14333333,
    0x143BBBBC,
    0x14444444,
    0x144CCCCD,
    0x14555555,
    0x145DDDDE,
    0x14666666,
    0x146EEEEF,
    0x14777777,
    0x14800000,
    0x14888889,
    0x14911111,
    0x1499999A,
    0x14A22222,
    0x14AAAAAB,
    0x14B33333,
#endif
#if XTAL52M
    0xF762762 , /* Channel 2412 MHz */
    0xF7E5BE5 , /* Channel 2417 MHz */
    0xF869069 , /* Channel 2422 MHz */
    0xF8EC4EC , /* Channel 2427 MHz */
    0xF96F96F , /* Channel 2432 MHz */
    0xF9F2DF2 , /* Channel 2437 MHz */
    0xFA76276 , /* Channel 2442 MHz */
    0xFAF96F9 , /* Channel 2447 MHz */
    0xFB7CB7C , /* Channel 2452 MHz */
    0xFC00000 , /* Channel 2457 MHz */
    0xFC83483 , /* Channel 2462 MHz */
    0xFD06906 , /* Channel 2467 MHz */
    0xFD89D89 , /* Channel 2472 MHz */
    0xFEC4EC4 , /* Channel 2484 MHz */
    0x100FC0FC , /* Channel 2505.6 MHz */
#endif
};

static uint16_t channel_cnt_table[E_RF_CHANNEL_NUM] = {
#if XTAL38P4M
    0xa780, //2412MHz
    0xa7d8, //2417MHz
    0xa831, //2422MHz
    0xa88a, //2427MHz
    0xa8e3, //2432MHz
    0xa93c, //2437MHz
    0xa995, //2442MHz
    0xa9ee, //2447MHz
    0xaa47, //2452MHz
    0xaaa0, //2457MHz
    0xaaf8, //2462MHz
    0xab51, //2467MHz
    0xabaa, //2472MHz
    0xac80  //2484MHz
#endif
#if XTAL24M
    0xa780, //2412MHz
    0xa7d8, //2417MHz
    0xa831, //2422MHz
    0xa88a, //2427MHz
    0xa8e3, //2432MHz
    0xa93c, //2437MHz
    0xa995, //2442MHz
    0xa9ee, //2447MHz
    0xaa47, //2452MHz
    0xaaa0, //2457MHz
    0xaaf8, //2462MHz
    0xab51, //2467MHz
    0xabaa, //2472MHz
    0xac80  //2484MHz
#endif
#if XTAL26M
    0xa78a, //2412MHz
    0xa7e3, //2417MHz
    0xa83c, //2422MHz
    0xa895, //2427MHz
    0xa8ed, //2432MHz
    0xa946, //2437MHz
    0xa99f, //2442MHz
    0xa9f8, //2447MHz
    0xaa51, //2452MHz
    0xaaaa, //2457MHz
    0xab03, //2462MHz
    0xab5c, //2467MHz
    0xabb5, //2472MHz
    0xac8a  //2484MHz
#endif
#if XTAL32M
    0xa788, //2412MHz
    0xa7e1, //2417MHz
    0xa83a, //2422MHz
    0xa893, //2427MHz
    0xa8ec, //2432MHz
    0xa944, //2437MHz
    0xa99d, //2442MHz
    0xa9f6, //2447MHz
    0xaa4f, //2452MHz
    0xaaa8, //2457MHz
    0xab01, //2462MHz
    0xab5a, //2467MHz
    0xabb3, //2472MHz
    0xac88  //2484MHz
#endif
#if XTAL40M
    // 0xA779, //2412MHz
    // 0xA7D2, //2417MHz
    // 0xA82B, //2422MHz
    // 0xA883, //2427MHz
    // 0xA8DC, //2432MHz
    // 0xA935, //2437MHz
    // 0xA98E, //2442MHz
    // 0xA9E7, //2447MHz
    // 0xAA40, //2452MHz
    // 0xAA99, //2457MHz
    // 0xAAF2, //2462MHz
    // 0xAB4A, //2467MHz
    // 0xABA3, //2472MHz
    // 0xAC79  //2484MHz
    0xA6EB,//2404
    0xA732,//2408
    0xA779,//2412
    0xA7C0,
    0xA808,
    0xA84F,
    0xA896,
    0xA8DD,
    0xA924,
    0xA96B,
    0xA9B2,
    0xA9F9,
    0xAA40,
    0xAA87,
    0xAACF,
    0xAB16,
    0xAB5D,
    0xABA4,
    0xABEB,
    0xAC32,
    0xAC79
#endif
#if XTAL52M
    0xA77A, //2412MHz
    0xA7D3, //2417MHz
    0xA82C, //2422MHz
    0xA885, //2427MHz
    0xA8DE, //2432MHz
    0xA937, //2437MHz
    0xA990, //2442MHz
    0xA9E8, //2447MHz
    0xAA41, //2452MHz
    0xAA9A, //2457MHz
    0xAAF3, //2462MHz
    0xAB4C, //2467MHz
    0xABA5, //2472MHz
    0xAC7A  //2484MHz
#endif
};

static uint16_t channel_cnt_range[3] = {
#if XTAL40M
    0xa6a0,
    0xa6e0,
    0xace0,
#endif
};

static uint16_t fcal_div = 0x855;

static int32_t rx_notch_para_40M[NUM_WIFI_CHANNELS][2] = {
    {0,0*1000*1000}, //2412MHz
    {0,0*1000*1000}, //2417MHz
    {0,0*1000*1000}, //2422MHz
    {0,0*1000*1000}, //2427MHz
    {1,8*1000*1000}, //2432MHz
    {1,3*1000*1000}, //2437MHz
    {1,-2*1000*1000}, //2442MHz
    {1,-7*1000*1000}, //2447MHz
    {0,0*1000*1000}, //2452MHz
    {0,0*1000*1000}, //2457MHz
    {0,0*1000*1000}, //2462MHz
    {0,0*1000*1000}, //2467MHz
    {1,8*1000*1000}, //2472MHz
    {1,-4*1000*1000}  //2484MHz    
};

static tx_pwr_index data;
tx_pwr_index* tp_index = &data;
static int32_t tx_pwr_table[E_RF_TXPWR_TBL_CNT][7] = {
    {0,1,7,0x14,32,4,233},
    {0,1,7,0x14,32,0,223},
    {0,1,7,0xe,32,4,212},
    {0,1,7,0xe,32,0,202},
    {0,1,7,0xa,32,4+1,188+3},
    {0,1,7,0xa,32,0+1,178+3},
    {0,1,7,8,32,4-1,173-3},
    {0,1,7,8,32,0-1,163-3},
    {0,1,7,6,32,4-1,153-3},
    {0,1,7,6,32,0-1,143-3},
    {0,1,6,5,32,4+1,129+3},
    {0,1,6,5,32,0+1,119+3},
    {0,1,6,4,32,4-1,114-3},
    {0,1,6,4,32,0-1,104-3},
    {0,1,5,4,32,0,91},
    {0,1,5,4,32,-4,81},
};
static int32_t tx_pwr_os=0;                     // channel offset
static int32_t tx_pwr_os_temperature = 0;       // temperature offset
static int32_t tx_pwr_ch_os[NUM_WIFI_CHANNELS] = {
    // 13, //2412MHz
    // 15, //2417MHz
    // 16, //2422MHz
    // 18, //2427MHz
    // 21, //2432MHz
    // 23, //2437MHz
    // 0, //2442MHz
    // 0, //2447MHz
    // 5, //2452MHz
    // 7, //2457MHz
    // 9, //2462MHz
    // 11, //2467MHz
    // 14, //2472MHz
    // 18  //2484MHz
    -16, //2412MHz
    -13, //2417MHz
    -10, //2422MHz
    -8, //2427MHz
    -5, //2432MHz
    -2, //2437MHz
    0, //2442MHz
    2, //2447MHz
    5, //2452MHz
    7, //2457MHz
    9, //2462MHz
    11, //2467MHz
    14, //2472MHz
    18  //2484MHz        
};

static uint32_t txcal_para[E_RF_TXCAL_GAIN_CNT][4] = {
    // {0,1,7,7,0,2,1,0},
    // {0,1,7,7,0,2,2,0},
    // {0,1,7,7,0,2,0,0x40},
    // {0,1,7,7,0,2,0,0x100},
    // 
    // {0,1,7,0xd,0,2,14,0x100},
    // {0,1,7,0xb,0,2,9,0x100},
    // {0,1,7,9,0,2,5,0x100},
    // {0,1,7,8,0,2,4,0x100},
    // {0,1,7,7,0,2,3,0x100},
    // {0,1,7,6,0,2,1,0x100},
    // {0,1,7,5,0,2,0,0x100},
    // {0,1,7,4,0,2,0,0x130},
    // 
    // {0,2,0,0x110},
    // {0,4,0,0xf0},
    // {0,4,0,0x130},
    // {0,4,0,0x160},
    // {0,4,0,0x1a0},
    // {0,4,0,0x1a0},
    // {0,4,0,0x200},
    // {0,4,0,0x200},
    // 
    {0,2,6,0x100},
    {0,2,1,0x100},
    {0,2,0,0x130},
    {0,4,0,0xf0},
    {0,4,0,0x130},
    {0,4,0,0x160},
    {0,4,0,0x1a0},
    {0,4,0,0x1a0},
};

static uint16_t channel_cw_table[NUM_CHANNELS];
static uint16_t channel_cnt_opt_table[CNT_TIMES];
/*@} end of group RF_PRIVATE_Private_Variables */

/** @defgroup  RF_PRIVATE_Global_Variables
 *  @{
 */

/*@} end of group RF_PRIVATE_Global_Variables */

/** @defgroup  RF_PRIVATE_Private_Fun_Declaration
 *  @{
 */

/*@} end of group RF_PRIVATE_Private_Fun_Declaration */

/** @defgroup  RF_PRIVATE_Private_Functions
 *  @{
 */

/*@} end of group RF_PRIVATE_Private_Functions */

/** @defgroup  RF_PRIVATE_Public_Functions
 *  @{
 */
// uint32_t rf_pri_channel_freq_to_index(uint32_t freq)
// {
//     if (freq == 2412) return E_RF_CHANNEL_2412M;
//     if (freq == 2417) return E_RF_CHANNEL_2417M;
//     if (freq == 2422) return E_RF_CHANNEL_2422M;
//     if (freq == 2427) return E_RF_CHANNEL_2427M;
//     if (freq == 2432) return E_RF_CHANNEL_2432M;
//     if (freq == 2437) return E_RF_CHANNEL_2437M;
//     if (freq == 2442) return E_RF_CHANNEL_2442M;
//     if (freq == 2447) return E_RF_CHANNEL_2447M;
//     if (freq == 2452) return E_RF_CHANNEL_2452M;
//     if (freq == 2457) return E_RF_CHANNEL_2457M;
//     if (freq == 2462) return E_RF_CHANNEL_2462M;
//     if (freq == 2467) return E_RF_CHANNEL_2467M;
//     if (freq == 2472) return E_RF_CHANNEL_2472M;
//     if (freq == 2484) return E_RF_CHANNEL_2484M;
//     if (freq == 2505) return E_RF_CHANNEL_2505P6M;
//     return E_RF_CHANNEL_2412M;
// }

static void rf_pri_wait_us(uint32_t us)
{
	BL602_Delay_US(us);
}

static void rf_pri_wait_ms(uint32_t ms)
{
    BL602_Delay_MS(ms);
}

static void rf_pri_set_gain_table_regs(){
    uint32_t tmpVal = 0;
    tmpVal=BL_RD_REG(RF_BASE,RF_TBB_GAIN_INDEX1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL0_GC_TBB,tx_pwr_table[14][3]);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL0_GC_TMX,tx_pwr_table[14][2]);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL0_DAC_BIAS_SEL,tx_pwr_table[14][1]);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL0_GC_TBB_BOOST,tx_pwr_table[14][0]); 
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL1_GC_TBB,tx_pwr_table[12][3]);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL1_GC_TMX,tx_pwr_table[12][2]);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL1_DAC_BIAS_SEL,tx_pwr_table[12][1]);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL1_GC_TBB_BOOST,tx_pwr_table[12][0]);       
    BL_WR_REG(RF_BASE,RF_TBB_GAIN_INDEX1,tmpVal);
    tmpVal=BL_RD_REG(RF_BASE,RF_TBB_GAIN_INDEX2);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL2_GC_TBB,tx_pwr_table[10][3]);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL2_GC_TMX,tx_pwr_table[10][2]);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL2_DAC_BIAS_SEL,tx_pwr_table[10][1]);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL2_GC_TBB_BOOST,tx_pwr_table[10][0]); 
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL3_GC_TBB,tx_pwr_table[8][3]);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL3_GC_TMX,tx_pwr_table[8][2]);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL3_DAC_BIAS_SEL,tx_pwr_table[8][1]);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL3_GC_TBB_BOOST,tx_pwr_table[8][0]);       
    BL_WR_REG(RF_BASE,RF_TBB_GAIN_INDEX2,tmpVal); 
    tmpVal=BL_RD_REG(RF_BASE,RF_TBB_GAIN_INDEX3);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL4_GC_TBB,tx_pwr_table[6][3]);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL4_GC_TMX,tx_pwr_table[6][2]);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL4_DAC_BIAS_SEL,tx_pwr_table[6][1]);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL4_GC_TBB_BOOST,tx_pwr_table[6][0]); 
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL5_GC_TBB,tx_pwr_table[4][3]);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL5_GC_TMX,tx_pwr_table[4][2]);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL5_DAC_BIAS_SEL,tx_pwr_table[4][1]);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL5_GC_TBB_BOOST,tx_pwr_table[4][0]);       
    BL_WR_REG(RF_BASE,RF_TBB_GAIN_INDEX3,tmpVal);  
    tmpVal=BL_RD_REG(RF_BASE,RF_TBB_GAIN_INDEX4);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL6_GC_TBB,tx_pwr_table[2][3]);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL6_GC_TMX,tx_pwr_table[2][2]);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL6_DAC_BIAS_SEL,tx_pwr_table[2][1]);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL6_GC_TBB_BOOST,tx_pwr_table[2][0]); 
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL7_GC_TBB,tx_pwr_table[0][3]);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL7_GC_TMX,tx_pwr_table[0][2]);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL7_DAC_BIAS_SEL,tx_pwr_table[0][1]);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GAIN_CTRL7_GC_TBB_BOOST,tx_pwr_table[0][0]);       
    BL_WR_REG(RF_BASE,RF_TBB_GAIN_INDEX4,tmpVal);         
}

static void rf_pri_fixed_val_regs(){
    uint32_t tmpVal = 0;   

    tmpVal=BL_RD_REG(AON_BASE,AON_DCDC18_TOP_0);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,AON_DCDC18_VPFM_AON,0x3);
    BL_WR_REG(AON_BASE,AON_DCDC18_TOP_0,tmpVal);
    tmpVal=BL_RD_REG(HBN_BASE,HBN_GLB);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,HBN_SW_LDO11_RT_VOUT_SEL,0x8);
    BL_WR_REG(HBN_BASE,HBN_GLB,tmpVal);

    tmpVal=BL_RD_REG(RF_BASE,RF_PUCR1);
    tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_SFREG);
    tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_ADDA_LDO);
    BL_WR_REG(RF_BASE,RF_PUCR1,tmpVal);

    tmpVal=BL_RD_REG(AON_BASE,AON_XTAL_CFG);
    tmpVal=BL_SET_REG_BIT(tmpVal,AON_XTAL_CAPCODE_EXTRA_AON);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,AON_XTAL_CAPCODE_IN_AON,50);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,AON_XTAL_CAPCODE_OUT_AON,50);
    BL_WR_REG(AON_BASE,AON_XTAL_CFG,tmpVal);
    if(!init_fast){
        rf_pri_wait_ms(10);
    }

    tmpVal=BL_RD_REG(RF_BASE,RF_PA1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_PA_VBCAS,4);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_PA_VBCORE,0xa);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_PA_IET,3);//5);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_PA_IAQ,2);
    BL_WR_REG(RF_BASE,RF_PA1,tmpVal);    
    tmpVal=BL_RD_REG(RF_BASE,RF_PA_REG_CTRL_HW1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_PA_VBCAS_11N,4);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_PA_VBCORE_11N,0xa);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_PA_IET_11N,3);//5);    
    BL_WR_REG(RF_BASE,RF_PA_REG_CTRL_HW1,tmpVal); 
    tmpVal=BL_RD_REG(RF_BASE,RF_PA_REG_CTRL_HW2);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_PA_VBCAS_11G,4);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_PA_VBCORE_11G,0xa);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_PA_IET_11G,3);//5); 
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_PA_VBCAS_11B,4);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_PA_VBCORE_11B,0xa);//0xf);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_PA_IET_11B,3);//7);        
    BL_WR_REG(RF_BASE,RF_PA_REG_CTRL_HW2,tmpVal);        

    tmpVal=BL_RD_REG(RF_BASE,RF_ADDA2);
    tmpVal=BL_SET_REG_BIT(tmpVal,RF_ADC_GT_RM);
    BL_WR_REG(RF_BASE,RF_ADDA2,tmpVal);

    tmpVal=BL_RD_REG(RF_BASE,RF_FBDV);
    tmpVal=BL_CLR_REG_BIT(tmpVal,RF_LO_FBDV_HALFSTEP_EN);
    BL_WR_REG(RF_BASE,RF_FBDV,tmpVal);
    tmpVal=BL_RD_REG(RF_BASE,RF_LO_REG_CTRL_HW1);
    tmpVal=BL_CLR_REG_BIT(tmpVal,RF_LO_FBDV_HALFSTEP_EN_TX);
    tmpVal=BL_CLR_REG_BIT(tmpVal,RF_LO_FBDV_HALFSTEP_EN_RX);
    BL_WR_REG(RF_BASE,RF_LO_REG_CTRL_HW1,tmpVal);

    tmpVal=BL_RD_REG(RF_BASE,RF_LO_REG_CTRL_HW1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_LO_LF_RZ_TX,0);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_LO_LF_CZ_TX,3);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_LO_CP_SEL_TX,0);
    BL_WR_REG(RF_BASE,RF_LO_REG_CTRL_HW1,tmpVal);

    tmpVal=BL_RD_REG(RF_BASE,RF_PA_REG_WIFI_CTRL_HW);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_PA_HALF_ON_WIFI,0);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_PA_IB_FIX_WIFI,0);
    BL_WR_REG(RF_BASE,RF_PA_REG_WIFI_CTRL_HW,tmpVal);
}

void rf_pri_full_cal(void);
void rf_pri_init(uint8_t reset)
{
    uint32_t tmpVal = 0;

    if(init_fast){
        return;
    }

    rf_pri_fixed_val_regs();
    rf_pri_set_gain_table_regs();

    tmpVal=BL_RD_REG(PDS_BASE,PDS_CLKPLL_TOP_CTRL);
    tmpVal=BL_SET_REG_BIT(tmpVal,PDS_CLKPLL_REFCLK_SEL);
    BL_WR_REG(PDS_BASE,PDS_CLKPLL_TOP_CTRL,tmpVal);
    tmpVal=BL_RD_REG(PDS_BASE,PDS_CLKPLL_OUTPUT_EN);
    tmpVal=BL_SET_REG_BIT(tmpVal,PDS_CLKPLL_EN_48M);
    tmpVal=BL_SET_REG_BIT(tmpVal,PDS_CLKPLL_EN_80M);
    tmpVal=BL_SET_REG_BIT(tmpVal,PDS_CLKPLL_EN_96M);
    tmpVal=BL_SET_REG_BIT(tmpVal,PDS_CLKPLL_EN_120M);
    tmpVal=BL_SET_REG_BIT(tmpVal,PDS_CLKPLL_EN_160M);
    tmpVal=BL_SET_REG_BIT(tmpVal,PDS_CLKPLL_EN_192M);
    tmpVal=BL_SET_REG_BIT(tmpVal,PDS_CLKPLL_EN_240M);
    tmpVal=BL_SET_REG_BIT(tmpVal,PDS_CLKPLL_EN_480M);
    BL_WR_REG(PDS_BASE,PDS_CLKPLL_OUTPUT_EN,tmpVal);

    rf_pri_init_calib_mem();
    // rf_pri_full_cal();

    // rf_pri_config_bandwidth(E_RF_BW_20M);
    // set OR TBD

    // run calibration (skip if restored from sleep)
    if (rf_calib_data->inited && !reset) {
        // TBD: set calib registers 
    }
    else {
        rf_pri_full_cal();
        rf_calib_data->inited = 1;
    }
}

void rf_pri_init_fast(uint32_t flag)
{
    init_fast = flag;
}

#ifdef RF_IN_SDK
void rf_pri_get_notch_param(uint32_t chanfreq_MHz,uint8_t *ncf_on, int32_t *ncf_freq_Hz)
{
    uint32_t channel_idx;
    if(chanfreq_MHz > 2472){
        channel_idx = 13;
    }else{
        channel_idx = (chanfreq_MHz-2412)/5;
    }

    *ncf_on = rx_notch_para_40M[channel_idx][0];
    *ncf_freq_Hz = rx_notch_para_40M[channel_idx][1];
}
#else
void rf_pri_notch_param(uint32_t chanfreq_MHz)
{
    uint32_t channel_idx;
    if(chanfreq_MHz > 2472){
        channel_idx = 13;
    }else{
        channel_idx = (chanfreq_MHz-2412)/5;
    }
    notch_p->notch_en = rx_notch_para_40M[channel_idx][0];
    notch_p->spur_freq = rx_notch_para_40M[channel_idx][1];
}
#endif

void rf_pri_update_param(uint32_t chanfreq_MHz)
{
    uint32_t tmpVal = 0;

    if(chanfreq_MHz <= 2437){
        tmpVal=BL_RD_REG(RF_BASE,RF_TMX);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TMX_CS,rf_calib_data->cal.tmx_csl);
        BL_WR_REG(RF_BASE,RF_TMX,tmpVal);
    }else{
        tmpVal=BL_RD_REG(RF_BASE,RF_TMX);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TMX_CS,rf_calib_data->cal.tmx_csh);
        BL_WR_REG(RF_BASE,RF_TMX,tmpVal);
    }

    switch(chanfreq_MHz){
        case 2412:
            tx_pwr_os=tx_pwr_ch_os[0];
            break;
        case 2417:
            tx_pwr_os=tx_pwr_ch_os[1];
            break;
        case 2422:
            tx_pwr_os=tx_pwr_ch_os[2];
            break;
        case 2427:
            tx_pwr_os=tx_pwr_ch_os[3];
            break;
        case 2432:
            tx_pwr_os=tx_pwr_ch_os[4];
            break;
        case 2437:
            tx_pwr_os=tx_pwr_ch_os[5];
            break;
        case 2442:
            tx_pwr_os=tx_pwr_ch_os[6];
            break;
        case 2447:
            tx_pwr_os=tx_pwr_ch_os[7];
            break;
        case 2452:
            tx_pwr_os=tx_pwr_ch_os[8];
            break;
        case 2457:
            tx_pwr_os=tx_pwr_ch_os[9];
            break;
        case 2462:
            tx_pwr_os=tx_pwr_ch_os[10];
            break;
        case 2467:
            tx_pwr_os=tx_pwr_ch_os[11];
            break;
        case 2472:
            tx_pwr_os=tx_pwr_ch_os[12];
            break;
        case 2484:
            tx_pwr_os=tx_pwr_ch_os[13];
            break;
        default:
            tx_pwr_os=0;
            break;
    }
}

void rf_pri_xtalfreq(uint32_t xtalfreq)
{
    // rf_pri_printf("xtalfreq=%ld\r\n",xtalfreq);
    if(xtalfreq == E_RF_XTAL_24M){
        channel_div_table[0] =  0x21638E39;
        channel_div_table[1] =  0x2171C71C;
        channel_div_table[2] =  0x21800000;
        channel_div_table[3] =  0x218E38E4;
        channel_div_table[4] =  0x219C71C7;
        channel_div_table[5] =  0x21AAAAAB;
        channel_div_table[6] =  0x21B8E38E;
        channel_div_table[7] =  0x21C71C72;
        channel_div_table[8] =  0x21D55555;
        channel_div_table[9] =  0x21E38E39;
        channel_div_table[10] = 0x21F1C71C;
        channel_div_table[11] = 0x22000000;
        channel_div_table[12] = 0x220E38E4;
        channel_div_table[13] = 0x221C71C7;
        channel_div_table[14] = 0x222AAAAB;
        channel_div_table[15] = 0x2238E38E;
        channel_div_table[16] = 0x22471C72;
        channel_div_table[17] = 0x22555555;
        channel_div_table[18] = 0x22638E39;
        channel_div_table[19] = 0x2271C71C;
        channel_div_table[20] = 0x22800000;
        // 
        channel_cnt_table[0] =  0xA6F2;
        channel_cnt_table[1] =  0xA739;
        channel_cnt_table[2] =  0xA780;
        channel_cnt_table[3] =  0xA7C7;
        channel_cnt_table[4] =  0xA80E;
        channel_cnt_table[5] =  0xA855;
        channel_cnt_table[6] =  0xA89C;
        channel_cnt_table[7] =  0xA8E4;
        channel_cnt_table[8] =  0xA92B;
        channel_cnt_table[9] =  0xA972;
        channel_cnt_table[10] = 0xA9B9;
        channel_cnt_table[11] = 0xAA00;
        channel_cnt_table[12] = 0xAA47;
        channel_cnt_table[13] = 0xAA8E;
        channel_cnt_table[14] = 0xAAD5;
        channel_cnt_table[15] = 0xAB1C;
        channel_cnt_table[16] = 0xAB64;
        channel_cnt_table[17] = 0xABAB;
        channel_cnt_table[18] = 0xABF2;
        channel_cnt_table[19] = 0xAC39;
        channel_cnt_table[20] = 0xAC80;
        //
        channel_cnt_range[0] = 0xA6A7;
        channel_cnt_range[1] = 0xA6E7;
        channel_cnt_range[2] = 0xACE7;
        // 
        fcal_div = 0x500;
        // 2400,2424,2448,2472,2496       
        rx_notch_para_40M[0][0] = 0; //2412MHz
        rx_notch_para_40M[1][0] = 1;//2417MHz
        rx_notch_para_40M[2][0] = 1;//2422MHz
        rx_notch_para_40M[3][0] = 1;//2427MHz
        rx_notch_para_40M[4][0] = 1;//2432MHz
        rx_notch_para_40M[5][0] = 0;//2437MHz
        rx_notch_para_40M[6][0] = 1; //2442MHz
        rx_notch_para_40M[7][0] = 1; //2447MHz
        rx_notch_para_40M[8][0] = 1;//2452MHz
        rx_notch_para_40M[9][0] = 1;//2457MHz
        rx_notch_para_40M[10][0] = 0;//2462MHz
        rx_notch_para_40M[11][0] = 1;//2467MHz
        rx_notch_para_40M[12][0] = 1;//2472MHz
        rx_notch_para_40M[13][0] = 0; //2484MHz
        rx_notch_para_40M[0][1] = 0*1000*1000; //2412MHz
        rx_notch_para_40M[1][1] = 7*1000*1000;//2417MHz
        rx_notch_para_40M[2][1] = 2*1000*1000;//2422MHz
        rx_notch_para_40M[3][1] = -3*1000*1000;//2427MHz
        rx_notch_para_40M[4][1] = -8*1000*1000;//2432MHz
        rx_notch_para_40M[5][1] = 0*1000*1000;//2437MHz
        rx_notch_para_40M[6][1] = 6*1000*1000; //2442MHz
        rx_notch_para_40M[7][1] = 1*1000*1000; //2447MHz
        rx_notch_para_40M[8][1] = -4*1000*1000;//2452MHz
        rx_notch_para_40M[9][1] = -9*1000*1000;//2457MHz
        rx_notch_para_40M[10][1] = 0*1000*1000;//2462MHz
        rx_notch_para_40M[11][1] = 5*1000*1000;//2467MHz
        rx_notch_para_40M[12][1] = 0*1000*1000;//2472MHz
        rx_notch_para_40M[13][1] = 0*1000*1000; //2484MHz                
    }else if(xtalfreq == E_RF_XTAL_26M){
        channel_div_table[0] =  0x1ED20D21;
        channel_div_table[1] =  0x1EDF2DF3;
        channel_div_table[2] =  0x1EEC4EC5;
        channel_div_table[3] =  0x1EF96F97;
        channel_div_table[4] =  0x1F069069;
        channel_div_table[5] =  0x1F13B13B;
        channel_div_table[6] =  0x1F20D20D;
        channel_div_table[7] =  0x1F2DF2DF;
        channel_div_table[8] =  0x1F3B13B1;
        channel_div_table[9] =  0x1F483483;
        channel_div_table[10] = 0x1F555555;
        channel_div_table[11] = 0x1F627627;
        channel_div_table[12] = 0x1F6F96F9;
        channel_div_table[13] = 0x1F7CB7CB;
        channel_div_table[14] = 0x1F89D89E;
        channel_div_table[15] = 0x1F96F970;
        channel_div_table[16] = 0x1FA41A42;
        channel_div_table[17] = 0x1FB13B14;
        channel_div_table[18] = 0x1FBE5BE6;
        channel_div_table[19] = 0x1FCB7CB8;
        channel_div_table[20] = 0x1FD89D8A;
        // 
        channel_cnt_table[0] =  0xA6FC;
        channel_cnt_table[1] =  0xA743;
        channel_cnt_table[2] =  0xA78A;
        channel_cnt_table[3] =  0xA7D1;
        channel_cnt_table[4] =  0xA819;
        channel_cnt_table[5] =  0xA860;
        channel_cnt_table[6] =  0xA8A7;
        channel_cnt_table[7] =  0xA8EE;
        channel_cnt_table[8] =  0xA935;
        channel_cnt_table[9] =  0xA97C;
        channel_cnt_table[10] = 0xA9C3;
        channel_cnt_table[11] = 0xAA0A;
        channel_cnt_table[12] = 0xAA52;
        channel_cnt_table[13] = 0xAA99;
        channel_cnt_table[14] = 0xAAE0;
        channel_cnt_table[15] = 0xAB27;
        channel_cnt_table[16] = 0xAB6E;
        channel_cnt_table[17] = 0xABB5;
        channel_cnt_table[18] = 0xABFC;
        channel_cnt_table[19] = 0xAC43;
        channel_cnt_table[20] = 0xAC8B;
        //
        channel_cnt_range[0] = 0xA6B1;
        channel_cnt_range[1] = 0xA6F1;
        channel_cnt_range[2] = 0xACF2;
        //
        fcal_div = 0x56b; 
        // 2418,2444,2470,2496       
        rx_notch_para_40M[0][0] = 1; //2412MHz
        rx_notch_para_40M[1][0] = 1;//2417MHz
        rx_notch_para_40M[2][0] = 1;//2422MHz
        rx_notch_para_40M[3][0] = 1;//2427MHz
        rx_notch_para_40M[4][0] = 0;//2432MHz
        rx_notch_para_40M[5][0] = 1;//2437MHz
        rx_notch_para_40M[6][0] = 1; //2442MHz
        rx_notch_para_40M[7][0] = 1; //2447MHz
        rx_notch_para_40M[8][0] = 1;//2452MHz
        rx_notch_para_40M[9][0] = 0;//2457MHz
        rx_notch_para_40M[10][0] = 1;//2462MHz
        rx_notch_para_40M[11][0] = 1;//2467MHz
        rx_notch_para_40M[12][0] = 1;//2472MHz
        rx_notch_para_40M[13][0] = 0; //2484MHz 
        rx_notch_para_40M[0][1] = 6*1000*1000; //2412MHz
        rx_notch_para_40M[1][1] = 1*1000*1000;//2417MHz
        rx_notch_para_40M[2][1] = -4*1000*1000;//2422MHz
        rx_notch_para_40M[3][1] = -9*1000*1000;//2427MHz
        rx_notch_para_40M[4][1] = 0*1000*1000;//2432MHz
        rx_notch_para_40M[5][1] = 7*1000*1000;//2437MHz
        rx_notch_para_40M[6][1] = 2*1000*1000; //2442MHz
        rx_notch_para_40M[7][1] = -3*1000*1000; //2447MHz
        rx_notch_para_40M[8][1] = -8*1000*1000;//2452MHz
        rx_notch_para_40M[9][1] = 0*1000*1000;//2457MHz
        rx_notch_para_40M[10][1] = 8*1000*1000;//2462MHz
        rx_notch_para_40M[11][1] = 3*1000*1000;//2467MHz
        rx_notch_para_40M[12][1] = -2*1000*1000;//2472MHz
        rx_notch_para_40M[13][1] = 0*1000*1000; //2484MHz                                                        
    }else if(xtalfreq == E_RF_XTAL_32M){
        channel_div_table[0] =  0x190AAAAB;
        channel_div_table[1] =  0x19155555;
        channel_div_table[2] =  0x19200000;
        channel_div_table[3] =  0x192AAAAB;
        channel_div_table[4] =  0x19355555;
        channel_div_table[5] =  0x19400000;
        channel_div_table[6] =  0x194AAAAB;
        channel_div_table[7] =  0x19555555;
        channel_div_table[8] =  0x19600000;
        channel_div_table[9] =  0x196AAAAB;
        channel_div_table[10] = 0x19755555;
        channel_div_table[11] = 0x19800000;
        channel_div_table[12] = 0x198AAAAB;
        channel_div_table[13] = 0x19955555;
        channel_div_table[14] = 0x19A00000;
        channel_div_table[15] = 0x19AAAAAB;
        channel_div_table[16] = 0x19B55555;
        channel_div_table[17] = 0x19C00000;
        channel_div_table[18] = 0x19CAAAAB;
        channel_div_table[19] = 0x19D55555;
        channel_div_table[20] = 0x19E00000;
        // 
        channel_cnt_table[0] =  0xA6FA;
        channel_cnt_table[1] =  0xA741;
        channel_cnt_table[2] =  0xA788;
        channel_cnt_table[3] =  0xA7D0;
        channel_cnt_table[4] =  0xA817;
        channel_cnt_table[5] =  0xA85E;
        channel_cnt_table[6] =  0xA8A5;
        channel_cnt_table[7] =  0xA8EC;
        channel_cnt_table[8] =  0xA933;
        channel_cnt_table[9] =  0xA97A;
        channel_cnt_table[10] = 0xA9C1;
        channel_cnt_table[11] = 0xAA09;
        channel_cnt_table[12] = 0xAA50;
        channel_cnt_table[13] = 0xAA97;
        channel_cnt_table[14] = 0xAADE;
        channel_cnt_table[15] = 0xAB25;
        channel_cnt_table[16] = 0xAB6C;
        channel_cnt_table[17] = 0xABB3;
        channel_cnt_table[18] = 0xABFA;
        channel_cnt_table[19] = 0xAC42;
        channel_cnt_table[20] = 0xAC89;
        //
        channel_cnt_range[0] = 0xA6AF;
        channel_cnt_range[1] = 0xA6EF;
        channel_cnt_range[2] = 0xACF0;
        // 
        fcal_div = 0x6ab;
        // 2400,2432,2464,2496       
        rx_notch_para_40M[0][0] = 0; //2412MHz
        rx_notch_para_40M[1][0] = 0;//2417MHz
        rx_notch_para_40M[2][0] = 1;//2422MHz
        rx_notch_para_40M[3][0] = 1;//2427MHz
        rx_notch_para_40M[4][0] = 1;//2432MHz
        rx_notch_para_40M[5][0] = 1;//2437MHz
        rx_notch_para_40M[6][0] = 1; //2442MHz
        rx_notch_para_40M[7][0] = 0; //2447MHz
        rx_notch_para_40M[8][0] = 0;//2452MHz
        rx_notch_para_40M[9][0] = 1;//2457MHz
        rx_notch_para_40M[10][0] = 1;//2462MHz
        rx_notch_para_40M[11][0] = 1;//2467MHz
        rx_notch_para_40M[12][0] = 1;//2472MHz
        rx_notch_para_40M[13][0] = 0; //2484MHz   
        rx_notch_para_40M[0][1] = 0*1000*1000; //2412MHz
        rx_notch_para_40M[1][1] = 0*1000*1000;//2417MHz
        rx_notch_para_40M[2][1] = 10*1000*1000;//2422MHz
        rx_notch_para_40M[3][1] = 5*1000*1000;//2427MHz
        rx_notch_para_40M[4][1] = 0*1000*1000;//2432MHz
        rx_notch_para_40M[5][1] = -5*1000*1000;//2437MHz
        rx_notch_para_40M[6][1] = -10*1000*1000; //2442MHz
        rx_notch_para_40M[7][1] = 0*1000*1000; //2447MHz
        rx_notch_para_40M[8][1] = 0*1000*1000;//2452MHz
        rx_notch_para_40M[9][1] = 7*1000*1000;//2457MHz
        rx_notch_para_40M[10][1] = 2*1000*1000;//2462MHz
        rx_notch_para_40M[11][1] = -3*1000*1000;//2467MHz
        rx_notch_para_40M[12][1] = -8*1000*1000;//2472MHz
        rx_notch_para_40M[13][1] = 0*1000*1000; //2484MHz             
    }else if(xtalfreq == E_RF_XTAL_38M4){
        channel_div_table[0] =  0x14DE38E4;
        channel_div_table[1] =  0x14E71C72;
        channel_div_table[2] =  0x14F00000;
        channel_div_table[3] =  0x14F8E38E;
        channel_div_table[4] =  0x1501C71C;
        channel_div_table[5] =  0x150AAAAB;
        channel_div_table[6] =  0x15138E39;
        channel_div_table[7] =  0x151C71C7;
        channel_div_table[8] =  0x15255555;
        channel_div_table[9] =  0x152E38E4;
        channel_div_table[10] = 0x15371C72;
        channel_div_table[11] = 0x15400000;
        channel_div_table[12] = 0x1548E38E;
        channel_div_table[13] = 0x1551C71C;
        channel_div_table[14] = 0x155AAAAB;
        channel_div_table[15] = 0x15638E39;
        channel_div_table[16] = 0x156C71C7;
        channel_div_table[17] = 0x15755555;
        channel_div_table[18] = 0x157E38E4;
        channel_div_table[19] = 0x15871C72;
        channel_div_table[20] = 0x15900000;
        // 
        channel_cnt_table[0] =  0xA6F2;
        channel_cnt_table[1] =  0xA739;
        channel_cnt_table[2] =  0xA780;
        channel_cnt_table[3] =  0xA7C7;
        channel_cnt_table[4] =  0xA80E;
        channel_cnt_table[5] =  0xA855;
        channel_cnt_table[6] =  0xA89C;
        channel_cnt_table[7] =  0xA8E4;
        channel_cnt_table[8] =  0xA92B;
        channel_cnt_table[9] =  0xA972;
        channel_cnt_table[10] = 0xA9B9;
        channel_cnt_table[11] = 0xAA00;
        channel_cnt_table[12] = 0xAA47;
        channel_cnt_table[13] = 0xAA8E;
        channel_cnt_table[14] = 0xAAD5;
        channel_cnt_table[15] = 0xAB1C;
        channel_cnt_table[16] = 0xAB64;
        channel_cnt_table[17] = 0xABAB;
        channel_cnt_table[18] = 0xABF2;
        channel_cnt_table[19] = 0xAC39;
        channel_cnt_table[20] = 0xAC80;
        //
        channel_cnt_range[0] = 0xA6A7;
        channel_cnt_range[1] = 0xA6E7;
        channel_cnt_range[2] = 0xACE7; 
        //
        fcal_div = 0x800; 
        // 2380.8,2419.2,2457.6‬,2496       
        rx_notch_para_40M[0][0] = 1; //2412MHz
        rx_notch_para_40M[1][0] = 1;//2417MHz
        rx_notch_para_40M[2][0] = 1;//2422MHz
        rx_notch_para_40M[3][0] = 1;//2427MHz
        rx_notch_para_40M[4][0] = 0;//2432MHz
        rx_notch_para_40M[5][0] = 0;//2437MHz
        rx_notch_para_40M[6][0] = 0; //2442MHz
        rx_notch_para_40M[7][0] = 0; //2447MHz
        rx_notch_para_40M[8][0] = 1;//2452MHz
        rx_notch_para_40M[9][0] = 1;//2457MHz
        rx_notch_para_40M[10][0] = 1;//2462MHz
        rx_notch_para_40M[11][0] = 1;//2467MHz
        rx_notch_para_40M[12][0] = 0;//2472MHz
        rx_notch_para_40M[13][0] = 0; //2484MHz  
        rx_notch_para_40M[0][1] = 7200*1000; //2412MHz
        rx_notch_para_40M[1][1] = 2200*1000;//2417MHz
        rx_notch_para_40M[2][1] = -2800*1000;//2422MHz
        rx_notch_para_40M[3][1] = -7800*1000;//2427MHz
        rx_notch_para_40M[4][1] = 0*1000*1000;//2432MHz
        rx_notch_para_40M[5][1] = 0*1000*1000;//2437MHz
        rx_notch_para_40M[6][1] = 0*1000*1000; //2442MHz
        rx_notch_para_40M[7][1] = 0*1000*1000; //2447MHz
        rx_notch_para_40M[8][1] = 5600*1000;//2452MHz
        rx_notch_para_40M[9][1] = 600*1000;//2457MHz
        rx_notch_para_40M[10][1] = -4400*1000;//2462MHz
        rx_notch_para_40M[11][1] = -9400*1000;//2467MHz
        rx_notch_para_40M[12][1] = 0*1000*1000;//2472MHz
        rx_notch_para_40M[13][1] = 0*1000*1000; //2484MHz                   
    }else if(xtalfreq == E_RF_XTAL_40M){
        channel_div_table[0] =  0x14088889;
        channel_div_table[1] =  0x14111111;
        channel_div_table[2] =  0x1419999A;
        channel_div_table[3] =  0x14222222;
        channel_div_table[4] =  0x142AAAAB;
        channel_div_table[5] =  0x14333333;
        channel_div_table[6] =  0x143BBBBC;
        channel_div_table[7] =  0x14444444;
        channel_div_table[8] =  0x144CCCCD;
        channel_div_table[9] =  0x14555555;
        channel_div_table[10] = 0x145DDDDE;
        channel_div_table[11] = 0x14666666;
        channel_div_table[12] = 0x146EEEEF;
        channel_div_table[13] = 0x14777777;
        channel_div_table[14] = 0x14800000;
        channel_div_table[15] = 0x14888889;
        channel_div_table[16] = 0x14911111;
        channel_div_table[17] = 0x1499999A;
        channel_div_table[18] = 0x14A22222;
        channel_div_table[19] = 0x14AAAAAB;
        channel_div_table[20] = 0x14B33333;
        // 
        channel_cnt_table[0] =  0xA6EB;
        channel_cnt_table[1] =  0xA732;
        channel_cnt_table[2] =  0xA779;
        channel_cnt_table[3] =  0xA7C0;
        channel_cnt_table[4] =  0xA808;
        channel_cnt_table[5] =  0xA84F;
        channel_cnt_table[6] =  0xA896;
        channel_cnt_table[7] =  0xA8DD;
        channel_cnt_table[8] =  0xA924;
        channel_cnt_table[9] =  0xA96B;
        channel_cnt_table[10] = 0xA9B2;
        channel_cnt_table[11] = 0xA9F9;
        channel_cnt_table[12] = 0xAA40;
        channel_cnt_table[13] = 0xAA87;
        channel_cnt_table[14] = 0xAACF;
        channel_cnt_table[15] = 0xAB16;
        channel_cnt_table[16] = 0xAB5D;
        channel_cnt_table[17] = 0xABA4;
        channel_cnt_table[18] = 0xABEB;
        channel_cnt_table[19] = 0xAC32;
        channel_cnt_table[20] = 0xAC79;
        //
        channel_cnt_range[0] = 0xa6a0;
        channel_cnt_range[1] = 0xa6e0;
        channel_cnt_range[2] = 0xace0;
        //
        fcal_div = 0x855;
        // 2400,2440,2480       
        rx_notch_para_40M[0][0] = 0; //2412MHz
        rx_notch_para_40M[1][0] = 0;//2417MHz
        rx_notch_para_40M[2][0] = 0;//2422MHz
        rx_notch_para_40M[3][0] = 0;//2427MHz
        rx_notch_para_40M[4][0] = 1;//2432MHz
        rx_notch_para_40M[5][0] = 1;//2437MHz
        rx_notch_para_40M[6][0] = 1; //2442MHz
        rx_notch_para_40M[7][0] = 1; //2447MHz
        rx_notch_para_40M[8][0] = 0;//2452MHz
        rx_notch_para_40M[9][0] = 0;//2457MHz
        rx_notch_para_40M[10][0] = 0;//2462MHz
        rx_notch_para_40M[11][0] = 0;//2467MHz
        rx_notch_para_40M[12][0] = 1;//2472MHz
        rx_notch_para_40M[13][0] = 1; //2484MHz     
        rx_notch_para_40M[0][1] = 0*1000*1000; //2412MHz
        rx_notch_para_40M[1][1] = 0*1000*1000;//2417MHz
        rx_notch_para_40M[2][1] = 0*1000*1000;//2422MHz
        rx_notch_para_40M[3][1] = 0*1000*1000;//2427MHz
        rx_notch_para_40M[4][1] = 8*1000*1000;//2432MHz
        rx_notch_para_40M[5][1] = 3*1000*1000;//2437MHz
        rx_notch_para_40M[6][1] = -2*1000*1000; //2442MHz
        rx_notch_para_40M[7][1] = -7*1000*1000; //2447MHz
        rx_notch_para_40M[8][1] = 0*1000*1000;//2452MHz
        rx_notch_para_40M[9][1] = 0*1000*1000;//2457MHz
        rx_notch_para_40M[10][1] = 0*1000*1000;//2462MHz
        rx_notch_para_40M[11][1] = 0*1000*1000;//2467MHz
        rx_notch_para_40M[12][1] = 8*1000*1000;//2472MHz
        rx_notch_para_40M[13][1] = -4*1000*1000; //2484MHz             
    }else if(xtalfreq == E_RF_XTAL_52M){
        channel_div_table[0] =  0xF690690;
        channel_div_table[1] =  0xF6F96F9;
        channel_div_table[2] =  0xF762762;
        channel_div_table[3] =  0xF7CB7CB;
        channel_div_table[4] =  0xF834835;
        channel_div_table[5] =  0xF89D89E;
        channel_div_table[6] =  0xF906907;
        channel_div_table[7] =  0xF96F970;
        channel_div_table[8] =  0xF9D89D9;
        channel_div_table[9] =  0xFA41A42;
        channel_div_table[10] = 0xFAAAAAB;
        channel_div_table[11] = 0xFB13B14;
        channel_div_table[12] = 0xFB7CB7D;
        channel_div_table[13] = 0xFBE5BE6;
        channel_div_table[14] = 0xFC4EC4F;
        channel_div_table[15] = 0xFCB7CB8;
        channel_div_table[16] = 0xFD20D21;
        channel_div_table[17] = 0xFD89D8A;
        channel_div_table[18] = 0xFDF2DF3;
        channel_div_table[19] = 0xFE5BE5C;
        channel_div_table[20] = 0xFEC4EC5;
        // 
        channel_cnt_table[0] =  0xA6ED;
        channel_cnt_table[1] =  0xA734;
        channel_cnt_table[2] =  0xA77B;
        channel_cnt_table[3] =  0xA7C2;
        channel_cnt_table[4] =  0xA809;
        channel_cnt_table[5] =  0xA850;
        channel_cnt_table[6] =  0xA897;
        channel_cnt_table[7] =  0xA8DE;
        channel_cnt_table[8] =  0xA925;
        channel_cnt_table[9] =  0xA96D;
        channel_cnt_table[10] = 0xA9B4;
        channel_cnt_table[11] = 0xA9FB;
        channel_cnt_table[12] = 0xAA42;
        channel_cnt_table[13] = 0xAA89;
        channel_cnt_table[14] = 0xAAD0;
        channel_cnt_table[15] = 0xAB17;
        channel_cnt_table[16] = 0xAB5E;
        channel_cnt_table[17] = 0xABA5;
        channel_cnt_table[18] = 0xABEC;
        channel_cnt_table[19] = 0xAC34;
        channel_cnt_table[20] = 0xAC7B;
        //
        channel_cnt_range[0] = 0xA6A2;
        channel_cnt_range[1] = 0xA6E2;
        channel_cnt_range[2] = 0xACE2; 
        //
        fcal_div = 0xad5;  
        // 2392,2444,2496       
        rx_notch_para_40M[0][0] = 0; //2412MHz
        rx_notch_para_40M[1][0] = 0;//2417MHz
        rx_notch_para_40M[2][0] = 0;//2422MHz
        rx_notch_para_40M[3][0] = 0;//2427MHz
        rx_notch_para_40M[4][0] = 0;//2432MHz
        rx_notch_para_40M[5][0] = 1;//2437MHz
        rx_notch_para_40M[6][0] = 1; //2442MHz
        rx_notch_para_40M[7][0] = 1; //2447MHz
        rx_notch_para_40M[8][0] = 1;//2452MHz
        rx_notch_para_40M[9][0] = 0;//2457MHz
        rx_notch_para_40M[10][0] = 0;//2462MHz
        rx_notch_para_40M[11][0] = 0;//2467MHz
        rx_notch_para_40M[12][0] = 0;//2472MHz
        rx_notch_para_40M[13][0] = 0; //2484MHz
        rx_notch_para_40M[0][1] = 0*1000*1000; //2412MHz
        rx_notch_para_40M[1][1] = 0*1000*1000;//2417MHz
        rx_notch_para_40M[2][1] = 0*1000*1000;//2422MHz
        rx_notch_para_40M[3][1] = 0*1000*1000;//2427MHz
        rx_notch_para_40M[4][1] = 0*1000*1000;//2432MHz
        rx_notch_para_40M[5][1] = 7*1000*1000;//2437MHz
        rx_notch_para_40M[6][1] = 2*1000*1000; //2442MHz
        rx_notch_para_40M[7][1] = -3*1000*1000; //2447MHz
        rx_notch_para_40M[8][1] = -8*1000*1000;//2452MHz
        rx_notch_para_40M[9][1] = 0*1000*1000;//2457MHz
        rx_notch_para_40M[10][1] = 0*1000*1000;//2462MHz
        rx_notch_para_40M[11][1] = 0*1000*1000;//2467MHz
        rx_notch_para_40M[12][1] = 0*1000*1000;//2472MHz
        rx_notch_para_40M[13][1] = 0*1000*1000; //2484MHz                   
    }
}

uint32_t rf_pri_get_vco_freq_cw(uint32_t chanfreq_MHz)
{
    int32_t k = (int32_t)((chanfreq_MHz-2404)/4 + 0.5);
    k = k < 0 ? 0 : k;
    k = k > NUM_CHANNELS-1 ? NUM_CHANNELS-1 : k;
    return rf_calib_data->lo[k].fcal;
}

uint32_t rf_pri_get_vco_idac_cw(uint32_t chanfreq_MHz)
{
    int32_t k = (int32_t)((chanfreq_MHz-2404)/4 + 0.5);
    k = k < 0 ? 0 : k;
    k = k > NUM_CHANNELS-1 ? NUM_CHANNELS-1 : k;
    return rf_calib_data->lo[k].acal;
}

void rf_pri_update_txgain_tempos(int16_t tempos)
{
    tx_pwr_os_temperature = tempos;
}

int32_t rf_pri_get_txgain_max()
{
    return tx_pwr_table[0][6] + tx_pwr_os + tx_pwr_os_temperature;
}

int32_t rf_pri_get_txgain_min()
{
    return tx_pwr_table[15][6] + tx_pwr_os + tx_pwr_os_temperature;
}

uint32_t rf_pri_get_txgain_index(int32_t pwr,uint32_t mode)  
{
    uint32_t i=0;
    pwr=pwr+tx_pwr_os;

    if(mode==E_RF_MODE_11B){
        pwr=pwr-33;
    }

    for(i=0;i<E_RF_TXPWR_TBL_CNT;i++){
        if(pwr>=tx_pwr_table[i][6]){
            break;
        }
    }

    return (i == E_RF_TXPWR_TBL_CNT) ? i - 1 : i;
}

void rf_pri_query_txgain_table(uint32_t index, uint32_t *rfg_index, uint32_t *dg)
{
    index = index > E_RF_TXPWR_TBL_CNT - 1 ? E_RF_TXPWR_TBL_CNT - 1 : index;
    *rfg_index = 7-(index >> 1);
    *dg = tx_pwr_table[index][5];
}

#if 0
void rf_pri_get_txgain_settings(int32_t pwr,uint32_t mode,uint32_t* rfg, uint32_t *dg)  
{
    uint32_t i;
    pwr=pwr+tx_pwr_os;

    if(mode==E_RF_MODE_11B){
        pwr=pwr-33;
    }

    for(i=0;i<E_RF_TXPWR_TBL_CNT;i++){
        if(pwr>=tx_pwr_table[i][6]){
            break;
        }
    }
    if(i==0){
        tp_index->index=i;
        tp_index->dvga=tx_pwr_table[i][5];
    }else if(i==E_RF_TXPWR_TBL_CNT){
        tp_index->index=i-1;
        tp_index->dvga=tx_pwr_table[i-1][5];
    }else{
        if((tx_pwr_table[i-1][6]-pwr)<(pwr-tx_pwr_table[i][6])){
            tp_index->index=i-1;
            tp_index->dvga=tx_pwr_table[i-1][5];            
        }else{
            tp_index->index=i;
            tp_index->dvga=tx_pwr_table[i][5];
        }
    }

    // if(mode==E_RF_MODE_11B){
    //     tp_index->dvga=tp_index->dvga+0;
    // }

    *dg = tp_index->dvga;
    *rfg = tp_index->index >> 1;
}
#endif

static void rf_pri_config_channel(uint32_t channel_index)
{
    uint32_t tmpVal = 0;
    // power up LO
    // tmpVal=BL_RD_REG(RF_BASE,RF_PUCR1);
    // tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_VCO);
    // tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_FBDV);
    // tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_PFD);
    // tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_OSMX);
    // BL_WR_REG(RF_BASE,RF_PUCR1,tmpVal);

    // set channel
    // rf_pri_printf("fcal=%d,acal=%d",rf_calib_data->lo[channel_index].fcal,rf_calib_data->lo[channel_index].acal);
    tmpVal=BL_RD_REG(RF_BASE,RF_VCO1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_LO_VCO_FREQ_CW,rf_calib_data->lo[channel_index].fcal);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_LO_VCO_IDAC_CW,rf_calib_data->lo[channel_index].acal);
    BL_WR_REG(RF_BASE,RF_VCO1,tmpVal);
    tmpVal=BL_RD_REG(RF_BASE,RF_LODIST);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_LO_OSMX_CAP,rf_calib_data->lo[channel_index].fcal >> 4);
    BL_WR_REG(RF_BASE,RF_LODIST,tmpVal);
    tmpVal=BL_RD_REG(RF_BASE,RF_SDM2);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_LO_SDMIN,channel_div_table[channel_index]);
    BL_WR_REG(RF_BASE,RF_SDM2,tmpVal);
    tmpVal=BL_RD_REG(RF_BASE,RF_SDM1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_LO_SDM_BYPASS,0);
    BL_WR_REG(RF_BASE,RF_SDM1,tmpVal);

    // wait channel lock
    // tmpVal=BL_RD_REG(RF_BASE,RFCTRL_HW_EN);    
    // tmpVal=BL_CLR_REG_BIT(tmpVal,RF_LO_CTRL_HW);
    // BL_WR_REG(RF_BASE,RFCTRL_HW_EN,tmpVal);
    while (1) {
        tmpVal=BL_RD_REG(RF_BASE,RF_FBDV);
        tmpVal=BL_SET_REG_BIT(tmpVal,RF_LO_FBDV_RST);
        BL_WR_REG(RF_BASE,RF_FBDV,tmpVal);
        rf_pri_wait_us(10);
        tmpVal=BL_RD_REG(RF_BASE,RF_FBDV);
        tmpVal=BL_CLR_REG_BIT(tmpVal,RF_LO_FBDV_RST);
        BL_WR_REG(RF_BASE,RF_FBDV,tmpVal);
        rf_pri_wait_us(50);
        tmpVal=BL_RD_REG(RF_BASE,RF_PFDCP);
        tmpVal=BL_SET_REG_BIT(tmpVal,RF_LO_PFD_RST_CSD);
        BL_WR_REG(RF_BASE,RF_PFDCP,tmpVal);
        rf_pri_wait_us(10);
        tmpVal=BL_RD_REG(RF_BASE,RF_PFDCP);
        tmpVal=BL_CLR_REG_BIT(tmpVal,RF_LO_PFD_RST_CSD);
        BL_WR_REG(RF_BASE,RF_PFDCP,tmpVal);
        rf_pri_wait_us(50);

        tmpVal=BL_RD_REG(RF_BASE,RF_LO);
        if (BL_GET_REG_BITS_VAL(tmpVal,RF_LO_SLIPPED_DN) || BL_GET_REG_BITS_VAL(tmpVal,RF_LO_SLIPPED_UP)) {
            rf_pri_printe(".");
        }
        else {
            rf_pri_printf("LO locked %ld %ld\r\n",channel_index,(uint32_t)(rf_calib_data->lo[channel_index].fcal));
            // tmpVal=BL_RD_REG(RF_BASE,RFCTRL_HW_EN);    
            // tmpVal=BL_SET_REG_BIT(tmpVal,RF_LO_CTRL_HW);
            // BL_WR_REG(RF_BASE,RFCTRL_HW_EN,tmpVal);
            break;
        }
    }
}

static void rf_pri_manu_pu(uint32_t mode)
{
    uint32_t tmpVal = 0;
    tmpVal=BL_RD_REG(RF_BASE,RF_FSM_CTRL_HW);
    tmpVal=BL_CLR_REG_BIT(tmpVal,RF_FSM_CTRL_EN);
    BL_WR_REG(RF_BASE,RF_FSM_CTRL_HW,tmpVal);
    BL_WR_REG(RF_BASE,RFCTRL_HW_EN,0);
    
    switch (mode) {
        case E_RF_MODE_LO_FCAL:
        case E_RF_MODE_LO_ACAL:
            tmpVal=BL_RD_REG(RF_BASE,RF_PUCR1);           
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_LNA);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_RMXGM);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_RMX);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_RBB);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_ADC);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_ADC_CLK_EN);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_PKDET);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_ROSDAC);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_PWRMX);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_PA);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_TMX);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_TBB);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_DAC);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_RXBUF);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_TXBUF);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_TRSW_EN);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_TOSDAC);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_OSMX);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_PFD);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_FBDV);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_VCO);  
            BL_WR_REG(RF_BASE,RF_PUCR1,tmpVal);                                                 
            break;
        case E_RF_MODE_ROSCAL:
            tmpVal=BL_RD_REG(RF_BASE,RF_PUCR1);           
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_LNA);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_RMXGM);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_RMX);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_RBB);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_ADC);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_ADC_CLK_EN);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_PKDET);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_ROSDAC);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_PWRMX);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_PA);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_TMX);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_TBB);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_DAC);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_RXBUF);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_TXBUF);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_TRSW_EN);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_TOSDAC);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_OSMX);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_PFD);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_FBDV);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_VCO);  
            BL_WR_REG(RF_BASE,RF_PUCR1,tmpVal);                                                 
            break;             
        case E_RF_MODE_RCCAL:
            tmpVal=BL_RD_REG(RF_BASE,RF_PUCR1);           
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_LNA);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_RMXGM);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_RMX);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_RBB);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_ADC);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_ADC_CLK_EN);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_PKDET);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_ROSDAC);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_PWRMX);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_PA);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_TMX);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_TBB);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_DAC);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_RXBUF);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_TXBUF);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_TRSW_EN);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_TOSDAC);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_OSMX);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_PFD);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_FBDV);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_VCO);  
            BL_WR_REG(RF_BASE,RF_PUCR1,tmpVal);                                                 
            break; 
        case E_RF_MODE_TXCAL:
            tmpVal=BL_RD_REG(RF_BASE,RF_PUCR1);           
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_LNA);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_RMXGM);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_RMX);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_RBB);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_ADC);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_ADC_CLK_EN);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_PKDET);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_ROSDAC);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_PWRMX);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_PA);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_TMX);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_TBB);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_DAC);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_RXBUF);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_TXBUF);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_TRSW_EN);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_TOSDAC);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_OSMX);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_PFD);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_FBDV);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_VCO);  
            BL_WR_REG(RF_BASE,RF_PUCR1,tmpVal);                                                 
            break;                        
        case E_RF_MODE_IDLE:
        default:
            tmpVal=BL_RD_REG(RF_BASE,RF_PUCR1);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_LNA);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_RMXGM);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_RMX);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_RBB);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_ADC);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_ADC_CLK_EN);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_PKDET);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_ROSDAC);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_PWRMX);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_PA);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_TMX);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_TBB);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_DAC);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_RXBUF);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_TXBUF);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_TRSW_EN);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_PU_TOSDAC);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_OSMX);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_PFD);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_FBDV);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_PU_VCO);            
            BL_WR_REG(RF_BASE,RF_PUCR1,tmpVal);
            break;
    }
}

static void rf_pri_config_mode(uint32_t mode)
{
    rf_pri_manu_pu(mode);
}

/****************************************************************************//**
 * @brief  RF set auto gain
 *
 * @param  None
 *
 * @return None
 *
*******************************************************************************/
static void rf_pri_auto_gain(void)
{
    uint32_t tmpVal = 0;

    tmpVal = BL_RD_REG(RF_BASE,RFCTRL_HW_EN);
    /* Because RF_RX_GAIN_CTRL_HW is only one bit, we can use BL_SET_REG_BIT
     * or BL_CLR_REG_BIT */
    tmpVal=BL_SET_REG_BIT(tmpVal,RF_RX_GAIN_CTRL_HW);
    tmpVal=BL_SET_REG_BIT(tmpVal,RF_TX_GAIN_CTRL_HW);
    BL_WR_REG(RF_BASE,RFCTRL_HW_EN,tmpVal);
}

static void rf_pri_save_state_for_cal()
{
    state_rf_fsm_ctrl_hw = BL_RD_REG(RF_BASE,RF_FSM_CTRL_HW);
    state_rfctrl_hw_en = BL_RD_REG(RF_BASE,RFCTRL_HW_EN);
    state_rfcal_ctrlen = BL_RD_REG(RF_BASE,RFCAL_CTRLEN);
    state_pucr1 = BL_RD_REG(RF_BASE,RF_PUCR1);
    state_fbdv = BL_RD_REG(RF_BASE,RF_FBDV);  
    state_sdm1 = BL_RD_REG(RF_BASE,RF_SDM1);
    state_sdm2 = BL_RD_REG(RF_BASE,RF_SDM2);
    state_rbb3 = BL_RD_REG(RF_BASE,RF_RBB3);
    state_adda1 = BL_RD_REG(RF_BASE,RF_ADDA1);
    state_dfe_ctrl_0 = BL_RD_REG(RF_BASE,RF_DFE_CTRL_0);
    state_dfe_ctrl_3 = BL_RD_REG(RF_BASE,RF_DFE_CTRL_3);
    state_dfe_ctrl_6 = BL_RD_REG(RF_BASE,RF_DFE_CTRL_6);   
    state_dfe_ctrl_7 = BL_RD_REG(RF_BASE,RF_DFE_CTRL_7); 
    state_trx_gain1 = BL_RD_REG(RF_BASE,RF_TRX_GAIN1);
    state_singen_ctrl0 = BL_RD_REG(RF_BASE,RF_SINGEN_CTRL0);
    state_singen_ctrl2 = BL_RD_REG(RF_BASE,RF_SINGEN_CTRL2);
    state_singen_ctrl3 = BL_RD_REG(RF_BASE,RF_SINGEN_CTRL3); 
    state_sram_ctrl0 = BL_RD_REG(RF_BASE,RF_SRAM_CTRL0);
    state_sram_ctrl1 = BL_RD_REG(RF_BASE,RF_SRAM_CTRL1);
    state_sram_ctrl2 = BL_RD_REG(RF_BASE,RF_SRAM_CTRL2);
    state_rf_resv_reg_1 = BL_RD_REG(RF_BASE,RF_RESV_REG_1);
    state_pa1 = BL_RD_REG(RF_BASE,RF_PA1);
    state_ten_ac = BL_RD_REG(RF_BASE,RF_TEN_AC);    
    state_rfif_dfe_ctrl0 = BL_RD_REG(RF_BASE,RFIF_DFE_CTRL0);
    state_tbb = BL_RD_REG(RF_BASE,RF_TBB);
    state_vco2 = BL_RD_REG(RF_BASE,RF_VCO2);
}

static void rf_pri_restore_state_for_cal()
{
    BL_WR_REG(RF_BASE,RF_FSM_CTRL_HW,state_rf_fsm_ctrl_hw);
    BL_WR_REG(RF_BASE,RFCTRL_HW_EN,state_rfctrl_hw_en);
    BL_WR_REG(RF_BASE,RFCAL_CTRLEN,state_rfcal_ctrlen);
    BL_WR_REG(RF_BASE,RF_PUCR1,state_pucr1);
    BL_WR_REG(RF_BASE,RF_FBDV,state_fbdv);  
    BL_WR_REG(RF_BASE,RF_SDM1,state_sdm1);
    BL_WR_REG(RF_BASE,RF_SDM2,state_sdm2);
    BL_WR_REG(RF_BASE,RF_RBB3,state_rbb3);
    BL_WR_REG(RF_BASE,RF_ADDA1,state_adda1);
    BL_WR_REG(RF_BASE,RF_DFE_CTRL_0,state_dfe_ctrl_0);
    BL_WR_REG(RF_BASE,RF_DFE_CTRL_3,state_dfe_ctrl_3);
    BL_WR_REG(RF_BASE,RF_DFE_CTRL_6,state_dfe_ctrl_6);  
    BL_WR_REG(RF_BASE,RF_DFE_CTRL_7,state_dfe_ctrl_7);
    BL_WR_REG(RF_BASE,RF_TRX_GAIN1,state_trx_gain1);
    BL_WR_REG(RF_BASE,RF_SINGEN_CTRL0,state_singen_ctrl0);
    BL_WR_REG(RF_BASE,RF_SINGEN_CTRL2,state_singen_ctrl2);
    BL_WR_REG(RF_BASE,RF_SINGEN_CTRL3,state_singen_ctrl3);
    BL_WR_REG(RF_BASE,RF_SRAM_CTRL0,state_sram_ctrl0);
    BL_WR_REG(RF_BASE,RF_SRAM_CTRL1,state_sram_ctrl1);
    BL_WR_REG(RF_BASE,RF_SRAM_CTRL2,state_sram_ctrl2);    
    BL_WR_REG(RF_BASE,RF_RESV_REG_1,state_rf_resv_reg_1);
    BL_WR_REG(RF_BASE,RF_PA1,state_pa1);    
    BL_WR_REG(RF_BASE,RF_TEN_AC,state_ten_ac);    
    BL_WR_REG(RF_BASE,RFIF_DFE_CTRL0,state_rfif_dfe_ctrl0);
    BL_WR_REG(RF_BASE,RF_TBB,state_tbb);
    BL_WR_REG(RF_BASE,RF_VCO2,state_vco2);
}

static void rf_pri_singen_config(uint32_t fcw,uint32_t start_addr_i,uint32_t start_addr_q)
{
    uint32_t tmpVal = 0;
    tmpVal=BL_RD_REG(RF_BASE,RF_SINGEN_CTRL0);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_SINGEN_INC_STEP0,fcw);
    BL_WR_REG(RF_BASE,RF_SINGEN_CTRL0,tmpVal);
    tmpVal=BL_RD_REG(RF_BASE,RF_SINGEN_CTRL2);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_SINGEN_START_ADDR0_I,start_addr_i);
    BL_WR_REG(RF_BASE,RF_SINGEN_CTRL2,tmpVal); 
    tmpVal=BL_RD_REG(RF_BASE,RF_SINGEN_CTRL3);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_SINGEN_START_ADDR0_Q,start_addr_q);
    BL_WR_REG(RF_BASE,RF_SINGEN_CTRL3,tmpVal);             
}

static void rf_pri_singen_start(void)
{
    uint32_t tmpVal = 0;
    tmpVal=BL_RD_REG(RF_BASE,RF_SINGEN_CTRL0);
    tmpVal=BL_CLR_REG_BIT(tmpVal,RF_SINGEN_EN);
    BL_WR_REG(RF_BASE,RF_SINGEN_CTRL0,tmpVal);
    tmpVal=BL_RD_REG(RF_BASE,RF_SINGEN_CTRL0);
    tmpVal=BL_SET_REG_BIT(tmpVal,RF_SINGEN_EN);
    BL_WR_REG(RF_BASE,RF_SINGEN_CTRL0,tmpVal);
}

static void rf_pri_singen_amplitude(uint32_t gain_i,uint32_t gain_q)
{
    uint32_t tmpVal = 0;
    tmpVal=BL_RD_REG(RF_BASE,RF_SINGEN_CTRL2);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_SINGEN_GAIN_I,gain_i);
    BL_WR_REG(RF_BASE,RF_SINGEN_CTRL2,tmpVal); 
    tmpVal=BL_RD_REG(RF_BASE,RF_SINGEN_CTRL3);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_SINGEN_GAIN_Q,gain_q);
    BL_WR_REG(RF_BASE,RF_SINGEN_CTRL3,tmpVal);    
}

static uint32_t rf_pri_pm_pwr()
{
    uint32_t tmpVal = 0;
    int32_t pm_iacc=0;
    int32_t pm_qacc=0;

    tmpVal=BL_RD_REG(RF_BASE,RF_DFE_CTRL_6);
    tmpVal=BL_CLR_REG_BIT(tmpVal,RF_RX_PM_FREQSHIFT_EN);
    tmpVal=BL_CLR_REG_BIT(tmpVal,RF_RX_PM_EN);
    BL_WR_REG(RF_BASE,RF_DFE_CTRL_6,tmpVal);        
    tmpVal=BL_RD_REG(RF_BASE,RF_DFE_CTRL_6);
    tmpVal=BL_SET_REG_BIT(tmpVal,RF_RX_PM_FREQSHIFT_EN);
    tmpVal=BL_SET_REG_BIT(tmpVal,RF_RX_PM_EN);
    BL_WR_REG(RF_BASE,RF_DFE_CTRL_6,tmpVal); 

    while(1){
        tmpVal=BL_RD_REG(RF_BASE,RF_DFE_CTRL_6);
        if (BL_GET_REG_BITS_VAL(tmpVal,RF_RX_PM_DONE))
        {
            break;
        }        
    }
    pm_iacc=BL_RD_REG(RF_BASE,RF_DFE_CTRL_8);
    pm_iacc=(pm_iacc<<7)>>(7+9);
    pm_qacc=BL_RD_REG(RF_BASE,RF_DFE_CTRL_9);
    pm_qacc=(pm_qacc<<7)>>(7+9);

    tmpVal=BL_RD_REG(RF_BASE,RF_DFE_CTRL_6);
    tmpVal=BL_CLR_REG_BIT(tmpVal,RF_RX_PM_FREQSHIFT_EN);
    tmpVal=BL_CLR_REG_BIT(tmpVal,RF_RX_PM_EN);
    BL_WR_REG(RF_BASE,RF_DFE_CTRL_6,tmpVal);

    return pm_iacc*pm_iacc+pm_qacc*pm_qacc;

}

static void rf_pri_rccal_config(uint32_t iq,uint32_t rbb_fc)
{
    uint32_t tmpVal = 0;
    if(iq){
        tmpVal=BL_RD_REG(RF_BASE,RF_RBB2);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_RBB_CAP1_FC_I,rbb_fc);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_RBB_CAP2_FC_I,rbb_fc);
        BL_WR_REG(RF_BASE,RF_RBB2,tmpVal);
    }else{
        tmpVal=BL_RD_REG(RF_BASE,RF_RBB2);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_RBB_CAP1_FC_Q,rbb_fc);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_RBB_CAP2_FC_Q,rbb_fc);
        BL_WR_REG(RF_BASE,RF_RBB2,tmpVal);
    }
}

// static void rf_pri_sram_config(uint32_t addr_start,uint32_t addr_end)
// {
//     uint32_t tmpVal = 0;
//     tmpVal=BL_RD_REG(RF_BASE,RF_SRAM_CTRL0);
//     tmpVal=BL_CLR_REG_BIT(tmpVal,RF_SRAM_SWAP);
//     tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_SRAM_LINK_MODE,0x0);
//     BL_WR_REG(RF_BASE,RF_SRAM_CTRL0,tmpVal);    
//     tmpVal=BL_RD_REG(RF_BASE,RF_SRAM_CTRL2);
//     tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_SRAM_ADC_ADDR_START,0x0);
//     tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_SRAM_ADC_ADDR_END,0x100);
//     BL_WR_REG(RF_BASE,RF_SRAM_CTRL2,tmpVal);     
//     tmpVal=BL_RD_REG(RF_BASE,RF_SRAM_CTRL1);
//     tmpVal=BL_CLR_REG_BIT(tmpVal,RF_SRAM_ADC_STS_CLR);
//     tmpVal=BL_CLR_REG_BIT(tmpVal,RF_SRAM_ADC_LOOP_EN);
//     BL_WR_REG(RF_BASE,RF_SRAM_CTRL1,tmpVal);
// }

// static void rf_pri_sram_start(void)
// {
//     uint32_t tmpVal = 0;
//     tmpVal=BL_RD_REG(RF_BASE,RF_SRAM_CTRL1);
//     tmpVal=BL_CLR_REG_BIT(tmpVal,RF_SRAM_ADC_EN);
//     BL_WR_REG(RF_BASE,RF_SRAM_CTRL1,tmpVal);
//     tmpVal=BL_RD_REG(RF_BASE,RF_SRAM_CTRL1);
//     tmpVal=BL_SET_REG_BIT(tmpVal,RF_SRAM_ADC_EN);
//     BL_WR_REG(RF_BASE,RF_SRAM_CTRL1,tmpVal);
//     while(1){
//         tmpVal=BL_RD_REG(RF_BASE,RF_SRAM_CTRL1);
//         if (BL_GET_REG_BITS_VAL(tmpVal,RF_SRAM_ADC_DONE))
//         {
//             break;
//         }        
//     }
//     tmpVal=BL_RD_REG(RF_BASE,RF_SRAM_CTRL1);
//     tmpVal=BL_CLR_REG_BIT(tmpVal,RF_SRAM_ADC_EN);
//     BL_WR_REG(RF_BASE,RF_SRAM_CTRL1,tmpVal);    
// }

// static int32_t rf_pri_sram_mean(uint32_t iq,uint32_t sram_start_addr,uint32_t length)
// {
//     // uint32_t tmpVal = 0;
//     uint32_t *p_sram;
//     int32_t sum=0;
//     uint32_t j=0;

//     p_sram=(uint32_t *)(0x42020000+sram_start_addr);

//     rf_pri_sram_start();

//     for(j=0;j<length;j++){
//         if(iq){
//             sum+=((p_sram[j]&0xffff)<<16)>>16;
//         }else{
//             sum+=(((p_sram[j]>>16)&0xffff)<<16)>>16;
//         }
//     }

//     BL_WR_REG(RF_BASE,RF_RESV_REG_1,sum);
//     return sum;

// }

static void rf_pri_roscal_config(uint32_t iq,uint32_t rosdac)
{
    uint32_t tmpVal = 0;
    if(iq){
        tmpVal=BL_RD_REG(RF_BASE,RF_RBB1);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROSDAC_I,rosdac);
        BL_WR_REG(RF_BASE,RF_RBB1,tmpVal);
    }else{
        tmpVal=BL_RD_REG(RF_BASE,RF_RBB1);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROSDAC_Q,rosdac);
        BL_WR_REG(RF_BASE,RF_RBB1,tmpVal);
    }
}

static void rf_pri_config_txgain(uint32_t gc_tbb_boost,uint32_t gc_tmx,uint32_t gc_tbb)
{
    uint32_t tmpVal = 0;
    tmpVal=BL_RD_REG(RF_BASE,RF_TRX_GAIN1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GC_TBB_BOOST,gc_tbb_boost);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GC_TMX,gc_tmx);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GC_TBB,gc_tbb);
    BL_WR_REG(RF_BASE,RF_TRX_GAIN1,tmpVal);    
}

static void rf_pri_start_txdfe()
{
    uint32_t tmpVal = 0;
    tmpVal=BL_RD_REG(RF_BASE,RFIF_DFE_CTRL0);
    tmpVal=BL_CLR_REG_BIT(tmpVal,RF_TX_DFE_EN_4S);
    tmpVal=BL_SET_REG_BIT(tmpVal,RFCKG_TXCLK_4S_ON);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_TEST_SEL,2);
    tmpVal=BL_SET_REG_BIT(tmpVal,RF_TX_DFE_EN_4S_EN);
    tmpVal=BL_SET_REG_BIT(tmpVal,RF_TX_DFE_EN_4S);        
    BL_WR_REG(RF_BASE,RFIF_DFE_CTRL0,tmpVal); 
}

static void rf_pri_stop_txdfe()
{
    uint32_t tmpVal = 0;
    tmpVal=BL_RD_REG(RF_BASE,RFIF_DFE_CTRL0);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_TEST_SEL,0);
    tmpVal=BL_CLR_REG_BIT(tmpVal,RFCKG_TXCLK_4S_ON);
    tmpVal=BL_CLR_REG_BIT(tmpVal,RF_TX_DFE_EN_4S_EN);
    tmpVal=BL_CLR_REG_BIT(tmpVal,RF_TX_DFE_EN_4S);        
    BL_WR_REG(RF_BASE,RFIF_DFE_CTRL0,tmpVal);    
}

static void rf_pri_start_rxdfe()
{
    uint32_t tmpVal = 0;
    tmpVal=BL_RD_REG(RF_BASE,RFIF_DFE_CTRL0);
    tmpVal=BL_CLR_REG_BIT(tmpVal,RF_RX_DFE_EN_4S);
    tmpVal=BL_SET_REG_BIT(tmpVal,RFCKG_RXCLK_4S_ON);
    tmpVal=BL_SET_REG_BIT(tmpVal,RF_RX_DFE_EN_4S_EN);  
    tmpVal=BL_SET_REG_BIT(tmpVal,RF_RX_DFE_EN_4S);          
    BL_WR_REG(RF_BASE,RFIF_DFE_CTRL0,tmpVal);
}

static void rf_pri_stop_rxdfe()
{
    uint32_t tmpVal = 0;
    tmpVal=BL_RD_REG(RF_BASE,RFIF_DFE_CTRL0);
    tmpVal=BL_CLR_REG_BIT(tmpVal,RFCKG_RXCLK_4S_ON);
    tmpVal=BL_CLR_REG_BIT(tmpVal,RF_RX_DFE_EN_4S_EN);  
    tmpVal=BL_CLR_REG_BIT(tmpVal,RF_RX_DFE_EN_4S);
    BL_WR_REG(RF_BASE,RFIF_DFE_CTRL0,tmpVal);
}

static int32_t rf_pri_pm_pwr_avg(uint32_t iq,uint32_t acc_len)
{
    uint32_t tmpVal = 0;
    int32_t pm_iq_avg=0;

    tmpVal=BL_RD_REG(RF_BASE,RF_DFE_CTRL_6);
    tmpVal=BL_CLR_REG_BIT(tmpVal,RF_RX_PM_FREQSHIFT_EN);
    tmpVal=BL_CLR_REG_BIT(tmpVal,RF_RX_PM_EN);
    BL_WR_REG(RF_BASE,RF_DFE_CTRL_6,tmpVal);
    tmpVal=BL_RD_REG(RF_BASE,RF_DFE_CTRL_7);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_RX_PM_ACC_LEN,acc_len);
    BL_WR_REG(RF_BASE,RF_DFE_CTRL_7,tmpVal);
    tmpVal=BL_RD_REG(RF_BASE,RF_DFE_CTRL_6);
    // tmpVal=BL_SET_REG_BIT(tmpVal,RF_RX_PM_FREQSHIFT_EN);
    tmpVal=BL_SET_REG_BIT(tmpVal,RF_RX_PM_EN);
    BL_WR_REG(RF_BASE,RF_DFE_CTRL_6,tmpVal); 


    while(1){
        tmpVal=BL_RD_REG(RF_BASE,RF_DFE_CTRL_6);
        if (BL_GET_REG_BITS_VAL(tmpVal,RF_RX_PM_DONE))
        {
            break;
        }        
    }
    if(iq){
        pm_iq_avg=BL_RD_REG(RF_BASE,RF_DFE_CTRL_8);
    }else{
        pm_iq_avg=BL_RD_REG(RF_BASE,RF_DFE_CTRL_9);
    }
    pm_iq_avg=(pm_iq_avg<<7)>>(7);

    tmpVal=BL_RD_REG(RF_BASE,RF_DFE_CTRL_6);
    tmpVal=BL_CLR_REG_BIT(tmpVal,RF_RX_PM_FREQSHIFT_EN);
    tmpVal=BL_CLR_REG_BIT(tmpVal,RF_RX_PM_EN);
    BL_WR_REG(RF_BASE,RF_DFE_CTRL_6,tmpVal);

    return pm_iq_avg;
}

static void rf_pri_txcal_config(uint32_t param_ind,int32_t val)
{
    uint32_t tmpVal = 0;
    if(param_ind==E_RF_GAIN){
        tmpVal=BL_RD_REG(RF_BASE,RF_DFE_CTRL_0);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQC_GAIN,val);
        tmpVal=BL_SET_REG_BIT(tmpVal,RF_TX_IQC_GAIN_EN);
        BL_WR_REG(RF_BASE,RF_DFE_CTRL_0,tmpVal);        
    }else if(param_ind==E_RF_PHASE){
        // rf_pri_printf("param_ind=%d,val=%d\r\n",param_ind,val);
        val = val >= 0 ? val : (val+0x400);
        tmpVal=BL_RD_REG(RF_BASE,RF_DFE_CTRL_0);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQC_PHASE,val);
        tmpVal=BL_SET_REG_BIT(tmpVal,RF_TX_IQC_PHASE_EN);
        BL_WR_REG(RF_BASE,RF_DFE_CTRL_0,tmpVal);
    }else if(param_ind==E_RF_BRANCH_I){
        tmpVal=BL_RD_REG(RF_BASE,RF_TBB);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_I,val);
        BL_WR_REG(RF_BASE,RF_TBB,tmpVal);
    }else if(param_ind==E_RF_BRANCH_Q){
        tmpVal=BL_RD_REG(RF_BASE,RF_TBB);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_Q,val);
        BL_WR_REG(RF_BASE,RF_TBB,tmpVal);
    }
}

static int32_t rf_pri_txcal_search_core(uint32_t param_ind,uint32_t center,uint32_t delta,uint32_t meas_freq)
{
    uint32_t tmpVal = 0;
    int32_t x_center = center;
    int32_t x_delta = delta;
    int32_t x_left, x_right;
    uint32_t y_center, y_left, y_right;
	
    // measure center
    rf_pri_txcal_config(param_ind,x_center);
    rf_pri_wait_us(10);
    tmpVal=BL_RD_REG(RF_BASE,RF_DFE_CTRL_6);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_RX_PM_FREQSHIFT_CW,meas_freq<<10);
    BL_WR_REG(RF_BASE,RF_DFE_CTRL_6,tmpVal);    
    y_center = rf_pri_pm_pwr();
	
    do {

        if (x_delta == 0) break;
    
        x_left  = x_center - x_delta;
        x_right = x_center + x_delta;
        if (param_ind == E_RF_BRANCH_I || param_ind == E_RF_BRANCH_Q){ 
                x_left = x_left < 0 ? 0 : x_left;
                x_right = x_right > 63 ? 63 : x_right;
        }
        else if (param_ind == E_RF_GAIN){
                x_left = x_left < 0 ? 0 : x_left;
                x_right = x_right > 2047 ? 2047 : x_right;					
        }
        else if (param_ind == E_RF_PHASE){
                x_left = x_left < -512 ? -512 : x_left;
                x_right = x_right > 511 ? 511 : x_right;					
        }
        x_delta = x_delta >> 1;

        // rf_pri_printf("param_ind=%d,x_delta=%d,x_center=%d,x_left=%d,x_right=%d,y_center=%d,y_left=%d,y_right=%d\r\n",param_ind,x_delta,x_center,x_left,x_right,y_center,y_left,y_right);

        // measure left
        rf_pri_txcal_config(param_ind,x_left);
        rf_pri_wait_us(10);
        y_left = rf_pri_pm_pwr();

        if (y_center >= y_left) {
            x_center = x_left;
            y_center = y_left;
            continue;
        }

        // measure right
        rf_pri_txcal_config(param_ind,x_right);
        rf_pri_wait_us(10);
        y_right = rf_pri_pm_pwr();

        if (y_center >= y_right) {
            x_center = x_right;
            y_center = y_right;
            continue;
        }        
    } while(x_delta);

    return x_center;
}

static void rf_pri_txcal_config_hw(){
    uint32_t tmpVal = 0;

    tmpVal=BL_RD_REG(RF_BASE,RF_TOSDAC_CTRL_HW1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_I_GC0,rf_calib_data->txcal[0].tosdac_i);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_Q_GC0,rf_calib_data->txcal[0].tosdac_q);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_I_GC1,rf_calib_data->txcal[1].tosdac_i);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_Q_GC1,rf_calib_data->txcal[1].tosdac_q);
    BL_WR_REG(RF_BASE,RF_TOSDAC_CTRL_HW1,tmpVal);
    tmpVal=BL_RD_REG(RF_BASE,RF_TOSDAC_CTRL_HW2);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_I_GC2,rf_calib_data->txcal[2].tosdac_i);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_Q_GC2,rf_calib_data->txcal[2].tosdac_q);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_I_GC3,rf_calib_data->txcal[3].tosdac_i);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_Q_GC3,rf_calib_data->txcal[3].tosdac_q);
    BL_WR_REG(RF_BASE,RF_TOSDAC_CTRL_HW2,tmpVal); 
    tmpVal=BL_RD_REG(RF_BASE,RF_TOSDAC_CTRL_HW3);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_I_GC4,rf_calib_data->txcal[4].tosdac_i);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_Q_GC4,rf_calib_data->txcal[4].tosdac_q);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_I_GC5,rf_calib_data->txcal[5].tosdac_i);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_Q_GC5,rf_calib_data->txcal[5].tosdac_q);
    BL_WR_REG(RF_BASE,RF_TOSDAC_CTRL_HW3,tmpVal);
    tmpVal=BL_RD_REG(RF_BASE,RF_TOSDAC_CTRL_HW4);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_I_GC6,rf_calib_data->txcal[6].tosdac_i);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_Q_GC6,rf_calib_data->txcal[6].tosdac_q);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_I_GC7,rf_calib_data->txcal[7].tosdac_i);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_Q_GC7,rf_calib_data->txcal[7].tosdac_q);
    BL_WR_REG(RF_BASE,RF_TOSDAC_CTRL_HW4,tmpVal);
    
    tmpVal=BL_RD_REG(RF_BASE,RF_TX_IQ_GAIN_HW0);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_GAIN_COMP_GC0,rf_calib_data->txcal[0].tx_iq_gain_comp);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_PHASE_COMP_GC0,rf_calib_data->txcal[0].tx_iq_phase_comp);    
    BL_WR_REG(RF_BASE,RF_TX_IQ_GAIN_HW0,tmpVal);
    tmpVal=BL_RD_REG(RF_BASE,RF_TX_IQ_GAIN_HW1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_GAIN_COMP_GC1,rf_calib_data->txcal[1].tx_iq_gain_comp);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_PHASE_COMP_GC1,rf_calib_data->txcal[1].tx_iq_phase_comp);  
    BL_WR_REG(RF_BASE,RF_TX_IQ_GAIN_HW1,tmpVal);
    tmpVal=BL_RD_REG(RF_BASE,RF_TX_IQ_GAIN_HW2);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_GAIN_COMP_GC2,rf_calib_data->txcal[2].tx_iq_gain_comp);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_PHASE_COMP_GC2,rf_calib_data->txcal[2].tx_iq_phase_comp);  
    BL_WR_REG(RF_BASE,RF_TX_IQ_GAIN_HW2,tmpVal);
    tmpVal=BL_RD_REG(RF_BASE,RF_TX_IQ_GAIN_HW3);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_GAIN_COMP_GC3,rf_calib_data->txcal[3].tx_iq_gain_comp);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_PHASE_COMP_GC3,rf_calib_data->txcal[3].tx_iq_phase_comp);  
    BL_WR_REG(RF_BASE,RF_TX_IQ_GAIN_HW3,tmpVal);
    tmpVal=BL_RD_REG(RF_BASE,RF_TX_IQ_GAIN_HW4);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_GAIN_COMP_GC4,rf_calib_data->txcal[4].tx_iq_gain_comp);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_PHASE_COMP_GC4,rf_calib_data->txcal[4].tx_iq_phase_comp);  
    BL_WR_REG(RF_BASE,RF_TX_IQ_GAIN_HW4,tmpVal);
    tmpVal=BL_RD_REG(RF_BASE,RF_TX_IQ_GAIN_HW5);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_GAIN_COMP_GC5,rf_calib_data->txcal[5].tx_iq_gain_comp);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_PHASE_COMP_GC5,rf_calib_data->txcal[5].tx_iq_phase_comp);  
    BL_WR_REG(RF_BASE,RF_TX_IQ_GAIN_HW5,tmpVal);
    tmpVal=BL_RD_REG(RF_BASE,RF_TX_IQ_GAIN_HW6);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_GAIN_COMP_GC6,rf_calib_data->txcal[6].tx_iq_gain_comp);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_PHASE_COMP_GC6,rf_calib_data->txcal[6].tx_iq_phase_comp);  
    BL_WR_REG(RF_BASE,RF_TX_IQ_GAIN_HW6,tmpVal);
    tmpVal=BL_RD_REG(RF_BASE,RF_TX_IQ_GAIN_HW7);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_GAIN_COMP_GC7,rf_calib_data->txcal[7].tx_iq_gain_comp);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_PHASE_COMP_GC7,rf_calib_data->txcal[7].tx_iq_phase_comp);   
    BL_WR_REG(RF_BASE,RF_TX_IQ_GAIN_HW7,tmpVal);       
}

static uint32_t rf_pri_roscal_iq(uint32_t iq,uint32_t sram_start_addr,uint32_t length)
{
    uint32_t i=0;
    uint32_t step=32;
    uint32_t seq_pattern=0;
    uint32_t rosdac=0;
    int32_t adc_mean_iq=0;

    for(i=0;i<6;i++){
        rosdac+=step;
        rf_pri_roscal_config(iq,rosdac);

        adc_mean_iq=rf_pri_pm_pwr_avg(iq,1024);
        // rf_pri_printf("iq=%d,i=%d,rosdac=%d,adc_mean_iq=%d\r\n",iq,i,rosdac,adc_mean_iq);
        if(adc_mean_iq>0){
        // if(rf_pri_sram_mean(iq,sram_start_addr,length)>0){
            rosdac-=step;
        }else{
            rosdac=rosdac;
        }
        step=step>>1;
    }

    for(i=0;i<63;i++){
        rf_pri_roscal_config(iq,rosdac);
        adc_mean_iq=rf_pri_pm_pwr_avg(iq,1024);
        // rf_pri_printf("iq=%d,i=%d,rosdac=%d,adc_mean_iq=%d\r\n",iq,i,rosdac,adc_mean_iq);
        if(adc_mean_iq>0){
        // if(rf_pri_sram_mean(iq,sram_start_addr,length)>0){
            rosdac--;
            seq_pattern=((seq_pattern<<1)+1)&0xf;
        }else{
            rosdac++;
            seq_pattern=(seq_pattern<<1)&0xf;
        }        
        if(seq_pattern==0xa || seq_pattern==0x5)break;
    }
    return rosdac;

} 

static uint32_t rf_pri_rccal_iq(uint32_t iq)
{
    uint32_t tmpVal = 0;
    uint32_t i=0;
    uint32_t step=32;
    uint32_t seq_pattern=0;
    uint32_t k1=3;
    uint32_t k2=181;
    uint32_t rbb_fc=0;
    double rccal_power_scale=0.9*0.9;
    uint32_t rccal_ref=0;
    uint32_t rccal_val=0;

    if(iq)
    {
        tmpVal=BL_RD_REG(RF_BASE,RF_DFE_CTRL_3);
        tmpVal=BL_CLR_REG_BIT(tmpVal,RF_RX_ADC_4S_I_EN);
        BL_WR_REG(RF_BASE,RF_DFE_CTRL_3,tmpVal);
        tmpVal=BL_RD_REG(RF_BASE,RF_DFE_CTRL_3);
        tmpVal=BL_SET_REG_BIT(tmpVal,RF_RX_ADC_4S_Q_EN);
        BL_WR_REG(RF_BASE,RF_DFE_CTRL_3,tmpVal);        
    }else
    {
        tmpVal=BL_RD_REG(RF_BASE,RF_DFE_CTRL_3);
        tmpVal=BL_SET_REG_BIT(tmpVal,RF_RX_ADC_4S_I_EN);
        BL_WR_REG(RF_BASE,RF_DFE_CTRL_3,tmpVal);
        tmpVal=BL_RD_REG(RF_BASE,RF_DFE_CTRL_3);
        tmpVal=BL_CLR_REG_BIT(tmpVal,RF_RX_ADC_4S_Q_EN);
        BL_WR_REG(RF_BASE,RF_DFE_CTRL_3,tmpVal);       
    }

    tmpVal=BL_RD_REG(RF_BASE,RF_TRX_GAIN1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GC_RBB1,1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GC_RBB2,3);
    BL_WR_REG(RF_BASE,RF_TRX_GAIN1,tmpVal);

    rf_pri_singen_config(k1,0x0,0x300);
    rf_pri_singen_amplitude(0x3ff,0x3ff);
    rf_pri_singen_start();

    tmpVal=BL_RD_REG(RF_BASE,RF_DFE_CTRL_6);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_RX_PM_FREQSHIFT_CW,k1<<10);
    BL_WR_REG(RF_BASE,RF_DFE_CTRL_6,tmpVal);    

    rf_pri_pm_pwr_avg(iq,1024);
    rccal_ref=(uint32_t)(rccal_power_scale*rf_pri_pm_pwr());

    tmpVal=BL_RD_REG(RF_BASE,RF_TRX_GAIN1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GC_RBB1,2);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GC_RBB2,6);
    BL_WR_REG(RF_BASE,RF_TRX_GAIN1,tmpVal);

    rf_pri_singen_config(k2,0x0,0x300);
    rf_pri_singen_start();   

    tmpVal=BL_RD_REG(RF_BASE,RF_DFE_CTRL_6);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_RX_PM_FREQSHIFT_CW,k2<<10);
    BL_WR_REG(RF_BASE,RF_DFE_CTRL_6,tmpVal);  

    // step=32;
    for(i=0;i<6;i++){
        rbb_fc+=step;
        rf_pri_rccal_config(iq,rbb_fc);

        rccal_val = rf_pri_pm_pwr();
        // rf_pri_printf("rccal_ref=%d,iq=%d,step=%d,rbb_fc=%d,rccal_val=%d\r\n",rccal_ref,iq,step,rbb_fc,rccal_val);

        if(rccal_val>rccal_ref){
            rbb_fc=rbb_fc;
        }else{
            rbb_fc-=step;
        }     
        step=step>>1;
    }

    // rf_pri_rccal_config(iq,rbb_fc);

    // seq_pattern=0;
    for(i=0;i<63;i++){
        rf_pri_rccal_config(iq,rbb_fc);

        rccal_val = rf_pri_pm_pwr();

        if(rccal_val>rccal_ref){
            rbb_fc++;
            seq_pattern=((seq_pattern<<1)+1)&0xf;
        }else{
            rbb_fc--;
            seq_pattern=(seq_pattern<<1)&0xf;
        }

        if(seq_pattern==0xa || seq_pattern==0x5)break;
    }

    if(i==63){
        return 2;
    }else{
        return 3;
    }

}

static uint16_t rf_pri_fcal_meas(uint32_t cw)
{
    uint32_t tmpVal = 0;
    uint16_t cnt;

    tmpVal=BL_RD_REG(RF_BASE,RF_VCO1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_LO_VCO_FREQ_CW,cw);
    BL_WR_REG(RF_BASE,RF_VCO1,tmpVal);
    rf_pri_wait_us(100);
    tmpVal=BL_RD_REG(RF_BASE,RF_VCO4);
    tmpVal=BL_SET_REG_BIT(tmpVal,RF_FCAL_CNT_START);
    BL_WR_REG(RF_BASE,RF_VCO4,tmpVal);
    while (1)
    {
        tmpVal=BL_RD_REG(RF_BASE,RF_VCO4);
        if (BL_GET_REG_BITS_VAL(tmpVal,RF_FCAL_CNT_RDY))
        {
            break;
        }
    }
    tmpVal=BL_RD_REG(RF_BASE,RF_VCO3);
    cnt = BL_GET_REG_BITS_VAL(tmpVal,RF_FCAL_CNT_OP);
    tmpVal=BL_RD_REG(RF_BASE,RF_VCO4);
    tmpVal=BL_CLR_REG_BIT(tmpVal,RF_FCAL_CNT_START);
    BL_WR_REG(RF_BASE,RF_VCO4,tmpVal);

    return cnt;
}

static void rf_pri_singen_pwrmx_dc(uint32_t amp,uint32_t num_data,int32_t adc_mean_max,int32_t adc_mean_min){
    uint32_t step;
    int32_t adc_mean_i=0;
    step = amp>>1;
    while(1){
        rf_pri_singen_amplitude(amp,amp);
        rf_pri_singen_start();
        // rf_pri_start_txdfe();
        // rf_pri_start_rxdfe();
        rf_pri_wait_us(10);
        adc_mean_i=rf_pri_pm_pwr_avg(1,num_data);
        adc_mean_i=adc_mean_i>>10;
        rf_pri_printf("amp=%ld,step=%ld,adc_mean_i=%ld\r\n",amp,step,adc_mean_i);
        // rf_pri_printf("adc_mean_max=%ld,adc_mean_min=%ld\r\n",adc_mean_max,adc_mean_min);
        // rf_pri_printf("adc_mean_i>adc_mean_max=%ld\r\n",adc_mean_i>adc_mean_max);
        // rf_pri_printf("adc_mean_i<adc_mean_min=%ld\r\n",adc_mean_i<adc_mean_min);
        if(adc_mean_i>adc_mean_max)
            amp-=step;
        else if(adc_mean_i<adc_mean_min)
            amp+=step;
        else 
            break;               
        if(step>0)
            step=step>>1;
        else
            break;            
    }
}

#if 0
void rf_pri_channel_opt(uint32_t channel_index)
{
    uint32_t tmpVal = 0;
    if(channel_index >= E_RF_CHANNEL_2404M && channel_index <= E_RF_CHANNEL_2436M){
        tmpVal=BL_RD_REG(RF_BASE,RF_TMX);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TMX_CS,6);
        BL_WR_REG(RF_BASE,RF_TMX,tmpVal);
    }else{
        tmpVal=BL_RD_REG(RF_BASE,RF_TMX);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TMX_CS,5);
        BL_WR_REG(RF_BASE,RF_TMX,tmpVal);
    }
}
#endif

void rf_pri_config_bandwidth(uint32_t bw)
{
    uint32_t tmpVal = 0;
    // Set RF RBB bandwidth, 0: n/a, 1: 5M mode, 2:10M mode, 3: 20M mode
    tmpVal=BL_RD_REG(RF_BASE,RF_RBB3);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_RBB_BW,bw);
    BL_WR_REG(RF_BASE,RF_RBB3,tmpVal);
}

void rf_pri_txcal(void)
{
    uint32_t tmpVal = 0;
    uint32_t gain=1024;
    int32_t phase=0;
    uint32_t tos_i=32;
    uint32_t tos_q=32;
    uint32_t k1=61;
    int32_t adc_mean_min=192;
    int32_t adc_mean_max=320;
    // int32_t adc_mean_i=0;
    uint32_t num_data=1024;
    uint32_t i=0;
    // uint32_t j=0;
    uint32_t amp;
    // uint32_t step;

    tmpVal=BL_RD_REG(RF_BASE,RFCAL_STATUS);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_LO_LEAKCAL_STATUS,0x1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TIQCAL_STATUS_RESV,0x1);
    BL_WR_REG(RF_BASE,RFCAL_STATUS,tmpVal);  

    rf_pri_save_state_for_cal();

    rf_pri_config_mode(E_RF_MODE_TXCAL);
    rf_pri_config_channel(E_RF_CHANNEL_2440M);
    tmpVal=BL_RD_REG(RF_BASE,RFCAL_CTRLEN);
    tmpVal=BL_SET_REG_BIT(tmpVal,RF_LO_LEAKCAL_EN);
    tmpVal=BL_SET_REG_BIT(tmpVal,RF_TIQCAL_EN);
    BL_WR_REG(RF_BASE,RFCAL_CTRLEN,tmpVal);

    tmpVal=BL_RD_REG(RF_BASE,RF_RBB3);
    tmpVal=BL_SET_REG_BIT(tmpVal,RF_PWR_DET_EN);
    BL_WR_REG(RF_BASE,RF_RBB3,tmpVal);
    tmpVal=BL_RD_REG(RF_BASE,RF_PA1);
    tmpVal=BL_SET_REG_BIT(tmpVal,RF_PA_PWRMX_DAC_PN_SWITCH);
    // tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_PA_PWRMX_OSDAC,5);
    BL_WR_REG(RF_BASE,RF_PA1,tmpVal);

    tmpVal=BL_RD_REG(RF_BASE,RF_TMX);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TMX_CS,3);//rf_calib_data->cal.tmx_csh);//3);
    BL_WR_REG(RF_BASE,RF_TMX,tmpVal);

    #if 0
    uint32_t k2=3;
    uint32_t tmxcs_pwr=0;
    uint32_t tmxcs_pwr_max=0;
    uint32_t tmxcs_max=0;
    tmpVal=BL_RD_REG(RF_BASE,RF_TMX);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TMX_CS,3);
    BL_WR_REG(RF_BASE,RF_TMX,tmpVal);
    rf_pri_singen_config(k2,0x0,0x300);
    tmpVal=BL_RD_REG(RF_BASE,RF_PA1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_PA_ATT_GC,0);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_PA_PWRMX_OSDAC,0);
    BL_WR_REG(RF_BASE,RF_PA1,tmpVal);
    tmpVal=BL_RD_REG(RF_BASE,RF_TEN_AC);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ATEST_GAIN_R5,4);
    BL_WR_REG(RF_BASE,RF_TEN_AC,tmpVal);
    rf_pri_config_txgain(0,3,3);
    amp=0x100;
    rf_pri_singen_pwrmx_dc(amp,num_data,adc_mean_min,adc_mean_min>>1);
    rf_pri_pm_pwr_avg(1,4096);
    tmpVal=BL_RD_REG(RF_BASE,RF_DFE_CTRL_6);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_RX_PM_FREQSHIFT_CW,0<<10);
    BL_WR_REG(RF_BASE,RF_DFE_CTRL_6,tmpVal);
    for(i=0;i<8;i++){
        tmpVal=BL_RD_REG(RF_BASE,RF_TMX);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TMX_CS,i);
        BL_WR_REG(RF_BASE,RF_TMX,tmpVal); 
        rf_pri_wait_us(10);
        tmxcs_pwr=rf_pri_pm_pwr();
        rf_pri_printf("tmx_cs=%ld, tmxcs_pwr=%ld\r\n",i,tmxcs_pwr);
        if(tmxcs_pwr>tmxcs_pwr_max){
            tmxcs_pwr_max=tmxcs_pwr;
            tmxcs_max=i;
        }
    }
    if(tmxcs_max<5){
        tmpVal=BL_RD_REG(RF_BASE,RF_TMX);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TMX_CS,tmxcs_max+1);
        BL_WR_REG(RF_BASE,RF_TMX,tmpVal);
        rf_pri_wait_us(10);
        tmxcs_pwr=rf_pri_pm_pwr();
        rf_pri_printf("tmx_cs=%ld, tmxcs_pwr=%ld\r\n",tmxcs_max+1,tmxcs_pwr);
        rf_pri_printf("tmxcs_pwr=%ld,tmxcs_pwr_max=%ld\r\n",tmxcs_pwr*28,tmxcs_pwr_max*25);
        if((tmxcs_pwr*28)>(tmxcs_pwr_max*25)){
            tmxcs_max++;
        }
        rf_calib_data->cal.tmx_csh=tmxcs_max+2;
        if(rf_calib_data->cal.tmx_csh<7){
            rf_calib_data->cal.tmx_csl=rf_calib_data->cal.tmx_csh+1;
        }else{
            rf_calib_data->cal.tmx_csl=rf_calib_data->cal.tmx_csh;
        }
    }else{
        rf_calib_data->cal.tmx_csh=7;
        rf_calib_data->cal.tmx_csl=7;
    }
    rf_pri_printf("tmx_csh=%d, tmx_csl=%d\r\n",rf_calib_data->cal.tmx_csh,rf_calib_data->cal.tmx_csl);
    tmpVal=BL_RD_REG(RF_BASE,RF_TMX);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TMX_CS,rf_calib_data->cal.tmx_csh);
    BL_WR_REG(RF_BASE,RF_TMX,tmpVal);
    #endif

    rf_pri_singen_config(k1,0x0,0x300);
    for(i=0;i<8;i++){
        tmpVal=BL_RD_REG(RF_BASE,RF_PA1);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_PA_ATT_GC,txcal_para[i][0]);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_PA_PWRMX_OSDAC,txcal_para[i][2]);
        BL_WR_REG(RF_BASE,RF_PA1,tmpVal);
        tmpVal=BL_RD_REG(RF_BASE,RF_TEN_AC);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ATEST_GAIN_R5,txcal_para[i][1]);
        BL_WR_REG(RF_BASE,RF_TEN_AC,tmpVal);

        // rf_pri_config_txgain(tx_pwr_table[i<<1][0],tx_pwr_table[i<<1][2],tx_pwr_table[i<<1][3]);
        rf_pri_config_txgain(tx_pwr_table[i<<1][0],7,tx_pwr_table[i<<1][3]);
        rf_pri_wait_us(10);

        amp=txcal_para[i][3];
        rf_pri_singen_pwrmx_dc(amp,num_data,adc_mean_max,adc_mean_min);

        rf_pri_pm_pwr_avg(1,4096);
        // LO leakage cal
        tos_i = rf_pri_txcal_search_core(E_RF_BRANCH_I,32,16,k1);
        rf_pri_txcal_config(E_RF_BRANCH_I,tos_i);

        tos_q = rf_pri_txcal_search_core(E_RF_BRANCH_Q,32,16,k1);
        rf_pri_txcal_config(E_RF_BRANCH_Q,tos_q);

        // LO leakage cal double search
        tos_i = rf_pri_txcal_search_core(E_RF_BRANCH_I,tos_i,2,k1);
        rf_pri_txcal_config(E_RF_BRANCH_I,tos_i);

        // Gain cal
        gain = rf_pri_txcal_search_core(E_RF_GAIN,0x400,0x80,2*k1);
        rf_pri_txcal_config(E_RF_GAIN,gain);

        // Phase cal
        phase = rf_pri_txcal_search_core(E_RF_PHASE,0,0x40,2*k1);
        rf_pri_txcal_config(E_RF_PHASE,phase);

        // Gain cal double search
        gain = rf_pri_txcal_search_core(E_RF_GAIN,gain,0x10,2*k1);
        rf_pri_txcal_config(E_RF_GAIN,gain);

        // Phase cal double search
        phase = rf_pri_txcal_search_core(E_RF_PHASE,0,0x8,2*k1);
        rf_pri_txcal_config(E_RF_PHASE,phase);        

        rf_calib_data->txcal[E_RF_TXCAL_GAIN_CNT-1-i].tosdac_i = tos_i;
        rf_calib_data->txcal[E_RF_TXCAL_GAIN_CNT-1-i].tosdac_q = tos_q;
        rf_calib_data->txcal[E_RF_TXCAL_GAIN_CNT-1-i].tx_iq_gain_comp = gain;
        rf_calib_data->txcal[E_RF_TXCAL_GAIN_CNT-1-i].tx_iq_phase_comp = phase;
        rf_pri_printf("tosdac_i=%ld,tosdac_q=%ld,tx_iq_gain_comp=%ld,tx_iq_phase_comp=%ld\r\n",tos_i,tos_q,gain,phase);
    }

    rf_pri_txcal_config_hw();

    tmpVal=BL_RD_REG(RF_BASE,RFCAL_CTRLEN);
    tmpVal=BL_CLR_REG_BIT(tmpVal,RF_LO_LEAKCAL_EN);
    tmpVal=BL_CLR_REG_BIT(tmpVal,RF_TIQCAL_EN);
    BL_WR_REG(RF_BASE,RFCAL_CTRLEN,tmpVal);   

    rf_pri_restore_state_for_cal();

    tmpVal=BL_RD_REG(RF_BASE,RFCAL_STATUS);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_LO_LEAKCAL_STATUS,0x3);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TIQCAL_STATUS_RESV,0x3);
    BL_WR_REG(RF_BASE,RFCAL_STATUS,tmpVal);  

    tmpVal=BL_RD_REG(RF_BASE,RF_TBB);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_I,rf_calib_data->txcal[3].tosdac_i);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_Q,rf_calib_data->txcal[3].tosdac_q);
    BL_WR_REG(RF_BASE,RF_TBB,tmpVal);
    tmpVal=BL_RD_REG(RF_BASE,RF_DFE_CTRL_0);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQC_GAIN,rf_calib_data->txcal[3].tx_iq_gain_comp);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQC_PHASE,rf_calib_data->txcal[3].tx_iq_phase_comp);
    BL_WR_REG(RF_BASE,RF_DFE_CTRL_0,tmpVal);          
}

void rf_pri_roscal(void){
    uint32_t tmpVal = 0;
    uint32_t sram_start_addr=0;
    uint32_t sram_length=32;
    uint32_t rosdac=32;

    tmpVal=BL_RD_REG(RF_BASE,RFCAL_STATEEN);
    if(BL_GET_REG_BITS_VAL(tmpVal,RF_ROSCAL_STEN)==0)
    {
        tmpVal=BL_RD_REG(RF_BASE,RFCAL_STATUS);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROS_STATUS,0x0);
        BL_WR_REG(RF_BASE,RFCAL_STATUS,tmpVal);
        return;
    }else
    {
        tmpVal=BL_RD_REG(RF_BASE,RFCAL_STATUS);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROS_STATUS,0x1);
        BL_WR_REG(RF_BASE,RFCAL_STATUS,tmpVal);        
    }

    rf_pri_save_state_for_cal();

    rf_pri_config_mode(E_RF_MODE_ROSCAL);
    tmpVal=BL_RD_REG(RF_BASE,RFCAL_CTRLEN);
    tmpVal=BL_SET_REG_BIT(tmpVal,RF_ROSCAL_EN);
    BL_WR_REG(RF_BASE,RFCAL_CTRLEN,tmpVal);

    rf_pri_config_channel(E_RF_CHANNEL_2440M);

    // rf_pri_sram_config(0x0,sram_length);

#if 0
    tmpVal=BL_RD_REG(RF_BASE,RF_TRX_GAIN1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GC_RBB2,6);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GC_RBB1,3);
    BL_WR_REG(RF_BASE,RF_TRX_GAIN1,tmpVal);
    rosdac=rf_pri_roscal_iq(1,sram_start_addr,sram_length);
    rf_calib_data->rxcal[3].rosdac_i=rosdac;
    tmpVal=BL_RD_REG(RF_BASE,RF_ROSDAC_CTRL_HW2);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROSDAC_I_GC3,rosdac);
    BL_WR_REG(RF_BASE,RF_ROSDAC_CTRL_HW2,tmpVal); 
    rf_pri_printf("rosdac_i_gc3=%ld\r\n",rosdac);        
    rosdac=rf_pri_roscal_iq(0,sram_start_addr,sram_length);
    rf_calib_data->rxcal[3].rosdac_q=rosdac;
    tmpVal=BL_RD_REG(RF_BASE,RF_ROSDAC_CTRL_HW2);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROSDAC_Q_GC3,rosdac);
    BL_WR_REG(RF_BASE,RF_ROSDAC_CTRL_HW2,tmpVal); 
    rf_pri_printf("rosdac_q_gc3=%ld\r\n",rosdac);      

    tmpVal=BL_RD_REG(RF_BASE,RF_TRX_GAIN1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GC_RBB1,2);
    BL_WR_REG(RF_BASE,RF_TRX_GAIN1,tmpVal);
    rosdac=rf_pri_roscal_iq(1,sram_start_addr,sram_length);
    rf_calib_data->rxcal[2].rosdac_i=rosdac;
    tmpVal=BL_RD_REG(RF_BASE,RF_ROSDAC_CTRL_HW2);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROSDAC_I_GC2,rosdac);
    BL_WR_REG(RF_BASE,RF_ROSDAC_CTRL_HW2,tmpVal);
    rf_pri_printf("rosdac_i_gc2=%ld\r\n",rosdac);       
    rosdac=rf_pri_roscal_iq(0,sram_start_addr,sram_length);
    rf_calib_data->rxcal[2].rosdac_q=rosdac;
    tmpVal=BL_RD_REG(RF_BASE,RF_ROSDAC_CTRL_HW2);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROSDAC_Q_GC2,rosdac);
    BL_WR_REG(RF_BASE,RF_ROSDAC_CTRL_HW2,tmpVal);  
    rf_pri_printf("rosdac_q_gc2=%ld\r\n",rosdac);    

    tmpVal=BL_RD_REG(RF_BASE,RF_TRX_GAIN1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GC_RBB1,1);
    BL_WR_REG(RF_BASE,RF_TRX_GAIN1,tmpVal);
    rosdac=rf_pri_roscal_iq(1,sram_start_addr,sram_length);
    rf_calib_data->rxcal[1].rosdac_i=rosdac;
    tmpVal=BL_RD_REG(RF_BASE,RF_ROSDAC_CTRL_HW1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROSDAC_I_GC1,rosdac);
    BL_WR_REG(RF_BASE,RF_ROSDAC_CTRL_HW1,tmpVal);  
    rf_pri_printf("rosdac_i_gc1=%ld\r\n",rosdac);       
    rosdac=rf_pri_roscal_iq(0,sram_start_addr,sram_length);
    rf_calib_data->rxcal[1].rosdac_q=rosdac;
    tmpVal=BL_RD_REG(RF_BASE,RF_ROSDAC_CTRL_HW1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROSDAC_Q_GC1,rosdac);
    BL_WR_REG(RF_BASE,RF_ROSDAC_CTRL_HW1,tmpVal);
    rf_pri_printf("rosdac_q_gc1=%ld\r\n",rosdac);                   

    tmpVal=BL_RD_REG(RF_BASE,RF_TRX_GAIN1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GC_RBB1,0);
    BL_WR_REG(RF_BASE,RF_TRX_GAIN1,tmpVal);
    rosdac=rf_pri_roscal_iq(1,sram_start_addr,sram_length);
    rf_calib_data->rxcal[0].rosdac_i=rosdac;
    tmpVal=BL_RD_REG(RF_BASE,RF_ROSDAC_CTRL_HW1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROSDAC_I_GC0,rosdac);
    BL_WR_REG(RF_BASE,RF_ROSDAC_CTRL_HW1,tmpVal);   
    rf_pri_printf("rosdac_i_gc0=%ld\r\n",rosdac);      
    rosdac=rf_pri_roscal_iq(0,sram_start_addr,sram_length);
    rf_calib_data->rxcal[0].rosdac_q=rosdac;
    tmpVal=BL_RD_REG(RF_BASE,RF_ROSDAC_CTRL_HW1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROSDAC_Q_GC0,rosdac);
    BL_WR_REG(RF_BASE,RF_ROSDAC_CTRL_HW1,tmpVal); 
    rf_pri_printf("rosdac_q_gc0=%ld\r\n",rosdac);  
#else
    tmpVal=BL_RD_REG(RF_BASE,RF_TRX_GAIN1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GC_RBB2,6);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_GC_RBB1,3);
    BL_WR_REG(RF_BASE,RF_TRX_GAIN1,tmpVal);

    //***********************************************
    rosdac=rf_pri_roscal_iq(1,sram_start_addr,sram_length);
    
    rf_calib_data->rxcal[3].rosdac_i=rosdac;
    tmpVal=BL_RD_REG(RF_BASE,RF_ROSDAC_CTRL_HW2);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROSDAC_I_GC3,rosdac);
    BL_WR_REG(RF_BASE,RF_ROSDAC_CTRL_HW2,tmpVal); 
    rf_pri_printf("rosdac_i_gc3=%ld\r\n",rosdac);    

    rf_calib_data->rxcal[2].rosdac_i=rosdac;
    tmpVal=BL_RD_REG(RF_BASE,RF_ROSDAC_CTRL_HW2);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROSDAC_I_GC2,rosdac);
    BL_WR_REG(RF_BASE,RF_ROSDAC_CTRL_HW2,tmpVal);
    rf_pri_printf("rosdac_i_gc2=%ld\r\n",rosdac);

    rf_calib_data->rxcal[1].rosdac_i=rosdac;
    tmpVal=BL_RD_REG(RF_BASE,RF_ROSDAC_CTRL_HW1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROSDAC_I_GC1,rosdac);
    BL_WR_REG(RF_BASE,RF_ROSDAC_CTRL_HW1,tmpVal);  
    rf_pri_printf("rosdac_i_gc1=%ld\r\n",rosdac);

    rf_calib_data->rxcal[0].rosdac_i=rosdac;
    tmpVal=BL_RD_REG(RF_BASE,RF_ROSDAC_CTRL_HW1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROSDAC_I_GC0,rosdac);
    BL_WR_REG(RF_BASE,RF_ROSDAC_CTRL_HW1,tmpVal);   
    rf_pri_printf("rosdac_i_gc0=%ld\r\n",rosdac);

    //***********************************************
    rosdac=rf_pri_roscal_iq(0,sram_start_addr,sram_length);

    rf_calib_data->rxcal[3].rosdac_q=rosdac;
    tmpVal=BL_RD_REG(RF_BASE,RF_ROSDAC_CTRL_HW2);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROSDAC_Q_GC3,rosdac);
    BL_WR_REG(RF_BASE,RF_ROSDAC_CTRL_HW2,tmpVal); 
    rf_pri_printf("rosdac_q_gc3=%ld\r\n",rosdac);      

    rf_calib_data->rxcal[2].rosdac_q=rosdac;
    tmpVal=BL_RD_REG(RF_BASE,RF_ROSDAC_CTRL_HW2);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROSDAC_Q_GC2,rosdac);
    BL_WR_REG(RF_BASE,RF_ROSDAC_CTRL_HW2,tmpVal);  
    rf_pri_printf("rosdac_q_gc2=%ld\r\n",rosdac); 

    rf_calib_data->rxcal[1].rosdac_q=rosdac;
    tmpVal=BL_RD_REG(RF_BASE,RF_ROSDAC_CTRL_HW1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROSDAC_Q_GC1,rosdac);
    BL_WR_REG(RF_BASE,RF_ROSDAC_CTRL_HW1,tmpVal);
    rf_pri_printf("rosdac_q_gc1=%ld\r\n",rosdac); 

    rf_calib_data->rxcal[0].rosdac_q=rosdac;
    tmpVal=BL_RD_REG(RF_BASE,RF_ROSDAC_CTRL_HW1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROSDAC_Q_GC0,rosdac);
    BL_WR_REG(RF_BASE,RF_ROSDAC_CTRL_HW1,tmpVal); 
    rf_pri_printf("rosdac_q_gc0=%ld\r\n",rosdac); 

#endif

    rf_pri_restore_state_for_cal();    

    tmpVal=BL_RD_REG(RF_BASE,RFCAL_STATUS);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROS_STATUS,0x3);
    BL_WR_REG(RF_BASE,RFCAL_STATUS,tmpVal);

}

void rf_pri_rccal(void)
{
    uint32_t tmpVal = 0;
    uint32_t rccal_i_status=0;
    uint32_t rccal_q_status=0;

    tmpVal=BL_RD_REG(RF_BASE,RFCAL_STATEEN);
    if(BL_GET_REG_BITS_VAL(tmpVal,RF_RCCAL_STEN)==0)
    {
        tmpVal=BL_RD_REG(RF_BASE,RFCAL_STATUS);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_RCCAL_STATUS,0x0);
        BL_WR_REG(RF_BASE,RFCAL_STATUS,tmpVal);
        return;
    }else
    {
        tmpVal=BL_RD_REG(RF_BASE,RFCAL_STATUS);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_RCCAL_STATUS,0x1);
        BL_WR_REG(RF_BASE,RFCAL_STATUS,tmpVal);        
    }

    rf_pri_save_state_for_cal();

    rf_pri_config_mode(E_RF_MODE_RCCAL);

    tmpVal=BL_RD_REG(RF_BASE,RF_RBB3);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_RBB_BW,2);
    BL_WR_REG(RF_BASE,RF_RBB3,tmpVal);
    tmpVal=BL_RD_REG(RF_BASE,RF_ADDA1);
    tmpVal=BL_SET_REG_BIT(tmpVal,RF_DAC_RCCALSEL);
    BL_WR_REG(RF_BASE,RF_ADDA1,tmpVal);        
    tmpVal=BL_RD_REG(RF_BASE,RFCAL_CTRLEN);
    tmpVal=BL_SET_REG_BIT(tmpVal,RF_RCCAL_EN);
    BL_WR_REG(RF_BASE,RFCAL_CTRLEN,tmpVal);

    rccal_i_status=rf_pri_rccal_iq(1);
    rccal_q_status=rf_pri_rccal_iq(0);

    // configure RBB to use minimum bandwidth
    tmpVal=BL_RD_REG(RF_BASE,RF_RBB2);
    rf_calib_data->cal.rbb_cap1_fc_i = BL_GET_REG_BITS_VAL(tmpVal,RF_RBB_CAP1_FC_I);
    rf_calib_data->cal.rbb_cap1_fc_q = BL_GET_REG_BITS_VAL(tmpVal,RF_RBB_CAP1_FC_Q);
    rf_calib_data->cal.rbb_cap2_fc_i = BL_GET_REG_BITS_VAL(tmpVal,RF_RBB_CAP2_FC_I);
    rf_calib_data->cal.rbb_cap2_fc_q = BL_GET_REG_BITS_VAL(tmpVal,RF_RBB_CAP2_FC_Q);
    rf_pri_printf("rbb_cap1_fc_i=%ld,rbb_cap2_fc_i=%ld,rbb_cap1_fc_q=%ld,rbb_cap2_fc_q=%ld\r\n",
        (uint32_t)(rf_calib_data->cal.rbb_cap1_fc_i),
        (uint32_t)(rf_calib_data->cal.rbb_cap2_fc_i),
        (uint32_t)(rf_calib_data->cal.rbb_cap1_fc_q),
        (uint32_t)(rf_calib_data->cal.rbb_cap2_fc_q));

    /// try to make rbb narrower
    #if 1
    int32_t max_cw, offset_cw, tgt_offset_cw;
    max_cw = rf_calib_data->cal.rbb_cap1_fc_i;
    max_cw = rf_calib_data->cal.rbb_cap1_fc_q > max_cw ? rf_calib_data->cal.rbb_cap1_fc_q : max_cw;
    max_cw = rf_calib_data->cal.rbb_cap2_fc_i > max_cw ? rf_calib_data->cal.rbb_cap2_fc_i : max_cw;
    max_cw = rf_calib_data->cal.rbb_cap2_fc_q > max_cw ? rf_calib_data->cal.rbb_cap2_fc_q : max_cw;
    
    tgt_offset_cw = 24;
    offset_cw = (max_cw + tgt_offset_cw) <= 63 ?  tgt_offset_cw : 63 - max_cw;

    rf_calib_data->cal.rbb_cap1_fc_i += offset_cw;
    rf_calib_data->cal.rbb_cap1_fc_q += offset_cw;
    rf_calib_data->cal.rbb_cap2_fc_i += offset_cw;
    rf_calib_data->cal.rbb_cap2_fc_q += offset_cw;

    rf_pri_rccal_config(1,rf_calib_data->cal.rbb_cap1_fc_i);
    rf_pri_rccal_config(0,rf_calib_data->cal.rbb_cap1_fc_q);

    rf_pri_printf("new rbb_cap1_fc_i=%ld,rbb_cap2_fc_i=%ld,rbb_cap1_fc_q=%ld,rbb_cap2_fc_q=%ld\r\n",
        (uint32_t)(rf_calib_data->cal.rbb_cap1_fc_i),
        (uint32_t)(rf_calib_data->cal.rbb_cap2_fc_i),
        (uint32_t)(rf_calib_data->cal.rbb_cap1_fc_q),
        (uint32_t)(rf_calib_data->cal.rbb_cap2_fc_q));
    #endif
    
    rf_pri_restore_state_for_cal();

    if(rccal_i_status==2 || rccal_q_status==2){
        tmpVal=BL_RD_REG(RF_BASE,RFCAL_STATUS);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_RCCAL_STATUS,0x2);
        BL_WR_REG(RF_BASE,RFCAL_STATUS,tmpVal);         
    }else{
        tmpVal=BL_RD_REG(RF_BASE,RFCAL_STATUS);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_RCCAL_STATUS,0x3);
        BL_WR_REG(RF_BASE,RFCAL_STATUS,tmpVal);        
    }

}

void rf_pri_lo_acal(void)
{
    uint32_t tmpVal;
    uint32_t cw;
    uint32_t bitwidth = 5;
    uint32_t ud;
    uint32_t channel_index;

    tmpVal=BL_RD_REG(RF_BASE,RFCAL_STATUS);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ACAL_STATUS,0x1);
    BL_WR_REG(RF_BASE,RFCAL_STATUS,tmpVal);

    rf_pri_save_state_for_cal();
	
	// Setup for lo acl
    rf_pri_config_mode(E_RF_MODE_LO_ACAL);
    
    for (channel_index = E_RF_CHANNEL_2404M; channel_index <= E_RF_CHANNEL_2484M; channel_index++) {

        tmpVal=BL_RD_REG(RF_BASE,RFCAL_CTRLEN);
        tmpVal=BL_SET_REG_BIT(tmpVal,RF_ACAL_EN);
        BL_WR_REG(RF_BASE,RFCAL_CTRLEN,tmpVal);
        tmpVal=BL_RD_REG(RF_BASE,RF_VCO2);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ACAL_VREF_CW,4);
        BL_WR_REG(RF_BASE,RF_VCO2,tmpVal);
        tmpVal=BL_RD_REG(RF_BASE,RF_VCO1);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_LO_VCO_IDAC_CW,0x10);
        BL_WR_REG(RF_BASE,RF_VCO1,tmpVal);
        //mixed_reg->vco.BF.lo_freq_cw = 0x40;
        tmpVal=BL_RD_REG(RF_BASE,RF_VCO1);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_LO_VCO_FREQ_CW,rf_calib_data->lo[channel_index].fcal);
        BL_WR_REG(RF_BASE,RF_VCO1,tmpVal);
        BL_WR_REG(RF_BASE,RF_SDM2,channel_div_table[xtalType*E_RF_CHANNEL_NUM+channel_index]);
        rf_pri_wait_us(1);

        // Binary search
        cw = 1 << (bitwidth-1);
        for (int i = 1; i < bitwidth; i++) {
            tmpVal=BL_RD_REG(RF_BASE,RF_VCO1);
            tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_LO_VCO_IDAC_CW,cw);
            BL_WR_REG(RF_BASE,RF_VCO1,tmpVal);
            rf_pri_wait_us(1);
            tmpVal=BL_RD_REG(RF_BASE,RF_VCO2);
            ud=BL_GET_REG_BITS_VAL(tmpVal,RF_ACAL_VCO_UD);
            if (ud) {
                cw -= (1 << (bitwidth-1-i));
            }
            else {
                cw += (1 << (bitwidth-1-i));
            }
        }

        // cw++ to make sure ud==1 after cal
        tmpVal=BL_RD_REG(RF_BASE,RF_VCO1);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_LO_VCO_IDAC_CW,cw);
        BL_WR_REG(RF_BASE,RF_VCO1,tmpVal);
        rf_pri_wait_us(1);
        tmpVal=BL_RD_REG(RF_BASE,RF_VCO2);
        ud=BL_GET_REG_BITS_VAL(tmpVal,RF_ACAL_VCO_UD);
        if (ud == 0 && cw < 0x1f) cw++;
        
        // Save calibration result
        rf_calib_data->lo[channel_index].acal = cw;
        rf_pri_printf("%ldth channel,vco_idac_cw=%ld\r\n",channel_index,cw);
    }
		// Restore state
    rf_pri_restore_state_for_cal();
    
    tmpVal=BL_RD_REG(RF_BASE,RFCAL_STATUS);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ACAL_STATUS,0x3);
    BL_WR_REG(RF_BASE,RFCAL_STATUS,tmpVal);
}

void rf_pri_fcal(void)
{
    uint32_t tmpVal = 0;
    int16_t i,n;
    uint16_t cw,cw_binary;
    uint16_t cnt;
    uint16_t bit_width = 8;
    uint16_t count1 = channel_cnt_range[0];//0xa6a0;//0xa700;    //0xa715
    uint16_t count2 = channel_cnt_range[1];//0xa6e0;//0xa779;		//Adjust the lower boundary (0xa780 --> 0xa779)  --Ling 2018/10/13
    uint16_t count3 = channel_cnt_range[2];//0xace0;//0xac8a;		//Adjust the upper boundary (0xac80 --> 0xac8a)  --Ling 2018/10/13

    tmpVal=BL_RD_REG(RF_BASE,RFCAL_STATUS);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_FCAL_STATUS,1);
    BL_WR_REG(RF_BASE,RFCAL_STATUS,tmpVal);
    
    // Save for calibration
    rf_pri_save_state_for_cal();
    
    // Setup for lo fcal
    rf_pri_config_mode(E_RF_MODE_LO_FCAL);
    tmpVal=BL_RD_REG(RF_BASE,RFCAL_CTRLEN);
    tmpVal=BL_SET_REG_BIT(tmpVal,RF_FCAL_EN);
    BL_WR_REG(RF_BASE,RFCAL_CTRLEN,tmpVal);

    cw = 1 << (bit_width-1);
    tmpVal=BL_RD_REG(RF_BASE,RF_VCO1);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_LO_VCO_FREQ_CW,cw);
    BL_WR_REG(RF_BASE,RF_VCO1,tmpVal);
    tmpVal=BL_RD_REG(RF_BASE,RF_FBDV);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_LO_FBDV_SEL_FB_CLK,0);
    BL_WR_REG(RF_BASE,RF_FBDV,tmpVal);
#if XTAL24M
    tmpVal=BL_RD_REG(RF_BASE,RF_VCO3);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_FCAL_DIV,0x500);
    BL_WR_REG(RF_BASE,RF_VCO3,tmpVal);
    //0x500:24M		0x56B:26M		0x6AB:32M		0x800:38.4M		0x855:40M		0xAD5:52M	--Ling 2018/10/13
#endif
#if XTAL26M
    tmpVal=BL_RD_REG(RF_BASE,RF_VCO3);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_FCAL_DIV,0x56B);
    BL_WR_REG(RF_BASE,RF_VCO3,tmpVal);
    //0x500:24M		0x56B:26M		0x6AB:32M		0x800:38.4M		0x855:40M		0xAD5:52M	--Ling 2018/10/13
#endif
#if XTAL32M
    tmpVal=BL_RD_REG(RF_BASE,RF_VCO3);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_FCAL_DIV,0x6AB);
    BL_WR_REG(RF_BASE,RF_VCO3,tmpVal);
    //0x500:24M		0x56B:26M		0x6AB:32M		0x800:38.4M		0x855:40M		0xAD5:52M	--Ling 2018/10/13
#endif
#if XTAL38P4M
    tmpVal=BL_RD_REG(RF_BASE,RF_VCO3);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_FCAL_DIV,0x800);
    BL_WR_REG(RF_BASE,RF_VCO3,tmpVal);
    //0x500:24M		0x56B:26M		0x6AB:32M		0x800:38.4M		0x855:40M		0xAD5:52M	--Ling 2018/10/13
#endif
#if XTAL40M
    tmpVal=BL_RD_REG(RF_BASE,RF_VCO3);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_FCAL_DIV,fcal_div);
    BL_WR_REG(RF_BASE,RF_VCO3,tmpVal);
    //0x500:24M		0x56B:26M		0x6AB:32M		0x800:38.4M		0x855:40M		0xAD5:52M	--Ling 2018/10/13
#endif
#if XTAL52M
    tmpVal=BL_RD_REG(RF_BASE,RF_VCO3);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_FCAL_DIV,0xAD5);
    BL_WR_REG(RF_BASE,RF_VCO3,tmpVal);
    //0x500:24M		0x56B:26M		0x6AB:32M		0x800:38.4M		0x855:40M		0xAD5:52M	--Ling 2018/10/13
#endif
    BL_WR_REG(RF_BASE,RF_SDM2,0x1000000);
    tmpVal=BL_RD_REG(RF_BASE,RF_SDM1);
    tmpVal=BL_SET_REG_BIT(tmpVal,RF_LO_SDM_BYPASS);
    BL_WR_REG(RF_BASE,RF_SDM1,tmpVal);

    tmpVal=BL_RD_REG(RF_BASE,RF_SDM1);
    tmpVal=BL_CLR_REG_BIT(tmpVal,RF_LO_SDM_RSTB);
    BL_WR_REG(RF_BASE,RF_SDM1,tmpVal);
    tmpVal=BL_RD_REG(RF_BASE,RF_FBDV);
    tmpVal=BL_SET_REG_BIT(tmpVal,RF_LO_FBDV_RST);
    BL_WR_REG(RF_BASE,RF_FBDV,tmpVal);
    rf_pri_wait_us(10);
    tmpVal=BL_RD_REG(RF_BASE,RF_SDM1);
    tmpVal=BL_SET_REG_BIT(tmpVal,RF_LO_SDM_RSTB);
    BL_WR_REG(RF_BASE,RF_SDM1,tmpVal);
    tmpVal=BL_RD_REG(RF_BASE,RF_FBDV);
    tmpVal=BL_CLR_REG_BIT(tmpVal,RF_LO_FBDV_RST);
    BL_WR_REG(RF_BASE,RF_FBDV,tmpVal); 
    rf_pri_wait_us(50);

    tmpVal=BL_RD_REG(RF_BASE,RF_VCO2);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_LO_VCO_VBIAS_CW,0x2);
    BL_WR_REG(RF_BASE,RF_VCO2,tmpVal);
    rf_pri_wait_us(50);

    // Binary search to find initial channel
		while(1){
		    for (i = 1; i < bit_width; i++) {
				    cnt = rf_pri_fcal_meas(cw);
                    // rf_pri_printf("cw=%ld, cnt=%ld\r\n",(uint32_t)cw,(uint32_t)cnt);
				    if (cnt >= count1 && cnt <= count2) {
						    break;
				    }
				    else if (cnt < count1) {
						    cw -= 1 << (bit_width-1-i);
				    }
				    else if (cnt > count2) {
						    cw += 1 << (bit_width-1-i);
				    }
		    }
				if (cw < 15) {
            rf_pri_printf("Unexpected cw %ld\r\n",(uint32_t)cw);
            cw = 1 << (bit_width-1);
            i = 1;
            tmpVal=BL_RD_REG(RF_BASE,RF_SDM1);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_LO_SDM_RSTB);
            BL_WR_REG(RF_BASE,RF_SDM1,tmpVal);
            tmpVal=BL_RD_REG(RF_BASE,RF_FBDV);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_LO_FBDV_RST);
            BL_WR_REG(RF_BASE,RF_FBDV,tmpVal);
            rf_pri_wait_us(50);
            tmpVal=BL_RD_REG(RF_BASE,RF_SDM1);
            tmpVal=BL_SET_REG_BIT(tmpVal,RF_LO_SDM_RSTB);
            BL_WR_REG(RF_BASE,RF_SDM1,tmpVal);
            tmpVal=BL_RD_REG(RF_BASE,RF_FBDV);
            tmpVal=BL_CLR_REG_BIT(tmpVal,RF_LO_FBDV_RST);
            BL_WR_REG(RF_BASE,RF_FBDV,tmpVal);
            rf_pri_wait_us(50);
        } else {
            break;
        }
		}
    cw = cw + 1;
    cw_binary = cw;
    cnt = rf_pri_fcal_meas(cw);

    // Store optim cnt table
    channel_cnt_opt_table[0] = cnt;
    for (i = 1; i < CNT_TIMES; i++) {
        cnt = rf_pri_fcal_meas(--cw);
        channel_cnt_opt_table[i] = cnt;
        if (cnt > count3) break;
    } 

    // Find cw for each channel
    for (i = 0,n = 0; i < NUM_CHANNELS; i++) {
        while (channel_cnt_table[i]>channel_cnt_opt_table[n]) n++;
        if ((channel_cnt_table[i]-channel_cnt_opt_table[n-1]) <
            (channel_cnt_opt_table[n]-channel_cnt_table[i])) {
            channel_cw_table[i] = cw_binary - (n-1);
        }
        else {
            channel_cw_table[i] = cw_binary - (n-1);
        }
        n = n > 0 ? n-1 : 0;
    }
    
    // Set osmx register
    //mixed_reg->osmx.BF.osmx_cap = (channel_cw_table[6]>>3)&0b1111;
    
    // Restore state
    rf_pri_restore_state_for_cal();

    // Save calibration result
    for (i = 0; i < NUM_CHANNELS; i++) {
        rf_calib_data->lo[i].fcal = channel_cw_table[i];
        rf_pri_printf("%ldth channel,lo_vco_freq_cw=%ld\r\n",(uint32_t)(i+1),(uint32_t)(channel_cw_table[i]));
    }
    tmpVal=BL_RD_REG(RF_BASE,RFCAL_STATUS);
    tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_FCAL_STATUS,3);
    BL_WR_REG(RF_BASE,RFCAL_STATUS,tmpVal);
}

void rf_pri_full_cal(void)
{
    rf_pri_start_rxdfe();
    rf_pri_start_txdfe();

    rf_pri_fcal();
    rf_pri_lo_acal();
	
    rf_pri_roscal();
    rf_pri_rccal();

    rf_pri_txcal();

    #if 0
    int i;
    for(i=0;i<10;i++){
        rf_pri_txcal();
    }
    #endif

    #if 0
    int32_t i,j,k;
    for(i=0;i<13;i++){
        rf_pri_update_param(2412+i*5);
        MSG("channel=%d,tx_pwr_os=%d\r\n",2412+i*5,tx_pwr_os);
        for(j=0;j<2;j++){
            for(k=10;k<300;k=k+10){
                rf_pri_get_pwr_index(k,j);
                MSG("mode=%d,target pwr=%d,index=%d,dvga=%d\r\n",j,k,tp_index->index,tp_index->dvga);
            }
        }
    }
    #endif

    rf_pri_auto_gain();
    rf_pri_stop_rxdfe();
    rf_pri_stop_txdfe();
}

#if BL602_DBG_RF
    void rf_pri_set_cal_reg(void)
    {
        uint32_t tmpVal = 0;

        tmpVal=BL_RD_REG(RF_BASE,RF_ROSDAC_CTRL_HW1);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROSDAC_I_GC0,32);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROSDAC_Q_GC0,32);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROSDAC_I_GC1,32);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROSDAC_Q_GC1,32);    
        BL_WR_REG(RF_BASE,RF_ROSDAC_CTRL_HW1,tmpVal);
        tmpVal=BL_RD_REG(RF_BASE,RF_ROSDAC_CTRL_HW2);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROSDAC_I_GC2,32);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROSDAC_Q_GC2,32);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROSDAC_I_GC3,32);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_ROSDAC_Q_GC3,32);
        BL_WR_REG(RF_BASE,RF_ROSDAC_CTRL_HW2,tmpVal);
        
        tmpVal=BL_RD_REG(RF_BASE,RF_RXIQ_CTRL_HW1);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_RX_IQ_GAIN_COMP_GC0,1024);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_RX_IQ_PHASE_COMP_GC0,0);    
        BL_WR_REG(RF_BASE,RF_RXIQ_CTRL_HW1,tmpVal);  
        tmpVal=BL_RD_REG(RF_BASE,RF_RXIQ_CTRL_HW2);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_RX_IQ_GAIN_COMP_GC1,1024);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_RX_IQ_PHASE_COMP_GC1,0);
        BL_WR_REG(RF_BASE,RF_RXIQ_CTRL_HW2,tmpVal);
        tmpVal=BL_RD_REG(RF_BASE,RF_RXIQ_CTRL_HW3);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_RX_IQ_GAIN_COMP_GC2,1024);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_RX_IQ_PHASE_COMP_GC2,0);
        BL_WR_REG(RF_BASE,RF_RXIQ_CTRL_HW3,tmpVal);
        tmpVal=BL_RD_REG(RF_BASE,RF_RXIQ_CTRL_HW4);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_RX_IQ_GAIN_COMP_GC3,1024);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_RX_IQ_PHASE_COMP_GC3,0);
        BL_WR_REG(RF_BASE,RF_RXIQ_CTRL_HW4,tmpVal);                  

        // rf_calib_data->cal.rx_i = 0;
        // rf_calib_data->cal.rx_q = 0;
        tmpVal=BL_RD_REG(RF_BASE,RF_RBB2);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_RBB_CAP1_FC_I,32);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_RBB_CAP2_FC_I,32);    
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_RBB_CAP1_FC_Q,32);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_RBB_CAP2_FC_Q,32);
        BL_WR_REG(RF_BASE,RF_RBB2,tmpVal);    

        tmpVal=BL_RD_REG(RF_BASE,RF_TOSDAC_CTRL_HW1);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_I_GC0,32);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_Q_GC0,32);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_I_GC1,32);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_Q_GC1,32);
        BL_WR_REG(RF_BASE,RF_TOSDAC_CTRL_HW1,tmpVal);
        tmpVal=BL_RD_REG(RF_BASE,RF_TOSDAC_CTRL_HW2);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_I_GC2,32);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_Q_GC2,32);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_I_GC3,32);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_Q_GC3,32);
        BL_WR_REG(RF_BASE,RF_TOSDAC_CTRL_HW2,tmpVal); 
        tmpVal=BL_RD_REG(RF_BASE,RF_TOSDAC_CTRL_HW3);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_I_GC4,32);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_Q_GC4,32);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_I_GC5,32);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_Q_GC5,32);
        BL_WR_REG(RF_BASE,RF_TOSDAC_CTRL_HW3,tmpVal);
        tmpVal=BL_RD_REG(RF_BASE,RF_TOSDAC_CTRL_HW4);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_I_GC6,32);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_Q_GC6,32);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_I_GC7,32);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TBB_TOSDAC_Q_GC7,32);
        BL_WR_REG(RF_BASE,RF_TOSDAC_CTRL_HW4,tmpVal);
        
        tmpVal=BL_RD_REG(RF_BASE,RF_TX_IQ_GAIN_HW0);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_GAIN_COMP_GC0,1024);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_PHASE_COMP_GC0,0);    
        BL_WR_REG(RF_BASE,RF_TX_IQ_GAIN_HW0,tmpVal);
        tmpVal=BL_RD_REG(RF_BASE,RF_TX_IQ_GAIN_HW1);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_GAIN_COMP_GC1,1024);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_PHASE_COMP_GC1,0);    
        BL_WR_REG(RF_BASE,RF_TX_IQ_GAIN_HW1,tmpVal);
        tmpVal=BL_RD_REG(RF_BASE,RF_TX_IQ_GAIN_HW2);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_GAIN_COMP_GC2,1024);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_PHASE_COMP_GC2,0);    
        BL_WR_REG(RF_BASE,RF_TX_IQ_GAIN_HW2,tmpVal);
        tmpVal=BL_RD_REG(RF_BASE,RF_TX_IQ_GAIN_HW3);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_GAIN_COMP_GC3,1024);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_PHASE_COMP_GC3,0);    
        BL_WR_REG(RF_BASE,RF_TX_IQ_GAIN_HW3,tmpVal);
        tmpVal=BL_RD_REG(RF_BASE,RF_TX_IQ_GAIN_HW4);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_GAIN_COMP_GC4,1024);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_PHASE_COMP_GC4,0);    
        BL_WR_REG(RF_BASE,RF_TX_IQ_GAIN_HW4,tmpVal);
        tmpVal=BL_RD_REG(RF_BASE,RF_TX_IQ_GAIN_HW5);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_GAIN_COMP_GC5,1024);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_PHASE_COMP_GC5,0);    
        BL_WR_REG(RF_BASE,RF_TX_IQ_GAIN_HW5,tmpVal);
        tmpVal=BL_RD_REG(RF_BASE,RF_TX_IQ_GAIN_HW6);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_GAIN_COMP_GC6,1024);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_PHASE_COMP_GC6,0);    
        BL_WR_REG(RF_BASE,RF_TX_IQ_GAIN_HW6,tmpVal);
        tmpVal=BL_RD_REG(RF_BASE,RF_TX_IQ_GAIN_HW7);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_GAIN_COMP_GC7,1024);
        tmpVal=BL_SET_REG_BITS_VAL(tmpVal,RF_TX_IQ_PHASE_COMP_GC7,0);    
        BL_WR_REG(RF_BASE,RF_TX_IQ_GAIN_HW7,tmpVal);
        // rf_calib_data->cal.tx_dc_comp_i = 0;
        // rf_calib_data->cal.tx_dc_comp_q = 0;
    }
#endif

/*@} end of group L1C_Public_Functions */

/*@} end of group L1C */

/*@} end of group BL602_Peripheral_Driver */
