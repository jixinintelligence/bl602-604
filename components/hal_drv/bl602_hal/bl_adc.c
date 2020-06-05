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
#include <stdint.h>
#include <string.h>

#if 0
#include <device/vfs_uart.h>
#include <vfs_err.h>
#include <vfs_register.h>
#include <hal/soc/uart.h>
#include <aos/kernel.h>
#endif

#include "bl602_aon.h"
#include "bl602_common.h"
#include "bl602_glb.h"
#include "bl602_hbn.h"
#include "bl602_dma.h"
#include "bl602_adc.h"

#include <utils_log.h>

//#define TEMP_OFFSET_X   22791
#define TEMP_OFFSET_X 2318

ADC_CFG_Type adcCfg = {
    .v18Sel=ADC_V18_SEL_1P82V,                /*!< ADC 1.8V select */
    .v11Sel=ADC_V11_SEL_1P1V,                 /*!< ADC 1.1V select */
    .clkDiv=ADC_CLK_DIV_16,                   /*!< Clock divider */
    .gain1=ADC_PGA_GAIN_NONE,                 /*!< PGA gain 1 */
    .gain2=ADC_PGA_GAIN_NONE,                 /*!< PGA gain 2 */
    .chopMode=ADC_CHOP_MOD_AZ_PGA_ON,           /*!< ADC chop mode select */
    .biasSel=ADC_BIAS_SEL_MAIN_BANDGAP,       /*!< ADC current form main bandgap or aon bandgap */
    .vcm=ADC_PGA_VCM_1V,                      /*!< ADC VCM value */
    .vref=ADC_VREF_2V,                      /*!< ADC voltage reference */
    .inputMode=ADC_INPUT_SINGLE_END,          /*!< ADC input signal type */
    .resWidth=ADC_DATA_WIDTH_16_WITH_256_AVERAGE,              /*!< ADC resolution and oversample rate */
    .offsetCalibEn=0,                         /*!< Offset calibration enable */
    .offsetCalibVal=0,                        /*!< Offset calibration value */
};


ADC_FIFO_Cfg_Type adcFifoCfg = {
    .fifoThreshold = ADC_FIFO_THRESHOLD_1,
    .dmaEn = DISABLE,
};

void ADC_Clock_Init(uint8_t div)
{
    GLB_Set_ADC_CLK(ENABLE,GLB_ADC_CLK_96M,div);
    log_info("GLB_Set_ADC_CLK_Div(%d) == clock 96M/(%d+1)\r\n",div,div);
}

void TSEN_Calibration(void)
{
    ADC_SET_TSVBE_LOW();
    ADC_Start();
    ARCH_Delay_MS(100);
    while(ADC_Get_FIFO_Count() == 0);
    ADC_Read_FIFO();

    ADC_SET_TSVBE_HIGH();
    ADC_Start();
    ARCH_Delay_MS(100);
    while(ADC_Get_FIFO_Count() == 0);
    ADC_Read_FIFO();

    ADC_SET_TSVBE_LOW();
}

static void ADC_tsen_case(void)
{
	ADC_Result_Type result;
	uint32_t regVal=0;
    uint8_t i=0;
    uint32_t v0=0,v1=0;
    float v_error=0;

    ADC_Reset();

    ADC_Disable();
    ADC_Enable();

    ADC_Init(&adcCfg);
    ADC_Channel_Config(ADC_CHAN_TSEN_P,ADC_CHAN_GND,0);
    ADC_Tsen_Init(ADC_TSEN_MOD_INTERNAL_DIODE);

    ADC_FIFO_Cfg(&adcFifoCfg);
    TSEN_Calibration();

	printf("\ntsen test case\n");

	for(i=0;i<40;i++){
		ADC_Start();

		while(ADC_Get_FIFO_Count() == 0);

		do{
			regVal = ADC_Read_FIFO();
			ADC_Parse_Result(&regVal,1,&result);

            if(i%2 ==0){
                v0 = result.value;
            }else{
                v1 = result.value;
            }  
		}while(ADC_Get_FIFO_Count() != 0);

        if(i%2 !=0){
              v_error = (float)v0 - (float)v1;
              v_error = v_error - TEMP_OFFSET_X;
              v_error = v_error /7.753;
            printf(" v0=%ld  v1 = %ld \n",v0,v1);
            //MSG(" ((v0-v1)-X)/7.753= %d \n",(uint32_t)(v_error * 1000));
            printf(" chip Tempture = %ld degree centigrade\n",(uint32_t)(v_error));
        }
        if( i%2 ==0 )
            ADC_SET_TSVBE_HIGH();
        else
            ADC_SET_TSVBE_LOW();

		ARCH_Delay_MS(500);
	}
}


int test_adc_init(void)
{
    ADC_Clock_Init(2);

    ADC_Reset();

    ADC_Disable();
    ADC_Enable();

    ADC_Init(&adcCfg);
    ADC_Channel_Config(ADC_CHAN_TSEN_P, ADC_CHAN_GND, 0);
    ADC_Tsen_Init(ADC_TSEN_MOD_INTERNAL_DIODE);

    ADC_FIFO_Cfg(&adcFifoCfg);
    TSEN_Calibration();

    return 0;
}

int test_adc_get(int16_t *tmp)
{
    ADC_Result_Type result;
    uint32_t regVal=0;
    uint8_t i=0;
    uint32_t v0=0,v1=0;
    float v_error=0;

    for (i = 0; i < 2; i++) {
        ADC_Start();

        while(ADC_Get_FIFO_Count() == 0) {
            extern void aos_msleep(int ms);
            aos_msleep(1);
        }

        do{
            regVal = ADC_Read_FIFO();
            ADC_Parse_Result(&regVal,1,&result);

            if(i%2 == 0) {
                v0 = result.value;
            } else {
                v1 = result.value;
            }
        } while(ADC_Get_FIFO_Count() != 0);

        if (i%2 != 0) {
            v_error = (float)v0 - (float)v1;
            //log_info("v_error = %d\r\n", v_error);
            v_error = v_error - TEMP_OFFSET_X;
            v_error = v_error / 7.753;
            //log_info("v0 = %ld, v1 = %ld\r\n", v0, v1);
            //log_info(" chip Tempture = %ld degree centigrade\r\n", (uint32_t)(v_error));
            *tmp = (int16_t)v_error;
        }
        if (i%2 == 0) {
            ADC_SET_TSVBE_HIGH();
        } else {
            ADC_SET_TSVBE_LOW();
        }
    }
    return 0;
}

int test_adc_test(void)
{
    ADC_tsen_case();
    return 0;
}
