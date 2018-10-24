/*
 * Copyright (c) 2017 Redpine Signals Inc. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 	1. Redistributions of source code must retain the above copyright
 * 	   notice, this list of conditions and the following disclaimer.
 *
 * 	2. Redistributions in binary form must reproduce the above copyright
 * 	   notice, this list of conditions and the following disclaimer in the
 * 	   documentation and/or other materials provided with the distribution.
 *
 * 	3. Neither the name of the copyright holder nor the names of its
 * 	   contributors may be used to endorse or promote products derived from
 * 	   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION). HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __OENBOX_BOOTPARAMS_HEADER_H__
#define __OENBOX_BOOTPARAMS_HEADER_H__

#include "wlan_common.h" 

#define CRYSTAL_GOOD_TIME                BIT(0)
#define BOOTUP_MODE_INFO                 BIT(1) 
#define DIGITAL_LOOP_BACK_PARAMS         BIT(2)
#define RTLS_TIMESTAMP_EN                BIT(3) 
#define HOST_SPI_INTR_CFG                BIT(4)
#define WIFI_TAPLL_CONFIGS               BIT(5)
#define WIFI_PLL960_CONFIGS              BIT(6)
#define WIFI_AFEPLL_CONFIGS              BIT(7)
#define WIFI_SWITCH_CLK_CONFIGS          BIT(8) 
#define BT_TAPLL_CONFIGS                 BIT(9)
#define BT_PLL960_CONFIGS                BIT(10)
#define BT_AFEPLL_CONFIGS                BIT(11)
#define BT_SWITCH_CLK_CONFIGS            BIT(12)
#define ZB_TAPLL_CONFIGS                 BIT(13)
#define ZB_PLL960_CONFIGS                BIT(14)
#define ZB_AFEPLL_CONFIGS                BIT(15)
#define ZB_SWITCH_CLK_CONFIGS            BIT(16)
#define BUCKBOOST_WAIT_INFO              BIT(17)
#define PMU_WAKEUP_SHUTDOWN_W            BIT(18)
#define WDT_PROG_VALUES                  BIT(19)
#define WDT_RESET_DELAY_VALUE            BIT(20)
#define DCDC_OPERATION_MODE_VALID        BIT(21) 
#define PMU_SLP_CLKOUT_SEL               BIT(22)
#define SOC_RESET_WAIT_CNT               BIT(23)


#define TA_PLL_DIS_20            BIT(5)
#define TA_PLL_M_VAL_20          9         
#define TA_PLL_N_VAL_20          0        
#define TA_PLL_P_VAL_20          4        //! 80mhz 8 = 40
                                      
#define PLL960_DIS_20            (0x9 << 4)          
#define PLL960_M_VAL_20          0x14    
#define PLL960_N_VAL_20          0    
#define PLL960_P_VAL_20          5

#define BBP_CLK_DIV_FAC_20       1 
#define LMAC_CLK_DIV_FAC_20      1  
#define AFE_CLK_DIV_FAC_20       1 
#define BB_LMAC_160_80_20        BIT(12)
#define CLK_2_SPARE_GATES_20     BIT(14) | BIT(13)

// umac_clk is calculated as (M + 1)*40 /(N+1)*(P+1)
#define UMAC_CLK_20BW			(((TA_PLL_M_VAL_20 +1)*40)/((TA_PLL_N_VAL_20 + 1)*(TA_PLL_P_VAL_20 + 1)))
#define VALID_20 				(WIFI_TAPLL_CONFIGS | WIFI_PLL960_CONFIGS | WIFI_AFEPLL_CONFIGS | WIFI_SWITCH_CLK_CONFIGS | BOOTUP_MODE_INFO | CRYSTAL_GOOD_TIME)

// umac_clk is calculated as (M + 1)*40 /(N+1)*(P+1)
#define UMAC_CLK_40BW			(((TA_PLL_M_VAL_40 +1)*40)/((TA_PLL_N_VAL_40 + 1)*(TA_PLL_P_VAL_40 + 1)))
#define VALID_40 				(WIFI_PLL960_CONFIGS | WIFI_AFEPLL_CONFIGS | WIFI_SWITCH_CLK_CONFIGS | \
								 WIFI_TAPLL_CONFIGS | CRYSTAL_GOOD_TIME | BOOTUP_MODE_INFO)
#define UMAC_CLK_40MHZ			80
static BOOTUP_PARAMETERS boot_params_10 =
{
  .magic_number = 0x5aa5,
  .crystal_good_time = 0x0,
  .valid = VALID_20,//0x1c0,
  .reserved_for_valids = 0x0,
  .bootup_mode_info = 0x0,
  .digital_loop_back_params = 0x0,
  .rtls_timestamp_en = 0x0,
  .host_spi_intr_cfg = 0x0,
  .device_clk_info = {
    //! Wifi params
    {
      .pll_config_g = {
        .tapll_info_g = {
	   .tapll_config_register_info = ONEBOX_CPU_TO_LE32(( RSI_CONFIG_REGISTER_LOWER & ((TA_PLL_N_VAL_20 << 8) | TA_PLL_M_VAL_20)) | (RSI_CONFIG_REGISTER_UPPER & (TA_PLL_P_VAL_20 << 16))),
        },
        .pll960_info_g = {
          .pll_config_register_1 = ONEBOX_CPU_TO_LE32( (RSI_CONFIG_REGISTER_LOWER & (((PLL960_P_VAL_20 << 8) | PLL960_N_VAL_20))) | (RSI_CONFIG_REGISTER_UPPER & (PLL960_M_VAL_20 << 16))  ) ,
	  .pll_config_register_2 = ONEBOX_CPU_TO_LE16( RSI_CONFIG_REGISTER_LOWER & 0x0 ),
        },
        .afepll_info_g = {
          .pll_config_register =ONEBOX_CPU_TO_LE16( RSI_CONFIG_REGISTER_LOWER & 0x9f0),
        }
      },
      .switch_clk_g = {
       	.switch_clk_info = ONEBOX_CPU_TO_LE32(RSI_SWITCH_UMAC_CLK | RSI_SWITCH_QSPI_CLK | RSI_SWITCH_BBP_LMAC_CLK_REG  | 0x022200000),
	.umac_clock_reg_config = 0x48,
        .qspi_uart_clock_reg_config = 0x1211
      }
    },
    //! Bluetooth params
    {
      .pll_config_g = {
        .tapll_info_g = {
	  .tapll_config_register_info = ONEBOX_CPU_TO_LE32(( RSI_CONFIG_REGISTER_LOWER & ((TA_PLL_N_VAL_20 << 8) | TA_PLL_M_VAL_20)) | (RSI_CONFIG_REGISTER_UPPER & (TA_PLL_P_VAL_20 << 16))),
        },
        .pll960_info_g = {
          .pll_config_register_1 = ONEBOX_CPU_TO_LE32( (RSI_CONFIG_REGISTER_LOWER & (((PLL960_P_VAL_20 << 8) | PLL960_N_VAL_20))) | (RSI_CONFIG_REGISTER_UPPER & (PLL960_M_VAL_20 << 16))  ) ,
	  .pll_config_register_2 = ONEBOX_CPU_TO_LE16( RSI_CONFIG_REGISTER_LOWER & 0x0 ),
        },
        .afepll_info_g = {
          .pll_config_register =ONEBOX_CPU_TO_LE16( RSI_CONFIG_REGISTER_LOWER & 0x9f0),
        }
      },
      .switch_clk_g = {
        .switch_clk_info   = 0x0,
	.umac_clock_reg_config = 0x0,
        .qspi_uart_clock_reg_config = 0x0
      }
    },
    //! Zigbee params
    {
      .pll_config_g = {
        .tapll_info_g = {
	    .tapll_config_register_info = ONEBOX_CPU_TO_LE32(( RSI_CONFIG_REGISTER_LOWER & ((TA_PLL_N_VAL_20 << 8) | TA_PLL_M_VAL_20)) | (RSI_CONFIG_REGISTER_UPPER & (TA_PLL_P_VAL_20 << 16))),
        },
        .pll960_info_g = {
	     .pll_config_register_1 = ONEBOX_CPU_TO_LE32( (RSI_CONFIG_REGISTER_LOWER & (((PLL960_P_VAL_20 << 8) | PLL960_N_VAL_20) | (RSI_CONFIG_REGISTER_UPPER & (PLL960_M_VAL_20 << 16))  )) ),
	     .pll_config_register_2 = ONEBOX_CPU_TO_LE16( RSI_CONFIG_REGISTER_LOWER & 0x0 ),
        
},
        .afepll_info_g = {
          .pll_config_register =ONEBOX_CPU_TO_LE16( RSI_CONFIG_REGISTER_LOWER & 0x9f0),
        }
      },
      .switch_clk_g = {
       	  .switch_clk_info = 0x0,
	  .umac_clock_reg_config = 0x0,
          .qspi_uart_clock_reg_config = 0x0
      }
    }
  },
  //! ULP Params
  .buckboost_wakeup_cnt = 0x0,
  .pmu_wakeup_wait = 0x0,
  .shutdown_wait_time = 0x0,
  .pmu_slp_clkout_sel = 0x0,
  .wdt_prog_value = 0x0,
  .wdt_soc_rst_delay = 0x0,
  .dcdc_operation_mode = 0x0, 
  .soc_reset_wait_cnt = 0x0,
  .waiting_time_at_fresh_sleep = 0x0, /* do not change; will be handled in f/w */
  .max_threshold_to_avoid_sleep = 0x0, /* do not change; will be handled in f/w */
  .beacon_resedue_alg_en = 0, /* do not change; will be handled in f/w */
};

#ifdef MODE_11AH

#define VALID_4 				(WIFI_TAPLL_CONFIGS | WIFI_PLL960_CONFIGS | WIFI_AFEPLL_CONFIGS | WIFI_SWITCH_CLK_CONFIGS | BOOTUP_MODE_INFO | CRYSTAL_GOOD_TIME)
#define VALID_2 				(WIFI_TAPLL_CONFIGS | WIFI_PLL960_CONFIGS | WIFI_AFEPLL_CONFIGS | WIFI_SWITCH_CLK_CONFIGS | BOOTUP_MODE_INFO | CRYSTAL_GOOD_TIME)




BOOTUP_PARAMETERS boot_params_4 ={
	.magic_number = 0x5aa5,
	.crystal_good_time = 0x0,
	.valid = VALID_4,//0x1c0,
	.reserved_for_valids = 0x0,
	.bootup_mode_info = 0x0,
	.digital_loop_back_params = 0x0,
	.rtls_timestamp_en = 0x0,
	.host_spi_intr_cfg = 0x0,
	.device_clk_info = {
		//! Wifi params
		{
			.pll_config_g = {

				.modem_pll_info_g = {
					.pll_ctrl_set_reg = 0xd518,
					.pll_ctrl_clr_reg = 0x2ae7,
					.pll_modem_conig_reg = 0x0110,
					//.pll_modem_conig_reg = 0x1000,
					.soc_clk_config_reg = 0x060c,
					.adc_dac_strm1_config_reg = 0x0110,
					//.adc_dac_strm1_config_reg = 0x1100,
					.adc_dac_strm2_config_reg = 0x6600,
				}



			},
			.switch_clk_g = {
				.switch_tass_clk = 0x1,
				.switch_qspi_clk = 0x0,
				.switch_slp_clk_2_32 = 0x0,
				.switch_wlan_bbp_lmac_clk_reg = 0x1,
				.switch_zbbt_bbp_lmac_clk_reg = 0x0,
				.switch_bbp_lmac_clk_reg = 0x1,
				.reserved = 0x0,
				.tass_clock_reg = 0x08200503,
				.wlan_bbp_lmac_clk_reg_val = 0x104A000,
				//.wlan_bbp_lmac_clk_reg_val = 0x01042000,
				.zbbt_bbp_lmac_clk_reg_val = 0x02010001,
				.bbp_lmac_clk_en_val = 0x0000003b,
			}
		},
	},
	//! ULP Params
	.buckboost_wakeup_cnt = 0x0,
	.pmu_wakeup_wait = 0x0,
	.shutdown_wait_time = 0x0,
	.pmu_slp_clkout_sel = 0x0,
	.wdt_prog_value = 0x0,
	.wdt_soc_rst_delay = 0x0,
	.dcdc_operation_mode = 0x0, 
	.soc_reset_wait_cnt = 0x0,
	.waiting_time_at_fresh_sleep = 0x0, /* do not change; will be handled in f/w */
	.max_threshold_to_avoid_sleep = 0x0, /* do not change; will be handled in f/w */
	.beacon_resedue_alg_en = 0, /* do not change; will be handled in f/w */

};

BOOTUP_PARAMETERS boot_params_2 ={
	.magic_number = 0x5aa5,
	.crystal_good_time = 0x0,
	.valid = VALID_2,//0x1c0,
	.reserved_for_valids = 0x0,
	.bootup_mode_info = 0x0,
	.digital_loop_back_params = 0x0,
	.rtls_timestamp_en = 0x0,
	.host_spi_intr_cfg = 0x0,
	.device_clk_info = {
		//! Wifi params
		{
			.pll_config_g = {
				.modem_pll_info_g = {
					.pll_ctrl_set_reg = 0xd518,
					.pll_ctrl_clr_reg = 0x2ae7,
					.pll_modem_conig_reg = 0x0111,
					//.pll_modem_conig_reg = 0x1000,
					.soc_clk_config_reg = 0x060c,
					.adc_dac_strm1_config_reg = 0x0111,
					//.adc_dac_strm1_config_reg = 0x1100,
					.adc_dac_strm2_config_reg = 0x6600,
				}
			},
			.switch_clk_g = {
				.switch_tass_clk = 0x1,
				.switch_qspi_clk = 0x0,
				.switch_slp_clk_2_32 = 0x0,
				.switch_wlan_bbp_lmac_clk_reg = 0x1,
				.switch_zbbt_bbp_lmac_clk_reg = 0x0,
				.switch_bbp_lmac_clk_reg = 0x1,
				.reserved = 0x0,
				.tass_clock_reg = 0x08200503,
				.wlan_bbp_lmac_clk_reg_val = 0x1052000,
				//.wlan_bbp_lmac_clk_reg_val = 0x01042000,
				.zbbt_bbp_lmac_clk_reg_val = 0x02010001,
				.bbp_lmac_clk_en_val = 0x0000003b,
			}
		},
	},
	//! ULP Params
	.buckboost_wakeup_cnt = 0x0,
	.pmu_wakeup_wait = 0x0,
	.shutdown_wait_time = 0x0,
	.pmu_slp_clkout_sel = 0x0,
	.wdt_prog_value = 0x0,
	.wdt_soc_rst_delay = 0x0,
	.dcdc_operation_mode = 0x0, 
	.soc_reset_wait_cnt = 0x0,
	.waiting_time_at_fresh_sleep = 0x0, /* do not change; will be handled in f/w */
	.max_threshold_to_avoid_sleep = 0x0, /* do not change; will be handled in f/w */
	.beacon_resedue_alg_en = 0, /* do not change; will be handled in f/w */

};





#endif


static BOOTUP_PARAMETERS_9116 boot_params_9116_20 =
{
  .magic_number = 0x5aa5,
  .crystal_good_time = 0x0,
  .valid = VALID_20,//0x1c0,
  .reserved_for_valids = 0x0,
  .bootup_mode_info = 0x0,
  .digital_loop_back_params = 0x0,
  .rtls_timestamp_en = 0x0,
  .host_spi_intr_cfg = 0x0,
  .device_clk_info_9116 = {
    //! Wifi params
    {
      .pll_config_9116_g = {
								.modem_pll_info_g = {
												.pll_ctrl_set_reg = 0xd518,
												.pll_ctrl_clr_reg = 0x2ae7,
												.pll_modem_conig_reg = 0x2000,
												.soc_clk_config_reg = 0x0C18,
												.adc_dac_strm1_config_reg = 0x1100,
												.adc_dac_strm2_config_reg = 0x6600,
        }
     },
      .switch_clk_9116_g = {
									.switch_tass_clk = 0x1,
									.switch_qspi_clk = 0x0,
									.switch_slp_clk_2_32 = 0x0,
									.switch_wlan_bbp_lmac_clk_reg = 0x1,
									.switch_zbbt_bbp_lmac_clk_reg = 0x0,
									.switch_bbp_lmac_clk_reg = 0x1,
									.modem_clk_is_160mhz = 0x0,
									.reserved = 0x0,
									.tass_clock_reg = 0x083C0503,
									.wlan_bbp_lmac_clk_reg_val = 0x01042001,
									//.wlan_bbp_lmac_clk_reg_val = 0x01041001, //! use this division factor once 9116 chip comes.
									.zbbt_bbp_lmac_clk_reg_val = 0x02010001,
									.bbp_lmac_clk_en_val = 0x0000003b,
      }
    },
 },
  //! ULP Params
  .buckboost_wakeup_cnt = 0x0,
  .pmu_wakeup_wait = 0x0,
  .shutdown_wait_time = 0x0,
  .pmu_slp_clkout_sel = 0x0,
  .wdt_prog_value = 0x0,
  .wdt_soc_rst_delay = 0x0,
  .dcdc_operation_mode = 0x0, 
  .soc_reset_wait_cnt = 0x0,
  .waiting_time_at_fresh_sleep = 0x0, /* do not change; will be handled in f/w */
  .max_threshold_to_avoid_sleep = 0x0, /* do not change; will be handled in f/w */
  .beacon_resedue_alg_en = 0, /* do not change; will be handled in f/w */
};



static BOOTUP_PARAMETERS boot_params_20 =
{
  .magic_number = ONEBOX_CPU_TO_LE16(0x5aa5),
  .crystal_good_time = ONEBOX_CPU_TO_LE16(0x0),
  .valid = ONEBOX_CPU_TO_LE32(VALID_20),//0x1c0,
  .reserved_for_valids = 0x0,
  .bootup_mode_info = 0x0,
  .digital_loop_back_params = 0x0,
  .rtls_timestamp_en = 0x0,
  .host_spi_intr_cfg = 0x0,
  .device_clk_info = {
    //! Wifi params
    {
      .pll_config_g = {
        .tapll_info_g = {
	    .tapll_config_register_info = ONEBOX_CPU_TO_LE32(( RSI_CONFIG_REGISTER_LOWER & ((TA_PLL_N_VAL_20 << 8) | TA_PLL_M_VAL_20)) | (RSI_CONFIG_REGISTER_UPPER & (TA_PLL_P_VAL_20 << 16))),
        },
        .pll960_info_g = {
          .pll_config_register_1 = ONEBOX_CPU_TO_LE32( (RSI_CONFIG_REGISTER_LOWER & (((PLL960_P_VAL_20 << 8) | PLL960_N_VAL_20))) | (RSI_CONFIG_REGISTER_UPPER & (PLL960_M_VAL_20 << 16))  ) ,
	  .pll_config_register_2 = ONEBOX_CPU_TO_LE16( RSI_CONFIG_REGISTER_LOWER & 0x0 ),
        },
        .afepll_info_g = {
          .pll_config_register =ONEBOX_CPU_TO_LE16( RSI_CONFIG_REGISTER_LOWER & 0x9f0),
        }
      },
      .switch_clk_g = {
       	 .switch_clk_info = ONEBOX_CPU_TO_LE32(RSI_SWITCH_UMAC_CLK | RSI_SWITCH_QSPI_CLK | RSI_SWITCH_BBP_LMAC_CLK_REG | 0x01110000),
         .umac_clock_reg_config = ONEBOX_CPU_TO_LE16(0x48),
         .qspi_uart_clock_reg_config = ONEBOX_CPU_TO_LE16(0x1211)
      }
    },
    //! Bluetooth params
    {
      .pll_config_g = {
        .tapll_info_g = {
	    .tapll_config_register_info = ONEBOX_CPU_TO_LE32(( RSI_CONFIG_REGISTER_LOWER & ((TA_PLL_N_VAL_20 << 8) | TA_PLL_M_VAL_20)) | (RSI_CONFIG_REGISTER_UPPER & (TA_PLL_P_VAL_20 << 16))),

        },
        .pll960_info_g = {
          .pll_config_register_1 = ONEBOX_CPU_TO_LE32( (RSI_CONFIG_REGISTER_LOWER & (((PLL960_P_VAL_20 << 8) | PLL960_N_VAL_20))) | (RSI_CONFIG_REGISTER_UPPER & (PLL960_M_VAL_20 << 16))  ) ,
	  .pll_config_register_2 = ONEBOX_CPU_TO_LE16( RSI_CONFIG_REGISTER_LOWER & 0x0 ),
        },
        .afepll_info_g = {
          .pll_config_register =ONEBOX_CPU_TO_LE16( RSI_CONFIG_REGISTER_LOWER & 0x9f0),
        }
      },
      .switch_clk_g = {
        .switch_clk_info = 0x0,
	.umac_clock_reg_config = 0x0,
        .qspi_uart_clock_reg_config = 0x0
      }
    },
    //! Zigbee params
    {
      .pll_config_g = {
        .tapll_info_g = {
	    .tapll_config_register_info = ONEBOX_CPU_TO_LE32(( RSI_CONFIG_REGISTER_LOWER & ((TA_PLL_N_VAL_20 << 8) | TA_PLL_M_VAL_20)) | (RSI_CONFIG_REGISTER_UPPER & (TA_PLL_P_VAL_20 << 16))),
        },
        .pll960_info_g = {
          .pll_config_register_1 = ONEBOX_CPU_TO_LE32( (RSI_CONFIG_REGISTER_LOWER & (((PLL960_P_VAL_20 << 8) | PLL960_N_VAL_20) | (RSI_CONFIG_REGISTER_UPPER & (PLL960_M_VAL_20 << 16))  )) ),
	.pll_config_register_2 = ONEBOX_CPU_TO_LE16( RSI_CONFIG_REGISTER_LOWER & 0x0 ),

        },
        .afepll_info_g = {
          .pll_config_register =ONEBOX_CPU_TO_LE16( RSI_CONFIG_REGISTER_LOWER & 0x9f0),
        }
      },
      .switch_clk_g = {
        .switch_clk_info = 0x0,
        .umac_clock_reg_config = 0x0,
        .qspi_uart_clock_reg_config = 0x0
      }
    }
  },
  //! ULP Params
  .buckboost_wakeup_cnt = 0x0,
  .pmu_wakeup_wait = 0x0,
  .shutdown_wait_time = 0x0,
  .pmu_slp_clkout_sel = 0x0,
  .wdt_prog_value = 0x0,
  .wdt_soc_rst_delay = 0x0,
  .dcdc_operation_mode = 0x0, 
  .soc_reset_wait_cnt = 0x0,
  .waiting_time_at_fresh_sleep = 0x0, /* do not change; will be handled in f/w */
  .max_threshold_to_avoid_sleep = 0x0, /* do not change; will be handled in f/w */
  .beacon_resedue_alg_en = 0, /* do not change; will be handled in f/w */
};

///
//  40MHz bootup_params with TA 80MHz
////////////
#define TA_PLL_DIS_40            BIT(5)
#define TA_PLL_M_VAL_40          9         
#define TA_PLL_N_VAL_40          0        
#define TA_PLL_P_VAL_40          4        //! 80mhz 8 = 40
                                      
#define PLL960_DIS_40            (0x9 << 4)          
#define PLL960_M_VAL_40          0x14    
#define PLL960_N_VAL_40          0    
#define PLL960_P_VAL_40          5

#define BBP_CLK_DIV_FAC_40       1 
#define LMAC_CLK_DIV_FAC_40      1  
#define AFE_CLK_DIV_FAC_40       1 
#define BB_LMAC_160_80_40        BIT(12)
#define CLK_2_SPARE_GATES_40     BIT(14) | BIT(13)



static BOOTUP_PARAMETERS_9116 boot_params_9116_40 =
{
  .magic_number = 0x5aa5,
  .crystal_good_time = 0x0,
  .valid = VALID_40,//0x1e3,
  .reserved_for_valids = 0x0,
  .bootup_mode_info = 0x0,
  .digital_loop_back_params = 0x0,
  .rtls_timestamp_en = 0x0,
  .host_spi_intr_cfg = 0x0,
  .device_clk_info_9116 = {
    //! Wifi params
    {
      .pll_config_9116_g = {
								.modem_pll_info_g = {
												.pll_ctrl_set_reg = 0xd518,
												.pll_ctrl_clr_reg = 0x2ae7,
												.pll_modem_conig_reg = 0x3000,
												.soc_clk_config_reg = 0x0C18,
												.adc_dac_strm1_config_reg = 0x0000,
												.adc_dac_strm2_config_reg = 0x6600,
        }
      },
      .switch_clk_9116_g = {
									.switch_tass_clk = 0x1,
									.switch_qspi_clk = 0x0,
									.switch_slp_clk_2_32 = 0x0,
									.switch_wlan_bbp_lmac_clk_reg = 0x1,
									.switch_zbbt_bbp_lmac_clk_reg = 0x0,
									.switch_bbp_lmac_clk_reg = 0x1,
									.modem_clk_is_160mhz = 0x1,
									.reserved = 0x0,
									.tass_clock_reg = 0x083C0503,
#ifdef CONFIG_FPGA_40MHZ
									.wlan_bbp_lmac_clk_reg_val = 0x01081002,
#else
									.wlan_bbp_lmac_clk_reg_val = 0x01042002,
#endif
									.zbbt_bbp_lmac_clk_reg_val = 0x04010002,
									.bbp_lmac_clk_en_val = 0x0000003b,

      }
    },
  },
  //! ULP Params
  .buckboost_wakeup_cnt = 0x0,
  .pmu_wakeup_wait = 0x0,
  .shutdown_wait_time = 0x0,
  .pmu_slp_clkout_sel = 0x0,
  .wdt_prog_value = 0x0,
  .wdt_soc_rst_delay = 0x0,
  .dcdc_operation_mode = 0x0, 
  .soc_reset_wait_cnt = 0x0,
  .waiting_time_at_fresh_sleep = 0x0, /* do not change; will be handled in f/w */
  .max_threshold_to_avoid_sleep = 0x0, /* do not change; will be handled in f/w */
  .beacon_resedue_alg_en = 0, /* do not change; will be handled in f/w */
};

static BOOTUP_PARAMETERS boot_params_40 =
{
  .magic_number = ONEBOX_CPU_TO_LE16(0x5aa5),
  .crystal_good_time = 0x0,
  .valid =ONEBOX_CPU_TO_LE16( VALID_40),//0x1e3,
  .reserved_for_valids = 0x0,
  .bootup_mode_info = 0x0,
  .digital_loop_back_params = 0x0,
  .rtls_timestamp_en = 0x0,
  .host_spi_intr_cfg = 0x0,
  .device_clk_info = {
    //! Wifi params
    {
      .pll_config_g = {
        .tapll_info_g = {
	    .tapll_config_register_info = ONEBOX_CPU_TO_LE32(( RSI_CONFIG_REGISTER_LOWER & ((TA_PLL_N_VAL_40 << 8) | TA_PLL_M_VAL_40)) | (RSI_CONFIG_REGISTER_UPPER & (TA_PLL_P_VAL_40 << 16))),
        },
        .pll960_info_g = {
		.pll_config_register_1 = ONEBOX_CPU_TO_LE32( (RSI_CONFIG_REGISTER_LOWER & (((PLL960_P_VAL_40 << 8) | PLL960_N_VAL_40))) | (RSI_CONFIG_REGISTER_UPPER & (PLL960_M_VAL_40 << 16))  ) ,
	  	.pll_config_register_2 = ONEBOX_CPU_TO_LE16( RSI_CONFIG_REGISTER_LOWER & 0x0 ),

        },
        .afepll_info_g = {
          .pll_config_register =ONEBOX_CPU_TO_LE16( RSI_CONFIG_REGISTER_LOWER & 0x9f0),
        }
      },
      .switch_clk_g = {
        .switch_clk_info = ONEBOX_CPU_TO_LE32( RSI_SWITCH_UMAC_CLK | RSI_SWITCH_QSPI_CLK | RSI_SWITCH_BBP_LMAC_CLK_REG | 0x011210000),
	.umac_clock_reg_config = ONEBOX_CPU_TO_LE16(0x48),
        .qspi_uart_clock_reg_config = ONEBOX_CPU_TO_LE16(0x1211)
      }
    },
    //! Bluetooth params
    {
      .pll_config_g = {
        .tapll_info_g = {
	    .tapll_config_register_info = ONEBOX_CPU_TO_LE32(( RSI_CONFIG_REGISTER_LOWER & ((TA_PLL_N_VAL_40 << 8) | TA_PLL_M_VAL_40)) | (RSI_CONFIG_REGISTER_UPPER & (TA_PLL_P_VAL_40 << 16))),
        },
        .pll960_info_g = {
            .pll_config_register_1 = ONEBOX_CPU_TO_LE32( (RSI_CONFIG_REGISTER_LOWER & (((PLL960_P_VAL_40 << 8) | PLL960_N_VAL_40))) | (RSI_CONFIG_REGISTER_UPPER & (PLL960_M_VAL_40 << 16))  ) ,
	    .pll_config_register_2 = ONEBOX_CPU_TO_LE16( RSI_CONFIG_REGISTER_LOWER & 0x0 ),
        },
        .afepll_info_g = {
          .pll_config_register =ONEBOX_CPU_TO_LE16( RSI_CONFIG_REGISTER_LOWER & 0x9f0),
        }
      },
      .switch_clk_g = {
          .switch_clk_info = 0x0,

        .umac_clock_reg_config = 0x0,
        .qspi_uart_clock_reg_config = 0x0
      }
    },
    //! Zigbee params
    {
      .pll_config_g = {
        .tapll_info_g = {
	    .tapll_config_register_info = ONEBOX_CPU_TO_LE32(( RSI_CONFIG_REGISTER_LOWER & ((TA_PLL_N_VAL_40 << 8) | TA_PLL_M_VAL_40)) | (RSI_CONFIG_REGISTER_UPPER & (TA_PLL_P_VAL_40 << 16))),
        },
        .pll960_info_g = {
            .pll_config_register_1 = ONEBOX_CPU_TO_LE32( (RSI_CONFIG_REGISTER_LOWER & (((PLL960_P_VAL_40 << 8) | PLL960_N_VAL_40))) | (RSI_CONFIG_REGISTER_UPPER & (PLL960_M_VAL_40 << 16))  ) ,
	    .pll_config_register_2 = ONEBOX_CPU_TO_LE16( RSI_CONFIG_REGISTER_LOWER & 0x0 ),
        },
        .afepll_info_g = {
          .pll_config_register =ONEBOX_CPU_TO_LE16( RSI_CONFIG_REGISTER_LOWER & 0x9f0),
        }
      },
      .switch_clk_g = {
         .switch_clk_info = 0x0,
         .umac_clock_reg_config = 0x0,
         .qspi_uart_clock_reg_config = 0x0
      }
    }
  },
  //! ULP Params
  .buckboost_wakeup_cnt = 0x0,
  .pmu_wakeup_wait = 0x0,
  .shutdown_wait_time = 0x0,
  .pmu_slp_clkout_sel = 0x0,
  .wdt_prog_value = 0x0,
  .wdt_soc_rst_delay = 0x0,
  .dcdc_operation_mode = 0x0, 
  .soc_reset_wait_cnt = 0x0,
  .waiting_time_at_fresh_sleep = 0x0, /* do not change; will be handled in f/w */
  .max_threshold_to_avoid_sleep = 0x0, /* do not change; will be handled in f/w */
  .beacon_resedue_alg_en = 0, /* do not change; will be handled in f/w */
};

#endif
  

