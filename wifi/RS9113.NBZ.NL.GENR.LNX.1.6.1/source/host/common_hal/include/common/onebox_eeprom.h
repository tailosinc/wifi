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

#ifndef __ONEBOX_EEPROM_H__
#define __ONEBOX_EEPROM_H__


#include "onebox_common.h"

#define MGMT_DESC_EEPROM_WRITE_CFM  0x8F00
#define MGMT_DESC_EEPROM_READ_CFM  0x8E00
/* Tx Path */
#define MGMT_DESC_EEPROM_READ    0x0B00


#define EEPROM_VER0 0
#define EEPROM_VER1 1
#define EEPROM_VER2 2
#define EEPROM_VER6 6

/*Data structures */

typedef struct eepromrw_info_s
{
	uint32 offset;
	uint32 length;
	uint8  write;
	uint16 eeprom_erase;
	uint8 data[480];
}eepromrw, EEPROMRW;

typedef struct eeprom_read
{
	uint16 length;
	uint16 off_set;
}*PEEPROM_READ,EEPROM_READ;


//! This structure members are used to configure qspi
typedef struct spi_config_1_s {
  /**
   * QSPI operation modes, all modes are single, dual or quad 
   **/
  uint32 inst_mode : 2;          //!< instruction will be sent in this mode
  uint32 addr_mode : 2;          //!< addr will be sent in this mode
  uint32 data_mode : 2;          //!< data will be sent/received in this mode
  uint32 dummy_mode : 2;         //!< dummy bytes will be sent/received in this mode
  uint32 extra_byte_mode : 2;    //!< extra bytes will be sent in this mode
  //! SPI mode
#define SINGLE_MODE                  0
  //! dual mode
#define DUAL_MODE                    1
  //! quad mode
#define QUAD_MODE                    2
  
    uint32 prefetch_en : 1;     //!< prefetch enable
  //! prefetch will be enabled
#define EN_PREFETCH                  1 
  //! prefetch will be disabled
#define DIS_PREFETCH                 0
  
  uint32 dummy_W_or_R : 1;      //!< dummy writes or read select
  //! dummy's are read
#define DUMMY_READS                  0
  //! dummy's are written
#define DUMMY_WRITES                 1
  
  uint32 extra_byte_en : 1;     //!< Enable extra byte
  //! Extra byte will be enabled
#define EN_EXTRA_BYTE
  //! Extra byte will be disabled
#define DIS_EXTRA_BYTE
  
  uint32 d3d2_data : 2;         //!< Data on D3 and D2 line in SPI or DUAL mode
  
  uint32 continuous : 1;        //!< continuous mode select
  //! continuous mode is selected
#define CONTINUOUS                   1
  //! discontinuous mode is selected
#define DIS_CONTINUOUS               0 
  
  uint32 read_cmd  : 8;         //!< read cmd to be used
  
  uint32 flash_type : 4;        //!< flash defines
  //! sst spi flash
#define SST_SPI_FLASH                0
  //! sst dual flash
#define SST_DUAL_FLASH               1
  //! sst quad flash
#define SST_QUAD_FLASH               2
  //! Winbond quad flash
#define WBOND_QUAD_FLASH             3
  //! Atmel quad flash
#define AT_QUAD_FLASH                4
  //! macronix quad flash
#define MX_QUAD_FLASH                5
#define EON_QUAD_FLASH               6
#define MICRON_QUAD_FLASH            7
  
  uint32 no_of_dummy_bytes : 4; //!< no_of_dummy_bytes to be used for read operations 
} spi_config_1_t;

//! This structure members are used to configure qspi
typedef struct spi_config_2_s {

  uint32 auto_mode : 1;         //!< mode select
  //! Auto mode selection
#define EN_AUTO_MODE                 1
  //! Manual mode selection
#define EN_MANUAL_MODE               0
 
  uint32 cs_no : 2;             //!<  QSPI chip_select
  //! cs-0
#define CHIP_ZERO                    0
  //! cs-1
#define CHIP_ONE                     1
  //! cs-2
#define CHIP_TWO                     2
  //! cs-3
#define CHIP_THREE                   3
  
  uint32 jump_en : 1;           //!< Jump Enable
  //! Enables jump
#define EN_JUMP                      1
  //! Disables jump
#define DIS_JUMP                     0
  
  uint32 neg_edge_sampling : 1; //!< For High speed mode, sample at neg edge 
  //! enables neg edge sampling
#define NEG_EDGE_SAMPLING            1
  //! enables pos edge sampling
#define POS_EDGE_SAMPLING            0 
  
  uint32 qspi_clk_en : 1;       //!< qspi clk select
  //! full time clk will be provided
#define QSPI_FULL_TIME_CLK           1
  //! dynamic clk gating will be enabled
#define QSPI_DYNAMIC_CLK             0
  
  uint32 protection : 2;        //!< flash protection select
  //! enable write protection
#define EN_WR_PROT                   2
  //! remove write protection  
#define REM_WR_PROT                  1
  //! no change to wr protection
#define DNT_REM_WR_PROT              0
  
  uint32 dma_mode : 1;          //!< dma mode enable
  //! use dma only in manaul mode
#define DMA_MODE                     1
  //! dma will not be used 
#define NO_DMA                       0
  
  uint32 swap_en : 1;           //!< swap enable for w/r
  //! swap will be enabled
#define SWAP                         1 
  //! swap will be disabled
#define NO_SWAP                      0
    
  uint32 full_duplex : 2;       //!< full duplex mode select
  //! do nothing for full duplex
#define IGNORE_FULL_DUPLEX           2
  //! enable full duplex
#define EN_FULL_DUPLEX               1
  //! disable full duplex
#define DIS_FULL_DUPLEX              0
  
  uint32 wrap_len_in_bytes : 3; //!< wrap len to be used
  //! wrap is diabled
#define NO_WRAP                      7
  //! 8 byte wrap will be used
#define _8BYTE_WRAP                  0
  //! 16 byte wrap will be used
#define _16BYTE_WRAP                 1
  //! 32 byte wrap will be used
#define _32BYTE_WRAP                 2 
  //! 64 byte wrap will be used
#define _64BYTE_WRAP                 3
  
  uint32 mode_0_or_3 : 1;       //!< qspi clk mode select
  //! mode 3 clk will be used
#define MODE_3                       1   
  //! mode 0 clk will be used
#define MODE_0                       0  
  
  uint32 addr_width : 2;        //!< addr width to used
  //! 24 bit addr is configured
#define _24BIT_ADDR                  3
  //! 16 bit addr is configured
#define _16BIT_ADDR                  2 
  //! 9 bit addr is configured
#define _9BIT_ADDR                   1 
  //! 8 bit addr is configured
#define _8BIT_ADDR                   0
  
  uint32 jump_inst : 8;         //!< Instruction to be used in case of jump

  uint32 dummys_4_jump : 2;     //!< no_of_dummy_bytes in case of jump instruction
  
  uint32 num_prot_bytes : 4;    //!< width of memory protection reg for sst flashes
  //! 10 bytes wide
#define _10BYTES_LONG                10
  //! 10 bytes wide
#define _6BYTES_LONG                 6

} spi_config_2_t;

//! This structure has two daughter structures to configure qspi
typedef struct spi_config_s {
  spi_config_1_t spi_config_1;  //!< daughter structure 1
  spi_config_2_t spi_config_2;  //!< daughter structure 2 
} spi_config_t;

#endif
