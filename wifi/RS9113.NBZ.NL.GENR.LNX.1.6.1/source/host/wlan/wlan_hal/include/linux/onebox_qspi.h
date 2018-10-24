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

#ifndef __ONEBOX_QSPI_H
#define __ONEBOX_QSPI_H


#include "onebox_eeprom.h"
/*****************************************
 *        Qspi register defines
 *****************************************/
//! cmd len will be 8 bits
#define CMD_LEN                      8
//! reg bit
#define RD_FIFO_EMPTY                BIT(7)
//! reg bit
#define BUSY                         BIT(0)
//! reg bit
#define FULL_DUPLEX_EN               BIT(22)  
//! reg bit
#define HW_CTRL_MODE                 BIT(25)
//! reg bit
#define AUTO_MODE                    BIT(6) 
//! reg bit
#define EXTRA_BYTE_EN                BIT(18)
//! reg bit
#define AUTO_RD_SWAP                 BIT(0)
//! reg bit
#define HW_CTRLD_QSPI_MODE_CTRL_SCLK BIT(14)
//! reg bit
#define AUTO_MODE_FSM_IDLE_SCLK      BIT(10)
//! reg bit
#define CSN_ACTIVE                   BIT(0)  
//! reg bit
#define QSPI_PREFETCH_EN             BIT(4)
//! reg bit
#define QSPI_WRAP_EN                 BIT(5)
//! reg bit
#define READ_TRIGGER                 BIT(2)
//! reg bit
#define WRITE_TRIGGER                BIT(1)

/******************************************
 *              FLASH CMDS 
 ******************************************/
//! Write enable cmd
#define WREN                         0x06
//! Write disable cmd
#define WRDI                         0x04
//! Read status reg cmd
#define RDSR                         0x05
//! chip erase cmd
#define CHIP_ERASE                   0xC7
//! block erase cmd
#define BLOCK_ERASE                  0xD8 
//! sector erase cmd
#define SECTOR_ERASE                 0x20      
//! high speed rd cmd
#define HISPEED_READ                 0x0B 
//! rd cmd
#define READ_ONLY                    0x03

/****************************************
 *        SST25 specific cmds 
 ****************************************/
//! Write status reg cmd
#define WRSR                         0x01
//! Enable Write status reg cmd
#define EWSR                         0x50                              
//! Auto address incremental rd cmd
#define AAI                          0xAF
//! Byte program cmd
#define BYTE_PROGRAM                 0x02

/****************************************
 *       SST26 specific cmds 
 ***************************************/
//! Enable quad IO
#define EQIO                         0x38 
//! Reset quad IO
#define RSTQIO                       0xFF
//! wrap : set burst
#define SET_BURST                    0xC0
//! wrap : read cmd
#define READ_BURST                   0x0C
//! Jump : page index read
#define READ_PI                      0x08
//! Jump : Index read
#define READ_I                       0x09    
//! Jump : Block Index read
#define READ_BI                      0x10
//! Page program cmd 
#define PAGE_PROGRAM                 0x02  
//! write suspend cmd
#define Write_Suspend                0xB0        
//! write resume cmd
#define Write_Resume                 0x30  
//! read block protection reg
#define RBPR                         0x72
//! Write block protection reg
#define WBPR                         0x42
//! Lockdown block protection reg 
#define LBPR                         0x8D

/****************************************
 * WINBOND + AT + MACRONIX specific cmds
 ***************************************/
//! fast read dual output
#define FREAD_DUAL_O                 0x3B
//! fast read quad output
#define FREAD_QUAD_O                 0x6B


/****************************************
 * WINBOND + MACRONIX specific cmds 
 ***************************************/ 
//! fast read dual IO
#define FREAD_DUAL_IO                0xBB
//! fast read quad IO
#define FREAD_QUAD_IO                0xEB


/****************************************
 *       WINBOND specific cmds
 ***************************************/
//! Octal word read (A7-A0 must be 00)
#define OCTAL_WREAD                  0xE3
//! Enable high performance cmd
#define HI_PERFMNC                   0xA3


/****************************************
 *        ATMEL specific cmds
 ***************************************/
//! write config reg
#define WCON                         0x3E
//! read config reg
#define RCON                         0x3F
//! supported upto 100MHz
#define HI_FREQ_SPI_READ             0x1B


/****************************************
 *    MACRONIX specific write cmds 
 ***************************************/
//! Address and data in quad
#define QUAD_PAGE_PROGRAM            0x38



/****************************************
 * ATMEL + WINBOND specific write cmds
 ***************************************/
//! Only data in quad mode
#define QUAD_IN_PAGE_PROGRAM         0x32



/****************************************
 * ATMEL specific write cmds
 ***************************************/
//! Data in dual 
#define DUAL_IN_PAGE_PROGRAM         0xA2


/*************************************** 
 *        Defines for arguments 
 **************************************/

//! disable hw ctrl
#define DIS_HW_CTRL                  1
//! donot disable hw ctrl
#define DNT_DIS_HW_CTRL              0

//! 32bit hsize
#define _32BIT                       3
//! 24bit hsize is not supported, so reserved
//      reserved                     2
//! 16bit hsize
#define _16BIT                       1
//! 8bit hsize
#define _8BIT                        0



#endif
