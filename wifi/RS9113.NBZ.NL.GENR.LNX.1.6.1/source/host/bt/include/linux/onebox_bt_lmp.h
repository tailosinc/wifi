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

/* LMP PACKET OPCODE DEFINITIONS */

#define LMP_NAME_REQ 1
#define LMP_NAME_RES 2
#define LMP_ACCEPTED 3
#define LMP_NOT_ACCEPTED 4
#define LMP_CLKOFFSET_REQ 5
#define LMP_CLKOFFSET_RES 6
#define LMP_DETACH 7
#define LMP_IN_RAND 8
#define LMP_COMB_KEY 9
#define LMP_UNIT_KEY 10
#define LMP_AU_RAND 11
#define LMP_SRES 12
#define LMP_TEMP_RAND 13
#define LMP_TEMP_KEY 14
#define LMP_ENCRYPTION_MODE_REQ 15
#define LMP_ENCRYPTION_KEY_SIZE_REQ 16
#define LMP_START_ENCRYPTION_REQ 17
#define LMP_STOP_ENCRYPTION_REQ 18
#define LMP_SWITCH_REQ 19
#define LMP_HOLD_PKT 20
#define LMP_HOLD_REQ 21
#define LMP_SNIFF_REQ 23
#define LMP_UNSNIFF_REQ 24
#define LMP_PARK_REQ 25
#define LMP_SET_BROADCAST_SCAN_WINDOW 27
#define LMP_MODIFY_BEACON 28
#define LMP_UNPARK_BD_ADDR_REQ 29
#define LMP_UNPARK_PM_ADDR_REQ 30
#define LMP_INCR_POWER_REQ 31
#define LMP_DECR_POWER_REQ 32
#define LMP_MAX_POWER 33
#define LMP_MIN_POWER 34
#define LMP_AUTO_RATE 35
#define LMP_PREFERRED_RATE 36
#define LMP_VERSION_REQ 37
#define LMP_VERSION_RES 38
#define LMP_FEATURES_REQ 39
#define LMP_FEATURES_RES 40
#define LMP_QUALITY_OF_SERVICE 41
#define LMP_QUALITY_OF_SERVICE_REQ 42
#define LMP_SCO_LINK_REQ 43
#define LMP_REMOVE_SCO_LINK_REQ 44
#define LMP_MAX_SLOT 45
#define LMP_MAX_SLOT_REQ 46
#define LMP_TIMING_ACCURACY_REQ 47
#define LMP_TIMING_ACCURACY_RES 48
#define LMP_SETUP_COMPLETE 49
#define LMP_USE_SEMI_PERMANENT_KEY 50
#define LMP_HOST_CONNECTION_REQ 51
#define LMP_SLOT_OFFSET 52
#define LMP_PAGE_MODE_REQ 53
#define LMP_PAGE_SCAN_MODE_REQ 54
#define LMP_SUPERVISION_TIMEOUT 55
#define LMP_TEST_ACTIVATE 56
#define LMP_TEST_CONTROL 57
#define LMP_ENCRYPTION_KEYSIZE_MASK_REQ 58
#define LMP_ENCRYPTION_KEYSIZE_MASK_RES 59
#define LMP_SET_AFH 60
#define LMP_ENCAPSULATED_HEADER 61
#define LMP_ENCAPSULATED_PAYLOAD 62
#define LMP_SIMPLE_PAIRING_CONFIRM 63
#define LMP_SIMPLE_PAIRING_NUMBER 64
#define LMP_DHKEY_CHECK 65
#define LMP_ESCAPE_OPCODE 127
/* extended op codes, LS byte(escape opcode) = 127 */
#define LMP_ACCEPTED_EXT 1
#define LMP_NOT_ACCEPTED_EXT 2
#define LMP_FEATURES_REQ_EXT 3
#define LMP_FEATURES_RES_EXT 4
#define LMP_PACKET_TYPE_TABLE_REQ 11
#define LMP_ESCO_LINK_REQ 12
#define LMP_REMOVE_eSCO_LINK_REQ 13
#define LMP_CHANNEL_CLASSIFICATION_REQ 16
#define LMP_CHANNEL_CLASSIFICATION 17
#define LMP_SNIFF_SUBRATING_REQ 21
#define LMP_SNIFF_SUBRATING_RES 22
#define LMP_PAUSE_ENCRYPTION_REQ 23
#define LMP_RESUME_ENCRYPTION_REQ 24
#define LMP_IO_CAPABILITIES_REQ 25
#define LMP_IO_CAPABILITIES_RES 26
#define LMP_NUMERIC_COMPARISON_FAILED 27
#define LMP_PASSKEY_FAILED 28
#define LMP_OOB_FAILED 29
#define LMP_KEYPRESS_NOTIFICATION 30
#define LMP_POWER_CONTROL_REQ 31
#define LMP_POWER_CONTROL_RES 32


/* LMP packet opcodes */
static char lmp_opcodes[66][32] = {
                            "",
                            "LMP_NAME_REQ",
                            "LMP_NAME_RES",
                            "LMP_ACCEPTED",
                            "LMP_NOT_ACCEPTED",
                            "LMP_CLKOFFSET_REQ",
                            "LMP_CLKOFFSET_RES",
                            "LMP_DETACH",
                            "LMP_IN_RAND",
                            "LMP_COMB_KEY",
                            "LMP_UNIT_KEY",
                            "LMP_AU_RAND",
                            "LMP_SRES",
                            "LMP_TEMP_RAND",
                            "LMP_TEMP_KEY",
                            "LMP_ENCRYPTION_MODE_REQ",
                            "LMP_ENCRYPTION_KEY_SIZE_REQ",
                            "LMP_START_ENCRYPTION_REQ",
                            "LMP_STOP_ENCRYPTION_REQ",
                            "LMP_SWITCH_REQ",
                            "LMP_HOLD",
                            "LMP_HOLD_REQ",
                            "",
                            "LMP_SNIFF_REQ",
                            "LMP_UNSNIFF_REQ",
                            "LMP_PARK_REQ",
                            "",
                            "LMP_SET_BROADCAST_SCAN_WINDOW",
                            "LMP_MODIFY_BEACON",
                            "LMP_UNPARK_BD_ADDR_REQ",
                            "LMP_UNPARK_PM_ADDR_REQ",
                            "LMP_INCR_POWER_REQ",
                            "LMP_DECR_POWER_REQ",
                            "LMP_MAX_POWER",
                            "LMP_MIN_POWER",
                            "LMP_AUTO_RATE",
                            "LMP_PREFERRED_RATE",
                            "LMP_VERSION_REQ",
                            "LMP_VERSION_RES",
                            "LMP_FEATURES_REQ",
                            "LMP_FEATURES_RES",
                            "LMP_QUALITY_OF_SERVICE",
                            "LMP_QUALITY_OF_SERVICE_REQ",
                            "LMP_SCO_LINK_REQ",
                            "LMP_REMOVE_SCO_LINK_REQ",
                            "LMP_MAX_SLOT",
                            "LMP_MAX_SLOT_REQ",
                            "LMP_TIMING_ACCURACY_REQ",
                            "LMP_TIMING_ACCURACY_RES",
                            "LMP_SETUP_COMPLETE",
                            "LMP_USE_SEMI_PERMANENT_KEY",
                            "LMP_HOST_CONNECTION_REQ",
                            "LMP_SLOT_OFFSET",
                            "LMP_PAGE_MODE_REQ",
                            "LMP_PAGE_SCAN_MODE_REQ",
                            "LMP_SUPERVISION_TIMEOUT",
                            "LMP_TEST_ACTIVATE",
                            "LMP_TEST_CONTROL",
                            "LMP_ENCRYPTION_KEYSIZE_MASK_REQ",
                            "LMP_ENCRYPTION_KEYSIZE_MASK_RES",
                            "LMP_SET_AFH",
                            "LMP_ENCAPSULATED_HEADER",
                            "LMP_ENCAPSULATED_PAYLOAD",
                            "LMP_SIMPLE_PAIRING_CONFIRM",
                            "LMP_SIMPLE_PAIRING_NUMBER",
                            "LMP_DHKEY_CHECK",
};

/* LMP extended packet opcodes */
static char lmp_ext_opcodes[33][32] = {
                            "",
                            "LMP_ACCEPTED_EXT",
                            "LMP_NOT_ACCEPTED_EXT",
                            "LMP_FEATURES_REQ_EXT",
                            "LMP_FEATURES_RES_EXT",
                            "",
                            "",
                            "",
                            "",
                            "",
                            "",
                            "LMP_PACKET_TYPE_TABLE_REQ",
                            "LMP_ESCO_LINK_REQ",
                            "LMP_REMOVE_eSCO_LINK_REQ",
                            "",
                            "",
                            "LMP_CHANNEL_CLASSIFICATION_REQ",
                            "LMP_CHANNEL_CLASSIFICATION",
                            "",
                            "",
                            "",
                            "LMP_SNIFF_SUBRATING_REQ",
                            "LMP_SNIFF_SUBRATING_RES",
                            "LMP_PAUSE_ENCRYPTION_REQ",
                            "LMP_RESUME_ENCRYPTION_REQ",
                            "LMP_IO_CAPABILITIES_REQ",
                            "LMP_IO_CAPABILITIES_RES",
                            "LMP_NUMERIC_COMPARISON_FAILED",
                            "LMP_PASSKEY_FAILED",
                            "LMP_OOB_FAILED",
                            "LMP_KEYPRESS_NOTIFICATION",
                            "LMP_POWER_CONTROL_REQ",
                            "LMP_POWER_CONTROL_RES",

};

/* Vendor ID packets - sub types */
#define LMP_PKT_DEBUG_MSG  0x0001
#define LLP_PKT_DEBUG_MSG  0x0002

/* sub packet debug message types */
#define TX_PKT_DEBUG_MSG 0x0
#define RX_PKT_DEBUG_MSG 0x1


