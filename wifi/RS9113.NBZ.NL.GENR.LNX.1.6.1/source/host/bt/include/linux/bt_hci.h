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

#if 0
/* HCI data types */
#define HCI_COMMAND_PKT   0x01
#define HCI_ACLDATA_PKT   0x02
#define HCI_SCODATA_PKT   0x03
#define HCI_EVENT_PKT     0x04
#define HCI_VENDOR_PKT    0xFF
#endif

/* HCI OGF types */
#define HCI_LINK_CNTRL_CMD      0x01
#define HCI_LINK_POLICY_CMD     0x02
#define HCI_CNTRL_BB_CMD        0x03
#define HCI_INFO_PARAMS_CMD     0x04
#define HCI_STATUS_PARAMS_CMD   0x05
#define HCI_TESTING_CMD         0x06
#define HCI_LE_CNTRL_CMD        0x08
#define HCI_VENDOR_SPECIFIC     0x3F

/* HCI link control command OCF types */
static char link_control_cmd_string[61][41] = {
                                       "",
                                       "HCI_OP_INQUIRY",
                                       "HCI_OP_INQUIRY_CANCEL",
                                       "HCI_OP_PERIODIC_INQUIRY_MODE",
                                       "HCI_OP_EXIT_PERIODIC_INQUIRY",
                                       "HCI_OP_CREATE_CON",
                                       "HCI_OP_DISCONNECT",
                                       "",
                                       "HCI_OP_CREATE_CON_CANCEL",
                                       "HCI_OP_ACCEPT_CONN_REQ",
                                       "HCI_OP_REJECT_CONN_REQ",
                                       "HCI_OP_LINK_KEY_REQ_RPLY",
                                       "HCI_OP_LINK_KEY_REQ_NEGATIVE_RPLY",
                                       "HCI_OP_PIN_CODE_REQUEST_RPLY",
                                       "HCI_OP_PIN_CODE_REQUEST_NEGATIVE_RPLY",
                                       "HCI_OP_CHANGE_CONN_PKT_TYPE",
                                       "",
                                       "HCI_OP_AUTH_REQ",
                                       "",
                                       "HCI_OP_SET_CONN_ENCRYPTION",
                                       "",
                                       "HCI_OP_CHANGE_CONN_LINK_TYPE",
                                       "",
                                       "HCI_OP_MASTER_LINK_KEY",
                                       "",
                                       "HCI_OP_REMOTE_NAME_REQ",
                                       "HCI_OP_REMOTE_NAME_REQ_CANCEL",
                                       "HCI_OP_READ_REMOTE_SUPP_FEATURES",
                                       "HCI_OP_READ_REMOTE_EXT_SUPP_FEATURES",
                                       "HCI_OP_READ_REMOTE_VERS_INFO",
                                       "",
                                       "HCI_OP_READ_REMOTE_CLK_OFFSET",
                                       "HCI_OP_READ_LMP_HANDLE",
                                       "",
                                       "",
                                       "",
                                       "",
                                       "",
                                       "",
                                       "",
                                       "HCI_OP_SETUP_SYNCHRONOUS_CONN",
                                       "HCI_OP_ACCEPT_SYNCHRONOUS_CONN",
                                       "HCI_OP_REJECT_SYNCHRONOUS_CONN",
                                       "HCI_OP_CAPABILITY_REQ_RPLY",
                                       "HCI_OP_USER_CONFIRM_REQ_RPLY",
                                       "HCI_OP_USER_CONFIRM_REQ_NEGATIVE_RPLY",
                                       "HCI_OP_USER_PASS_KEY_REQ_RPLY",
                                       "HCI_OP_USER_PASS_KEY_REQ_NEGATIVE_RPLY",
                                       "HCI_OP_REMOTE_OOB_DATA_REQ_RPLY",
                                       "",
                                       "",
                                       "HCI_OP_REMOYE_OOB_DATA_REQ_NEGATIVE_RPLY",
                                       "HCI_OP_IO_CAP_REQ_NEGATIVE_RPLY",
                                       "HCI_OP_CREATE_PHYSICAL_LINK",
                                       "HCI_OP_ACCEPT_PHYSICAL_LINK",
                                       "HCI_OP_DISCONNECT_PHYSICAL_LINK",
                                       "HCI_OP_CREATE_LOGICAL_LINK",
                                       "HCI_OP_ACCEPT_LOGICAL_LINK",
                                       "HCI_OP_DISCONNECT_LOGICAL_LINK",
                                       "HCI_OP_LOGICAL_LINK_CANCEL",
                                       "HCI_OP_FLOW_SPEC_MODIFY"
};

/* HCI link policy command OCF types */
static char link_policy_cmd_string[18][30] = {
                                      "",
                                      "HCI_OP_HOLD_MODE",
									  "",
                                      "HCI_OP_SNIFF_MODE",
                                      "HCI_OP_EXIT_SNIFF_MODE",
                                      "HCI_OP_PARK_STATE",
                                      "HCI_OP_EXIT_PARK_STATE",
                                      "HCI_OP_QOS_SETUP",
									  "",
                                      "HCI_OP_ROLE_DISCOVERY",
									  "",
                                      "HCI_OP_SWITCH_ROLE",
                                      "HCI_OP_READ_LINK_POLICY",
                                      "HCI_OP_WRITE_LINK_POLICY",
                                      "HCI_OP_READ_DEF_LINK_POLICY",
                                      "HCI_OP_WRITE_DEF_LINK_POLICY",
                                      "HCI_OP_FLOW_SPECIFICATION",
                                      "HCI_OP_SNIFF_SUBRATING",
};

/* HCI control and baseband OCF types */
static char link_bbp_ctrl_cmd_string[110][39] = {
                                       "",
                                       "HCI_OP_SET_EVENT_MASK",
                                       "",
                                       "HCI_OP_RESET",
                                       "",
                                       "HCI_OP_SET_EVENT_FLT",
                                       "",
                                       "",
                                       "HCI_OP_FLUSH",
                                       "HCI_OP_READ_PIN_TYPE",
                                       "HCI_OP_WRITE_PIN_TYPE",
                                       "HCI_OP_CREATE_NEW_UNIT_KEY",
                                       "",
                                       "HCI_OP_READ_STORED_LINK_KEY",
                                       "",
                                       "",
                                       "",
                                       "HCI_OP_WRITE_STORED_LINK_KEY",
                                       "HCI_OP_DELETE_STORED_LINK_KEY",
                                       "HCI_OP_WRITE_LOCAL_NAME",
                                       "HCI_OP_READ_LOCAL_NAME",
                                       "HCI_OP_READ_CA_TIMEOUT",
                                       "HCI_OP_WRITE_CA_TIMEOUT",
                                       "HCI_OP_READ_PAGE_TIMEOUT",
                                       "HCI_OP_WRITE_PAGE_TIMEOUT",
                                       "HCI_OP_READ_SCAN_ENABLE",
                                       "HCI_OP_WRITE_SCAN_ENABLE",
                                       "HCI_OP_READ_PAGE_SCAN_ACTIVITY",
                                       "HCI_OP_WRITE_PAGE_SCAN_ACTIVITY",
                                       "HCI_OP_READ_INQUIRY_SCAN_ACTIVITY",
                                       "HCI_OP_WRITE_INQUIRY_SCAN_ACTIVITY",
                                       "HCI_OP_READ_AUTH_ENABLE",
                                       "HCI_OP_WRITE_AUTH_ENABLE",
                                       "",
                                       "",
                                       "HCI_OP_READ_CLASS_OF_DEVICE",
                                       "HCI_OP_WRITE_CLASS_OF_DEVICE",
                                       "HCI_OP_READ_VOICE_SETTING",
                                       "HCI_OP_WRITE_VOICE_SETTING",
                                       "HCI_OP_READ_AUTO_FLUSH_TOUT",
                                       "HCI_OP_WRITE_AUTO_FLUSH_TOUT",
                                       "HCI_OP_READ_NUM_BROADCAST_RETRY",
                                       "HCI_OP_WRITE_NUM_BROADCADT_RETRY",
                                       "HCI_OP_READ_HOLD_MODE_ACTIVITY",
                                       "HCI_OP_WRITE_HOLD_MODE_ACTIVITY",
                                       "HCI_OP_READ_TX_PWR_LEVEL",
                                       "HCI_OP_READ_SYNCHRONOUS_FLOW_CTRL_EN",
                                       "HCI_OP_WRITE_SYNCHRONOUS_FLOW_CTRL_EN",
                                       "",
                                       "HCI_OP_SET_CNTRLER_TO_HOST_FLOW_CTRL",
                                       "HCI_OP_HOST_BUFFER_SIZE",
                                       "",
                                       "",
                                       "HCI_OP_HOST_NUM_COMPLETED_PKTS",
                                       "HCI_OP_READ_LINK_SUPERVISION_TOUT",
                                       "HCI_OP_WRITE_LINK_SUPERVISION_TOUT",
                                       "HCI_OP_READ_NUM_SUPPORTED_IAC",
                                       "HCI_OP_READ_CURRENT_IAC_LAP",
                                       "HCI_OP_WRITE_CURRENT_IAC_LAP",
                                       "",
                                       "",
                                       "",
                                       "",
                                       "HCI_OP_SET_AFH_HOST_CH_CLASSIFICATION",
                                       "",
                                       "",
                                       "HCI_OP_READ_INQUIRY_SCAN_TYPE",
                                       "HCI_OP_WRITE_INQUIRY_SCAN_TYPE",
                                       "HCI_OP_READ_INQUIRY_MODE",
                                       "HCI_OP_WRITE_INQUIRY_MODE",
                                       "HCI_OP_READ_PAGE_SCAN_TYPE",
                                       "HCI_OP_WRITE_PAGE_SCAN_TYPE",
                                       "HCI_OP_READ_AFH_CH_ASSESSMENT_MODE",
                                       "HCI_OP_WRITE_AFH_CH_ASSESSMENT_MODE",
                                       "",
                                       "",
                                       "",
                                       "",
                                       "",
                                       "",
                                       "",
                                       "HCI_OP_READ_EXT_INQUIRY_RESP",
                                       "HCI_OP_WRITE_EXT_INQUIRY_RESP",
                                       "HCI_OP_REFRESH_ENCRYPTION_KEY",
                                       "",
                                       "HCI_OP_READ_SIMPLE_PAIRING_MODE",
                                       "HCI_OP_WRITE_SIMPLE_PAIRING_MODE",
                                       "HCI_OP_READ_LOCAL_OOB_DATA",
                                       "HCI_OP_READ_INQUIRY_RESP_TX_PWR_LEVEL",
                                       "HCI_OP_WRITE_INQUIRY_RESP_TX_PWR_LEVEL",
                                       "HCI_OP_READ_DEF_ERRONEOUS_DATA_REP",
                                       "HCI_OP_WRITE_DEF_ERRONEOUS_DATA_REP",
                                       "",
                                       "",
                                       "",
                                       "HCI_OP_ENHANCED_FLUSH",
                                       "HCI_OP_SEND_KEY_PRESS_NOTIFICATION",
                                       "HCI_OP_READ_LOGICAL_LINK_ACCEPT_TOUT",
                                       "HCI_OP_WRITE_LOGICAL_LINK_ACCEPT_TOUT",
                                       "HCI_OP_SET_EVENT_MASK_PAGE_2",
                                       "HCI_OP_READ_LOCATION_DATA",
                                       "HCI_OP_WRITE_LOCATION_DATA",
                                       "HCI_OP_READ_FLOW_CNTRL_MODE",
                                       "HCI_OP_WRITE_FLOW_CONTROL_MODE",
                                       "HCI_OP_READ_ENHANCED_TX_PWR_LEVEL",
                                       "HCI_OP_READ_BE_FLUSH_TOUT",
                                       "HCI_OP_WRITE_BE_FLUSH_TOUT",
                                       "HCI_OP_SHORT_RANGE_MODE",
                                       "HCI_OP_READ_LE_HOST_SUPPORT",
                                       "HCI_OP_WRITE_LE_HOST_SUPPORT"
};

/* HCI Information Command OCF types */
static char link_info_cmd_string[11][31] = {
                                       "",
                                       "HCI_OP_READ_LOCAL_VERSION",
                                       "HCI_OP_READ_LOCAL_COMMANDS",
                                       "HCI_OP_READ_LOCAL_FEATURES",
                                       "HCI_OP_READ_LOCAL_EXT_FEATURES",
                                       "HCI_OP_READ_BUFFER_SIZE",
                                       "",
                                       "",
                                       "",
                                       "HCI_OP_READ_BD_ADDR",
                                       "HCI_OP_READ_DATA_BLK_SIZE"
};

/* HCI Status Command OCF types */
static char link_status_cmd_string[12][37] = {
                                       "",
                                       "HCI_OP_READ_FAILED_CONTACT_COUNTER",
                                       "HCI_OP_RESET_FAILED_CONTACT_COUNTER",
                                       "HCI_OP_READ_LINK_QUALITY",
                                       "HCI_OP_READ_RSSI",
                                       "",
                                       "HCI_OP_READ_AFH_CH_MAP",
                                       "HCI_OP_READ_CLK",
                                       "HCI_OP_READ_ENCRYPTION_KEY_SIZE",
                                       "HCI_OP_READ_LOCAL_AMP_INFO",
                                       "HCI_OP_READ_LOCAL_AMP_ASSOC",
                                       "HCI_OP_WRITE_REMOTE_AMP_ASSOC",
};

/* HCI Testing Command OCF types */
static char link_testing_cmd_string[10][35] = {
                                       "",
                                       "HCI_OP_READ_LOOP_BACK_MODE",
                                       "HCI_OP_WRITE_LOOP_BACK_MODE",
                                       "HCI_OP_ENABLE_DEV_UNDER_TEST_MODE",
                                       "HCI_OP_WRITE_PAIRINF_DBG_MODE",
                                       "",
                                       "",
                                       "HCI_OP_ENABLE_AMP_RECEIVER_REPORT",
                                       "HCI_OP_AMP_TEST_END",
                                       "HCI_OP_AMP_TEST",
};

/*HCI LE Commands OCF types */
static char le_cmd_string[32][40] = {
                                     "",
                                     "LE_SET_EVENT_MASK_CMD",
                                     "LE_READ_BUFFER_SIZE_CMD",                     
                                     "LE_READ_LOCAL_SUPPORTED_FEATURES_CMD",
                                     "",
                                     "LE_SET_RANDOM_ADDRESS_CMD",
                                     "LE_SET_ADVERTISING_PARAMETERS_CMD",
                                     "LE_READ_ADVERTISING_CHANNEL_TX_POWER_CMD",
                                     "LE_SET_ADVERTISING_DATA_CMD",
                                     "LE_SET_SCAN_RESPONSE_DATA_CMD",
                                     "LE_SET_ADVERTISE_ENABLE_CMD",        
                                     "LE_SET_SCAN_PARAMETERS_CMD",           
                                     "LE_SET_SCAN_ENABLE_CMD",         
                                     "LE_CREATE_CONNECTION_CMD",
                                     "LE_CREATE_CONNECTION_CANCEL_CMD",
                                     "LE_READ_WHITE_LIST_SIZE_CMD",
                                     "LE_CLEAR_WHITE_LIST_CMD",
                                     "LE_ADD_DEVICE_TO_WHITE_LIST_CMD",
                                     "LE_REMOVE_DEVICE_FROM_WHITE_LIST_CMD",
                                     "LE_CONNECTION_UPDATE_CMD",
                                     "LE_SET_HOST_CHANNEL_CLASSIFICATION_CMD",
                                     "LE_READ_CHANNEL_MAP_CMD",
                                     "LE_READ_REMOTE_USED_FEATURES_CMD",
                                     "LE_ENCRYPT_CMD",
                                     "LE_RAND_CMD",
                                     "LE_START_ENCRYPTION_CMD",
                                     "LE_LONG_TERM_KEY_REQUEST_REPLY_CMD",
                                     "LE_LONG_TERM_KEY_REQ_NEGATIVE_REPLY_CMD",
                                     "LE_READ_SUPPORTED_STATES_CMD",
                                     "LE_RECEIVER_TEST_CMD",
                                     "LE_TRANSMITTER_TEST_CMD",
                                     "LE_TEST_END_CMD",
};

/* HCI Event codes */
char hci_events[78][47] = {
                           "",
                           "INQUIRY_COMPLETE_EVENT",
                           "INQUIRY_RESULT_EVENT",
                           "CONNECTION_COMPLETE_EVENT",
                           "CONNECTION_REQUEST_EVENT",
                           "DISCONNECTION_COMPLETE_EVENT",
                           "AUTHENTICATION_COMPLETE_EVENT",
                           "REMOTE_NAME_REQUEST_COMPLETE_EVENT",
                           "ENCRYPTION_CHANGE_EVENT",
                           "CHANGE_CONNECTION_LINK_KEY_COMPLETE_EVENT",
                           "MASTER_LINK_KEY_COMPLETE_EVENT",
                           "READ_REMOTE_SUPPORTED_FEATURES_COMPLETE_EVENT",
                           "READ_REMOTE_VERSION_INFORMATION_COMPLETE_EVENT",
                           "QOS_SETUP_COMPLETE_EVENT",
                           "COMMAND_COMPLETE_EVENT",
                           "COMMAND_STATUS_EVENT",
                           "HARDWARE_ERROR_EVENT",
                           "FLUSH_OCCURRED_EVENT",
                           "ROLE_CHANGE_EVENT",
                           "NUMBER_OF_COMPLETED_PACKETS_EVENT",
                           "MODE_CHANGE_EVENT",
                           "RETURN_LINK_KEYS_EVENT",
                           "PIN_CODE_REQUEST_EVENT",
                           "LINK_KEY_REQUEST_EVENT",
                           "LINK_KEY_NOTIFICATION_EVENT",
                           "LOOPBACK_COMMAND_EVENT",
                           "DATA_BUFFER_OVERFLOW_EVENT",
                           "MAX_SLOTS_CHANGE_EVENT",
                           "READ_CLOCK_OFFSET_COMPLETE_EVENT",
                           "CONNECTION_PACKET_TYPE_CHANGED_EVENT",
                           "QOS_VIOLATION_EVENT",
                           "",
                           "PAGE_SCAN_REPETITION_MODE_CHANGE_EVENT",
                           "FLOW_SPECIFICATION_COMPLETE_EVENT",
                           "INQUIRY_RESULT_WITH_RSSI_EVENT",
                           "READ_REMOTE_EXTENDED_FEATURES_COMPLETE_EVENT",
                           "",
                           "",
                           "",
                           "",
                           "",
                           "",
                           "",
                           "",
                           "SYNCHRONOUS_CONNECTION_COMPLETE_EVENT",
                           "SYNCHRONOUS_CONNECTION_CHANGED_EVENT",
                           "SNIFF_SUBRATING_EVENT",
                           "EXTENDED_INQUIRY_RESULT_EVENT",
                           "ENCRYPTION_KEY_REFRESH_COMPLETE_EVENT",
                           "IO_CAPABILITY_REQUEST_EVENT",
                           "IO_CAPABILITY_RESPONSE_EVENT",
                           "USER_CONFIRMATION_REQUEST_EVENT",
                           "USER_PASSKEY_REQUEST_EVENT",
                           "REMOTE_OOB_DATA_REQUEST_EVENT",
                           "SIMPLE_PAIRING_COMPLETE_EVENT",
                           "",
                           "LINK_SUPERVISION_TIMEOUT_CHANGED_EVENT",
                           "ENHANCED_FLUSH_COMPLETE_EVENT",
                           "",
                           "USER_PASSKEY_NOTIFICATION_EVENT",
                           "KEYPRESS_NOTIFICATION_EVENT",
                           "REM_HOST_SUPP_FEATURES_NOTIFICATION_EVENT",
                           "LE_META_EVENT",
                           "",
                           "PHYSICAL_LINK_COMPLETE_EVENT",
                           "CHANNEL_SELECTED_EVENT",
                           "DISCONNECTION_PHYSICAL_LINK_COMPLETE_EVENT",
                           "PHYSICAL_LINK_LOSS_EARLY_WARNING_EVENT",
                           "PHYSICAL_LINK_RECOVERY_EVENT",
                           "LOGICAL_LINK_COMPLETE_EVENT",
                           "DISCONNECTION_LOGICAL_LINK_COMPLETE_EVENT",
                           "FLOW_SPEC_MODIFY_COMPLETE_EVENT",
                           "NUMBER_OF_COMPLETED_DATA_BLOCKS_EVENT",
                           "AMP_START_TEST_EVENT",
                           "AMP_TEST_END_EVENT",
                           "AMP_RECEIVER_REPORT_EVENT",
                           "SHORT_RANGE_MODE_CHANGE_COMPLETE_EVENT",
                           "AMP_STATUS_CHANGE_EVENT",
};
