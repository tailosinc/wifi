
cat /dev/null > /var/log/messages
#dmesg -c > /dev/null
#sh dump_msg.sh &
#dmesg -n 7


#Driver Mode 1 END-TO-END mode, 
#            2 RF Evaluation Mode

DRIVER_MODE=1

# COEX MODE:                  
#							1    WLAN STATION /WIFI-Direct/WLAN PER
#							2    WLAN ACCESS POINT(including muliple APs on different vaps)
#							3    WLAN ACCESS POINT + STATION MODE(on multiple vaps)

#							4    BT CLASSIC MODE/BT CLASSIC PER MODE
#							5    WLAN STATION + BT CLASSIC MODE
#							6    WLAN ACCESS POINT + BT CLASSIC MODE
#							8    BT LE MODE /BT LE PER MODE
#							9    WLAN STATION + BT LE MODE
#							12   BT CLASSIC + BT LE MODE 							
#							13   WLAN STATION + BT CLASSIC MODE + BT LE MODE
#							14   WLAN ACCESS POINT + BT CLASSIC MODE+ BT LE MODE
#
#							16   ZIGBEE MODE/ ZIGBEE PER MODE
#							17   WLAN STATION + ZIGBEE
#							32   ZIGBEE COORDINATOR MODE
#                           48   ZIGBEE ROUTER MODE

COEX_MODE=1

#To enable TA-level SDIO aggregation set 1 else set 0 to disable it.
TA_AGGR=3

#Disable Firmware load set 1 to skip FW loading through Driver else set to 0.
SKIP_FW_LOAD=0

#FW Download Mode	
#	1 - Full Flash mode with Secondary Boot Loader
#	2 - Full RAM mode with Secondary Boot Loader
#	3 - Flash + RAM mode with Secondary Boot Loader
#	4 - Firmware loading WITHOUT Secondary Boot Loader
#	5 - Firmware loading WITHOUT Secondary Boot Loader with Scatters
# Recommended to use the default mode 1
#FW_LOAD_MODE=5
FW_LOAD_MODE=1

#Module Type
# 0 - B8
# 1 - Q7
# 2 - M15B
# 3 - M15DB-T
# 4 - M15SB
# 5 - M7DB
# Default module type is 0 - B8
MODULE_TYPE=0

#PLL Mode Selection
# 0 - PLL_MODE0
# 1 - PLL_MODE1
# 2 - PLL_MODE2
# Default PLL mode is 0 - PLL_MODE0
PLL_MODE=0

#RF Type selection only for 2G
# 0 - External RF
# 1 - Internal RF
# Default value for RF type selection is 1 - Internal RF
RF_TYPE_FOR_2G=1

#Power Save options
# 0 - Disable Duty Cycling & Undestined Packet Drop
# 1 - Enable Duty Cycling
# 2 - Enable Undestined Packet Drop
# 3 - Enable Duty Cycling & Undestined Packet Drop
# Default value for power save option is 3 - Enable Duty Cycling & Undestined Packet Drop
POWER_SAVE_OPTION=3

#LP/HP Chain Selection in standby associated mode
# 0 - HP Chain Enabled
# 1 - LP Chain Enabled
# Default value for Chain Selection is 1 - LP Chain Enabled
STANDBY_ASSOC_CHAIN_SEL=1

#LMAC BEACON DROP Feature Options
# 0 - Disable LMAC BEACON DROP Feature
# 1 - Enable LMAC BEACON DROP Feature
# Default value for LMAC BEACON DROP Feature Options is 1 - Enable LMAC BEACON DROP Feature
LMAC_BEACON_DROP=1

#Host Interface on Demand Feature Options
# 0 - Disable Host Interface on Demand Feature
# 1 - Enable Host Interface on Demand Feature
# Default value for Host Interface on Demand Feature Options is 0 - Disable Host Interface on Demand Feature
HOST_INTF_ON_DEMAND=0

#Extended Options
# 0 - NA
# Default value for Extended options is 0
EXTENDED_OPTION=0

#ulp_ps_handshake_mode
#  0 - No hand shake Mode
#  1 - GPIO Hand shake Mode 
#  2 - Packet hand shake Mode
###########Default is Packet handshake mode=2
ULP_HANDSHAKE_MODE=2
 
#lp_ps_handshake_mode
#   0 - No hand shake Mode
#   1 - GPIO Hand shake Mode
###########Default is No hand shake mode=0
LP_HANDSHAKE_MODE=0

#SDIO Clock speed
#SDIO_CLOCK_SPEED=25000
SDIO_CLOCK_SPEED=5000

#Antenna diversity enable
RSI_ANTENNA_DIVERSITY=0

#Antenna Selection
ANT_SEL_VAL=2  # 2 Internal Antenna Selection
		# 3 External Antenna Selection

####RF_POWER_MODE Selection

# 0x00 For Both TX and RX High Power 
# 0x11 For Both TX and RX Medium Power
# 0x22 For Both TX and RX LOW Power

# 0x10 For High Power TX and Medium RX Power
# 0x20 For High Power TX and LOW RX Power

# 0x01 For Medium TX and RX High Power
# 0x21 For Medium Power TX and LOW RX Power

# 0x02 For Low Power TX and RX High Power 
# 0x12 For LOW Power TX and Medium RX Power


WLAN_RF_PWR_MODE=0x00
BT_RF_PWR_MODE=0x00
ZIGB_RF_PWR_MODE=0x00

#ONBOARD ANT SELECTION
# 1 Module is having inbuit antenna
# 0 EXt Antenna
ONBOARD_ANT_SEL=1

#RETRY_COUNT value (valid range is 7 to 15)
SET_RETRY_COUNT=15

#COUNTRY Selection
# 0 World Domain
# 840 US Domain Maps to US Region
# 276 Germany Maps to EU Region
# 392 Japan Maps to Japan Region
SET_COUNTRY_CODE=0

SET_BT_FEATURE_BITMAP=0x00

UART_DEBUG=0x0

#Enter the distance of the PEER in meters 
#This is to configure ACK and SLOT related timeout values.
CONFIG_PEER_DISTANCE=0

#Enter the Tx Pkt Life time in msecs. 
#This is to configure tx pkt life time in LMAC.
CONFIG_TXPKT_LIFETIME=0

PARAMS=" driver_mode=$DRIVER_MODE"
PARAMS=$PARAMS" firmware_path=/mnt/data/docker/volumes/${RESIN_APP_ID}_resin-data/_data/firmware/"
PARAMS=$PARAMS" onebox_zone_enabled=0x1"
PARAMS=$PARAMS" ta_aggr=$TA_AGGR"
PARAMS=$PARAMS" skip_fw_load=$SKIP_FW_LOAD"
PARAMS=$PARAMS" fw_load_mode=$FW_LOAD_MODE"
PARAMS=$PARAMS" sdio_clock=$SDIO_CLOCK_SPEED"
PARAMS=$PARAMS" enable_antenna_diversity=$RSI_ANTENNA_DIVERSITY"
PARAMS=$PARAMS" coex_mode=$COEX_MODE"
PARAMS=$PARAMS" lp_ps_handshake_mode=$LP_HANDSHAKE_MODE"
PARAMS=$PARAMS" ulp_ps_handshake_mode=$ULP_HANDSHAKE_MODE"
PARAMS=$PARAMS" obm_ant_sel_val=$ANT_SEL_VAL"
PARAMS=$PARAMS" user_onboard_ant_val=$ONBOARD_ANT_SEL"
PARAMS=$PARAMS" wlan_rf_power_mode=$WLAN_RF_PWR_MODE"
PARAMS=$PARAMS" bt_rf_power_mode=$BT_RF_PWR_MODE"
PARAMS=$PARAMS" zigb_rf_power_mode=$ZIGB_RF_PWR_MODE"
PARAMS=$PARAMS" country_code=$SET_COUNTRY_CODE"
PARAMS=$PARAMS" retry_count=$SET_RETRY_COUNT"
PARAMS=$PARAMS" bt_feature_bitmap=$SET_BT_FEATURE_BITMAP"
PARAMS=$PARAMS" uart_debug=$UART_DEBUG"
PARAMS=$PARAMS" peer_dist=$CONFIG_PEER_DISTANCE"
PARAMS=$PARAMS" module_type=$MODULE_TYPE"
PARAMS=$PARAMS" pll_mode=$PLL_MODE"
PARAMS=$PARAMS" rf_type_2g=$RF_TYPE_FOR_2G"
PARAMS=$PARAMS" pwr_save_opt=$POWER_SAVE_OPTION"
PARAMS=$PARAMS" ext_opt=$EXTENDED_OPTION"
PARAMS=$PARAMS" standby_assoc_chain_sel=$STANDBY_ASSOC_CHAIN_SEL"
PARAMS=$PARAMS" lmac_bcon_drop=$LMAC_BEACON_DROP"
PARAMS=$PARAMS" host_intf_on_demand=$HOST_INTF_ON_DEMAND"
PARAMS=$PARAMS" txpkt_lifetime=$CONFIG_TXPKT_LIFETIME"

#insmod common_nongpl.ko $PARAMS
insmod onebox_nongpl.ko	$PARAMS   
#insmod onebox_nongpl.ko  
insmod onebox_gpl.ko
