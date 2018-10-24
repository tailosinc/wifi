
cat /dev/null > /var/log/messages
#dmesg -c > /dev/null
#sh dump_msg.sh &
#dmesg -n 7


#Driver Mode 1 END-TO-END mode, 
#            2 RF Evaluation Mode

DRIVER_MODE=5

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
#							14   WLAN ACCESS POINT + BT CLASSIC MODE+ BT LE MODE

#							16   ZIGBEE MODE/ ZIGBEE PER MODE
#							17   WLAN STATION + ZIGBEE

COEX_MODE=1

#To enable TA-level SDIO aggregation set 1 else set 0 to disable it.
TA_AGGR=4

#Disable Firmware load set 1 to skip FW loading through Driver else set to 0.
SKIP_FW_LOAD=0

#FW Download Mode	
#	1 - Full Flash mode with Secondary Boot Loader
#	2 - Full RAM mode with Secondary Boot Loader
#	3 - Flash + RAM mode with Secondary Boot Loader
#	4 - Firmware loading WITHOUT Secondary Boot Loader
# Recommended to use the default mode 1
FW_LOAD_MODE=4

#ps_handshake_mode
#   1 - No hand shake Mode
#	2 - Packet hand shake Mode
#	3 - GPIO Hand shake Mode
###########Default is Packet handshake mode=2
HANDSHAKE_MODE=2

#SDIO Clock speed
SDIO_CLOCK_SPEED=12000

#Antenna diversity enable
RSI_ANTENNA_DIVERSITY=0

#Antenna Selection
ANT_SEL_VAL=2         # 2 Internal Antenna Selection
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

#COUNTRY Selection
# 0 World Domain
# 840 US Domain Maps to US Region
# 276 Germany Maps to EU Region
# 392 Japan Maps to Japan Region
SET_COUNTRY_CODE=0  

PARAMS=" driver_mode=$DRIVER_MODE"
PARAMS=$PARAMS" firmware_path=$PWD/firmware/"
PARAMS=$PARAMS" onebox_zone_enabled=0xFFFFFF"
PARAMS=$PARAMS" ta_aggr=$TA_AGGR"
PARAMS=$PARAMS" skip_fw_load=$SKIP_FW_LOAD"
PARAMS=$PARAMS" fw_load_mode=$FW_LOAD_MODE"
PARAMS=$PARAMS" sdio_clock=$SDIO_CLOCK_SPEED"
PARAMS=$PARAMS" enable_antenna_diversity=$RSI_ANTENNA_DIVERSITY"
PARAMS=$PARAMS" coex_mode=$COEX_MODE"
#PARAMS=$PARAMS" ps_handshake_mode=$HANDSHAKE_MODE"
PARAMS=$PARAMS" obm_ant_sel_val=$ANT_SEL_VAL"
PARAMS=$PARAMS" wlan_rf_power_mode=$WLAN_RF_PWR_MODE"
PARAMS=$PARAMS" bt_rf_power_mode=$BT_RF_PWR_MODE"
PARAMS=$PARAMS" zigb_rf_power_mode=$ZIGB_RF_PWR_MODE"
PARAMS=$PARAMS" country_code=$SET_COUNTRY_CODE"

#insmod common_nongpl.ko $PARAMS
insmod onebox_nongpl.ko	$PARAMS   
#insmod onebox_nongpl.ko  
insmod onebox_gpl.ko

