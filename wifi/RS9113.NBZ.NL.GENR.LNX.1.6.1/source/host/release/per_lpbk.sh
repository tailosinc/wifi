cat /dev/null > /var/log/messages

insmod onebox_common_gpl.ko
#dmesg -c > /dev/null
#sh dump_msg.sh &
#dmesg -n 7

insmod wlan.ko
insmod wlan_wep.ko
insmod wlan_tkip.ko
insmod wlan_ccmp.ko
insmod wlan_acl.ko
insmod wlan_xauth.ko
insmod wlan_scan_sta.ko
insmod onebox_wlan_nongpl.ko
insmod onebox_wlan_gpl.ko

#Power_mode type
# 0 - HIGH  POWER MODE
# 1 - MEDIUM POWER MODE 
# 2 - LOW POWER MODE
BT_RF_TX_POWER_MODE=0
BT_RF_RX_POWER_MODE=0

PARAMS=$PARAMS" bt_rf_tx_power_mode=$BT_RF_TX_POWER_MODE"
PARAMS=$PARAMS" bt_rf_rx_power_mode=$BT_RF_RX_POWER_MODE"

insmod onebox_bt_nongpl.ko $PARAMS 
insmod onebox_bt_gpl.ko

insmod onebox_zigb_nongpl.ko
insmod onebox_zigb_gpl.ko
#Driver Mode 
#						 3 PER + Loopback + Calib
#            4 PER + Loopback
DRIVER_MODE=3


COEX_MODE=1
TA_AGGR=4
SKIP_FW_LOAD=0
FW_LOAD_MODE=2

insmod onebox_nongpl.ko	driver_mode=$DRIVER_MODE firmware_path=$PWD/firmware/ onebox_zone_enabled=0x1 coex_mode=$COEX_MODE ta_aggr=$TA_AGGR skip_fw_load=$SKIP_FW_LOAD fw_load_mode=$FW_LOAD_MODE
insmod onebox_gpl.ko

