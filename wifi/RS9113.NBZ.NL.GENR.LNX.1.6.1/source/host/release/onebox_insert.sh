
cat /dev/null > /var/log/messages
cmd=`lsmod | grep rsi_91x`
if [ "$cmd" ]; then
echo "Removing the Redpine open source kernel binaries";
rmmod rsi_usb.ko
rmmod rsi_sdio.ko
rmmod rsi_91x.ko
fi

insmod onebox_common_gpl.ko

insmod wlan.ko
insmod wlan_wep.ko
insmod wlan_tkip.ko
insmod wlan_ccmp.ko
insmod wlan_aes_cmac.ko
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

#BT RF Type

#0 - EXTERNAL RF
#1 - INTERNAL RF
BT_RF_TYPE=1

#Default Value for BLE Tx Power Index is 8
# Range for the BLE Tx Power Index is 0 to 18
BLE_TX_PWR_INX=8

#BIT(0) - BLE_DUTY_CYCLING
#BIT(1) - BLR_DUTY_CYCLING
BLE_PWR_SAVE_OPTIONS=0

PARAMS=$PARAMS" bt_rf_tx_power_mode=$BT_RF_TX_POWER_MODE"
PARAMS=$PARAMS" bt_rf_rx_power_mode=$BT_RF_RX_POWER_MODE"
PARAMS=$PARAMS" bt_rf_type=$BT_RF_TYPE"
PARAMS=$PARAMS" ble_tx_pwr_inx=$BLE_TX_PWR_INX"
PARAMS=$PARAMS" ble_pwr_save_options=$BLE_PWR_SAVE_OPTIONS"

insmod onebox_bt_nongpl.ko $PARAMS 
insmod onebox_bt_gpl.ko

insmod onebox_zigb_nongpl.ko
insmod onebox_zigb_gpl.ko

if [ $1 ] && [ $1 == "1" ]
then
sh common_insert_flash.sh
else
sh common_insert.sh
fi
