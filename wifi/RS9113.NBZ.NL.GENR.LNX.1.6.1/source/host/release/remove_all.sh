#service dhcpd stop
#pkill dhcpd
#pkill dhclient
#ifconfig wifi0 down
killall wpa_supplicant
killall hostapd
killall -9 dhcpd
rm -rf /var/run/wpa_supplicant/
sleep 2

### COMMON HAL MODULES 
rmmod onebox_gpl.ko
rmmod onebox_nongpl.ko

###WLAN MODULES
rmmod onebox_wlan_gpl.ko
rmmod onebox_wlan_nongpl.ko
rmmod wlan_scan_sta.ko
rmmod wlan_aes_cmac.ko
rmmod wlan_xauth.ko
rmmod wlan_acl.ko
rmmod wlan_tkip.ko
rmmod wlan_ccmp.ko
rmmod wlan_wep.ko
rmmod wlan.ko

###BT MODULES
rmmod onebox_bt_gpl.ko
rmmod onebox_bt_nongpl.ko

###ZIGB MODULES
rmmod onebox_zigb_gpl.ko
rmmod onebox_zigb_nongpl.ko

rmmod onebox_common_gpl.ko
