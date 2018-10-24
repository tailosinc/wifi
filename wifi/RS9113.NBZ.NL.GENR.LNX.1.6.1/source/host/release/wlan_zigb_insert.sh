insmod onebox_common_gpl.ko

insmod wlan.ko
insmod wlan_wep.ko
insmod wlan_tkip.ko
insmod wlan_ccmp.ko
insmod wlan_acl.ko
insmod wlan_xauth.ko
insmod wlan_scan_sta.ko
insmod onebox_wlan_nongpl.ko
insmod onebox_wlan_gpl.ko

insmod onebox_zigb_nongpl.ko
insmod onebox_zigb_gpl.ko

sh common_insert.sh

WLAN_ZIGB=5
./onebox_util rpine0 enable_protocol $WLAN_ZIGB
