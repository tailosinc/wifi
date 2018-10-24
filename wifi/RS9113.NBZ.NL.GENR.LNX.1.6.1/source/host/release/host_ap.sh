sh wlan_enable.sh
./onebox_util rpine0 create_vap wifi1 ap
#sh form_bridge.sh
sleep 1
if [ "$1" == "" ]; then
	echo "please specify a config file"
	echo "example: sh host_ap.sh ap hostapd_open.conf"
else
	./hostapd $1 -dddddtK > log_hap &
fi
