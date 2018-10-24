cat /dev/null > /var/log/messages
sh wlan_enable.sh
./onebox_util rpine0 create_vap wifi0 sta sw_bmiss
# ./wpa_supplicant -i wifi0 -D bsd -c sta_settings.conf -ddddt > log1 &
