
cat /dev/null > /var/log/messages
sh wlan_enable.sh
./onebox_util rpine0 create_vap wifi0 p2p_go
wpa_supplicant -i wifi0 -D nl80211 -c p2p_nl80211.conf -ddddt > log &
