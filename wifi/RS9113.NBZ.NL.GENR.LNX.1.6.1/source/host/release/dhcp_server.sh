cp -rf $(pwd)/dhcpd.conf /etc/
cp -rf $(pwd)/dhcpd.conf /etc/dhcp/
/sbin/ifconfig $1 192.168.2.1
sleep 1
/sbin/service dhcpd restart
