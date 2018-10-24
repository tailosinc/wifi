#!/bin/sh

#############################################################################
#  This script will bring up the bridge interface for eth0 and vap0
#############################################################################

echo "add the bridge interface"
sudo brctl addbr br0

echo "set ip address to the bridge interface"
sudo ifconfig br0 192.168.1.1 netmask 255.255.255.0

echo "starting bridge interface"
sudo ifconfig br0 up

#echo "disabling STP"
#brctl setfd bri0 1
#brctl stp bri0 off

interface=`ifconfig |grep eth |cut -f1 -d\ `
echo "adding $interface to bridge"
sudo brctl addif br0 `ifconfig | grep eth | cut -f1 -d\ `
sudo ifconfig `ifconfig -a | grep eth | cut -f1 -d\ ` up 0.0.0.0
sudo brctl addif br0 vap0 ; sudo ifconfig vap0 up 0.0.0.0 ;
echo "finished"
