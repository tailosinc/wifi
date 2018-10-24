#! /usr/bin/perl

#system("/home/rsi/release/wpa_cli -i vap0 scan");
system("/home/rsi/release/wpa_cli -i vap0 p2p_find");
sleep 5;
#system("/home/rsi/release/wpa_cli -i vap0 p2p_listen");
system("/home/rsi/release/wpa_cli -i vap0 p2p_peers");
