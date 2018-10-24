#/*
#* Copyright (c) 2017 Redpine Signals Inc. All rights reserved.
#*
#* Redistribution and use in source and binary forms, with or without
#* modification, are permitted provided that the following conditions are met:
#*
#* 	1. Redistributions of source code must retain the above copyright
#* 	   notice, this list of conditions and the following disclaimer.
#*
#* 	2. Redistributions in binary form must reproduce the above copyright
#* 	   notice, this list of conditions and the following disclaimer in the
#* 	   documentation and/or other materials provided with the distribution.
#*
#* 	3. Neither the name of the copyright holder nor the names of its
#* 	   contributors may be used to endorse or promote products derived from
#* 	   this software without specific prior written permission.
#*
#* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
#* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
#* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
#* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
#* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
#* INTERRUPTION). HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
#* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
#* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#* POSSIBILITY OF SUCH DAMAGE.
#*/

cpy=cp -rf

ifndef ROOT_DIR
ROOT_DIR=$(PWD)
endif


include $(ROOT_DIR)/.config
include $(ROOT_DIR)/config/make.config

#EXTRA_CFLAGS += -DENABLE_PER_MODE
#CFLAGS += -DPER_ENHANCEMENTS
 
EXTRA_CFLAGS += -MMD -O2 -Wall -g
#CFLAGS += -DCHIP_9116
#chip=9116

#CFLAGS += -DRSSI_AVG
#CFLAGS += -DMODE_11AH

OBJS =onebox_util.o

all: onebox_util copy
onebox_util: $(OBJS)
	$(CC) -o onebox_util $(OBJS)
copy:
	$(CC) -o receive receive.c $(CFLAGS)  
	$(CC) -o transmit transmit.c $(CFLAGS)
	$(CC) -o doth_over_ride doth_over_ride.c $(CFLAGS)
	$(CC) -o wlan_iqs wlan_iq_capture.c $(CFLAGS)
	$(CC) -o transmit_packet transmit_packet.c
	#$(CC) -o nl80211_util nl80211_util.c
	$(CC) -o bbp_util matlab_utils.c -lm -lpthread $(CFLAGS)
	$(CC) -o sniffer_app sniffer.c
	$(CC) -o zb_transmit zigb/zb_transmit.c zigb/rsi_nl_app.c
	$(CC) -o zb_util zigb/zb_utils.c zigb/rsi_nl_app.c
	$(CC) -o zb_bbp_util zigb/matlab_utils.c zigb/rsi_nl_app.c
	$(CC) -o bt_util bt_utils/bt_util.c bt_utils/rsi_nl_app.c -DCHIP_9116
	$(CC) -o bt_transmit bt_utils/bt_transmit.c bt_utils/rsi_nl_app.c -DCHIP_9116
#	$(CC) -o bt_ber bt_utils/bt_ber.c bt_utils/rsi_nl_app.c
#	$(CC) -o bt_stats bt_utils/bt_stats.c bt_utils/rsi_nl_app.c
	$(CC) -o bt_bbp_utils bt_utils/matlab_utils.c bt_utils/rsi_nl_app.c
	$(CC) -o bt_receive bt_utils/bt_receive.c bt_utils/rsi_nl_app.c -DCHIP_9116
	$(CC) -o append_pwr_values append_pwr_values.c 	
	$(CC) -o START_RS9116_NBZ_D0N master_app.c -lm -lpthread -DDUAL_BAND -DANT_0 -DNLINK 
ifeq ($(chip),9116)
	$(CC) -o ble_aoa_aod_transmit bt_utils/ble_aoa_aod_transmit.c bt_utils/rsi_nl_app.c 
	$(CC) -o ble_aoa_aod_receive bt_utils/ble_aoa_aod_receive.c bt_utils/rsi_nl_app.c 
	$(CC) -o bt_br_edr_transmit bt_utils/bt_br_edr_transmit.c bt_utils/rsi_nl_app.c -DCHIP_9116
	$(CC) -o ble_transmit bt_utils/bt_le_transmit.c bt_utils/rsi_nl_app.c -DCHIP_9116
	$(CC) -o bt_br_edr_receive bt_utils/bt_br_edr_receive.c bt_utils/rsi_nl_app.c -DCHIP_9116
	$(CC) -o ble_receive bt_utils/bt_le_receive.c bt_utils/rsi_nl_app.c -DCHIP_9116
	$(CC) -o bt_util bt_utils/bt_util.c bt_utils/rsi_nl_app.c -DCHIP_9116
	$(CC) -o bt_bbp_utils bt_utils/matlab_utils.c bt_utils/rsi_nl_app.c -DCHIP_9116
else
	$(CC) -o bt_util bt_utils/bt_util.c bt_utils/rsi_nl_app.c -DCHIP_9116
	$(CC) -o bt_bbp_utils bt_utils/matlab_utils.c bt_utils/rsi_nl_app.c
endif
	@echo -e "\033[32mcopying to release folder...\033[0m"

ifeq ($(chip) , 9116)
	$(cpy) onebox_util transmit transmit_packet receive bt_br_edr_receive ble_receive bbp_util sniffer_app bt_util bt_br_edr_transmit ble_transmit ble_aoa_aod_transmit ble_aoa_aod_receive bt_bbp_utils zb_transmit zb_util zb_bbp_util START_RS9116_NBZ_D0N wlan_iqs doth_over_ride ../release/
else
	$(cpy) onebox_util transmit transmit_packet receive bt_receive bbp_util sniffer_app bt_util bt_transmit bt_bbp_utils zb_transmit zb_util zb_bbp_util START_RS9116_NBZ_D0N ../release/
endif

clean:
	rm -f onebox_util onebox_util.d onebox_util.o
	rm -rf onebox_util transmit transmit_packet  receive bt_br_edr_receive bt_br_edr_transmit ble_transmit ble_receive bt_receive bbp_util sniffer_app bt_util bt_freq_hop bt_ber bt_stats bt_transmit ble_aoa_aod_transmit ble_aoa_aod_receive bt_bbp_utils zb_transmit zb_util zb_bbp_util append_pwr_values START_RS9116_NBZ_D0N doth_over_ride wlan_iqs
