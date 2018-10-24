TOP_DIR := $(PWD)
RM=/bin/rm -f
CC=gcc
PROGNAME=rsi_wsc_zigb_app
WLAN_PROGNAME=rsi_wsc_wifi_app
INCLUDES=  -I./ -I/work/colibri-2.6-kernel/include/
LINKS= -lpthread

AUTO_SRC = ./

CFLAGS= -g -Os -Wall -fno-strict-aliasing $(INCLUDES)

CFLAGS+= -I ../inc

CFLAGS += -DENABLE_DEBUG_PRINTS
CFLAGS += -DRSI_DEBUG_PRINT
CFLAGS += -DZB_DEBUG
CFLAGS += -DLINUX_PLATFORM
CFLAGS += -D ZB_ENABLE
CFLAGS += -D RSI_ENABLE_DEBUG_PRINT
CFLAGS += -lrt

OBJS += main.o\
	rsi_zigb_linux_apis.o\
	rsi_apis.o \
	rsi_zigb_frame_desc.o \
	rsi_zigb_api.o \
	rsi_zigb_app_cb_handler.o \
	rsi_zigb_utility.o \
	rsi_nlink_main.o\
	rsi_nl_app.o
		

.c.o:
	$(RM) $@
	$(CC) $(CFLAGS)  -o $*.o  -c $*.c

all: switch

switch: clean $(OBJS)
	$(CC) $(CFLAGS) $(LINKS) -o $(PROGNAME) $(OBJS)

zb: clean $(OBJS)
	$(CC) $(CFLAGS) $(LINKS) -o $(PROGNAME) $(OBJS)

clean:
	$(RM)  $(OBJS) $(OBJS_API) $(PROGNAME) $(PROGNAME1) core *~
