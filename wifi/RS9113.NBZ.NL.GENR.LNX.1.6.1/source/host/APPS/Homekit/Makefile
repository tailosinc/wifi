HAP = ./Personal-HomeKit-HAP-master
LINK = /usr/lib/libcrypto.so.10 /usr/lib/libssl.so.10 -ldl -lpthread
CFLAG = -g -Os 
INCLUDE = -I./
INCLUDE += -I./Personal-HomeKit-HAP-master
CC = gcc
CPP = g++
cpy=cp
PHK_LIBNAME = $(HAP)/libphk
ifeq ($(OS),Windows_NT)
    PHK_LIBFILE=$(PHK_LIBNAME).dll
    ifeq ($(PROCESSOR_ARCHITECTURE),AMD64)
    endif
    ifeq ($(PROCESSOR_ARCHITECTURE),x86)
    endif
else
    UNAME_S := $(shell uname -s)
    ifeq ($(UNAME_S),Linux)
        LINK +=  /usr/lib/libdns_sd.so.1
        PHK_LIBFILE=$(PHK_LIBNAME).so
    endif
    ifeq ($(UNAME_S),Darwin)
        PHK_LIBFILE=$(PHK_LIBNAME).dylib
    endif
    UNAME_P := $(shell uname -p)
endif

OBJFILE  =   main.o
OBJFILE +=   iap_main.o
OBJFILE +=    $(HAP)/chacha20.o
OBJFILE +=   $(HAP)/curve25519.o 
OBJFILE +=   $(HAP)/ed25519.o
OBJFILE +=   $(HAP)/poly1305.o 
OBJFILE +=   $(HAP)/rfc6234-master/hkdf.o
OBJFILE +=   $(HAP)/rfc6234-master/hmac.o
OBJFILE +=   $(HAP)/rfc6234-master/sha.o
OBJFILE +=   $(HAP)/Homekit_main.o 
OBJFILE +=   $(HAP)/PHKAccessory.o
OBJFILE +=   $(HAP)/PHKControllerRecord.o
OBJFILE +=   $(HAP)/PHKNetworkIP.o 
OBJFILE +=   $(HAP)/PHKArduinoLightInterface.o 
OBJFILE +=   $(HAP)/srp/srp.o
OBJFILE +=   $(HAP)/srp/cstr.o
OBJFILE +=   $(HAP)/srp/t_math.o 
OBJFILE +=   $(HAP)/srp/srp6_server.o
OBJFILE +=   $(HAP)/srp/t_conf.o
OBJFILE +=   $(HAP)/srp/t_conv.o
OBJFILE +=   $(HAP)/srp/t_pw.o
OBJFILE +=   $(HAP)/srp/t_misc.o
OBJFILE +=   $(HAP)/srp/t_truerand.o 
OBJFILE +=   $(HAP)/srp/t_read.o
OBJFILE +=   $(HAP)/Accessory.o

all: PHK
PHK: $(OBJFILE)
	$(CPP) $(CFLAG)  -o PHK $(OBJFILE) $(LINK)
phklib: PHK
	$(CPP) $(CFLAG) -dynamiclib $(LINK) -o $(PHK_LIBFILE) $(PHK_OBJFILES)
$(HAP)/chacha20.o: $(HAP)/Chacha20/chacha20_simple.c $(HAP)/Chacha20/chacha20_simple.h
	$(CC) $(CFLAG) -w -o $(HAP)/chacha20.o -c $(HAP)/Chacha20/chacha20_simple.c
$(HAP)/curve25519.o: $(HAP)/curve25519/curve25519-donna.c $(HAP)/curve25519/curve25519-donna.h
	$(CC) $(CFLAG) -w -o $(HAP)/curve25519.o -c $(HAP)/curve25519/curve25519-donna.c
$(HAP)/ed25519.o: $(HAP)/ed25519-donna/ed25519.c $(HAP)/ed25519-donna/ed25519.h
	$(CC) $(CFLAG) $(INCLUDE) -w -o $(HAP)/ed25519.o -c $(HAP)/ed25519-donna/ed25519.c
$(HAP)/poly1305.o: $(HAP)/poly1305-opt-master/poly1305.c $(HAP)/poly1305-opt-master/poly1305.h
	$(CC) $(CFLAG) -w -o $(HAP)/poly1305.o -c $(HAP)/poly1305-opt-master/poly1305.c
$(HAP)/rfc6234-master/%.o: $(HAP)/rfc6234-master/%.c
	$(CC) $(CFLAG) $(INCLUDE) -w -c $< -o $@
%.o: %.c
	$(CC) $(CFLAG) $(INCLUDE) -w -c $< -o $@
$(HAP)/srp/%.o: $(HAP)/srp/%.c
	$(CC) $(CFLAG) $(INCLUDE) -lcrypto -w -c $< -o $@
$(HAP)/%.o: $(HAP)/%.c
	$(CC) $(CFLAG) $(INCLUDE) -w -c $< -o $@
$(HAP)/%.o: $(HAP)/%.cpp
	$(CPP) $(CFLAG) $(INCLUDE) -w -c $< -o $@

#@echo -e "\033[32mcopying to release folder...\033[0m"
#$(cpy) PHK home_kit
#$(cpy) PHK ../../release/
clean:
	rm -rf *.o $(HAP)/*.o $(HAP)/Chacha20/*.o $(HAP)/curve25519/*.o $(HAP)/ed25519-donna/*.o $(HAP)/poly1305-opt-master/*.o $(HAP)/rfc6234-master/*.o  $(HAP)/srp/*.o PHK  home_kit $(PHK_LIBFILE)
