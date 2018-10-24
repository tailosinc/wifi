==================================================================
 README
==================================================================
This package contains the following folders/files :

I.  main.c
     This file contains the main of the Homekit demo application
       
II. Makefile 
     Makefile for the Homekit demo application.

III.Personal-HomeKit-HAP-master
     Application for the HomeKit which initiates and integrates the Accesory.
    

Procedure:
     1.Install the mDNS service as follows
        Go to host/APPS/WAC_POSIX_Server_1.22/mDNSResponder-567/mDNSPosix and install mdns service by command "make install os=linux"
     2.Go to path  host/APPS/Homekit/ and compile the homekit application by command "make".
     3.Copy the generated binary PHK to the release folder "cp PHK ../../release/" 
     4.Go to the release path and run the binary as ./PHK which will start in station mode and connect to the AP that was configured in sta_settings.conf file.
     5.Connect to the same ap with the apple iphone/ipad and launch the insteon app in apple phone and select the scanned accesory,enter setup code 
       "52312643" which will add our accesory to the apple iphone/ipad and able to control the accesories say light,fan etc...
     6.we can enable other attributes like lock management,outlet,window,window cover ... in host/APPS/Homekit/Personal-HomeKit-HAP-master/Configuration.h.By default Light,Fan,GDO,Lock Mechanism and siwtch are working.


KNOWN ISSUES:
     1.Delay response from the MDNS responder in some casses.
     2.We have to remove /var/PHK_Controller for the fresh connection to happen after successful disconnection.
     3.Have to delete added devices in the insteon app(edit devices) in the apple device before making a new connection.




