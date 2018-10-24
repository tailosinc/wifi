# RS9113 Driver Test Setup

## Instructions

1. Create an account on [resin.io](https://dashboard.resin.io/signup) by signing up with your github or gmail account.

1. Once you have created an account and logged in, navigate to the [apps page](https://dashboard.resin.io/apps). 

1. On the top-left side of the page, enter the following details and hit the button to create a new application:  
  APPLICATION NAME: `WifiTest`  
  DEFAULT DEVICE TYPE: `Raspberry Pi 3`  
  APPLICATION TYPE: `Starter`

1. As soon as you click the button, you will be navigated to a new page. On the left hand side of this page, click on FLEET CONFIGURATION and scroll to the bottom. You will see a green button on the right that says 'Add Custom Variable'. Click that, enter the following details in the popup, and click 'Add':  
  NAME: `RESIN_HOST_CONFIG_dtoverlay`  
  VALUE: `sdio,sdio_overclock=5`

1. Scroll to the top of the page, and click the DEVICES button on the left side to get back to the previous page. Click the button on the top left that says 'Add Device', fill in the following details, and click 'Download ResinOS':  
  SELECT DEVICE TYPE: `Raspberry Pi 3`  
  SELECT RESINOS VERSION: `v2.12.7+rev2`  
  SELECT EDITION: `Development`  
  NETWORK CONNECTION: `Wifi + Ethernet`  
  WIFI SSID: `<your ssid>`  
  WIFI PASSPHRASE: `<your password>`

1. Once the download finishes, flash the image to a new 16GB+ SD card using [etcher](https://etcher.io/) or any similar tool.

1. Plug the SD card into your Raspberry Pi 3, plug in ethernet, and power it up. It will show up online in a few minutes on the devices page on resin.io under a random name.

1. Clone this repository to your laptop. Look through the contents -- it basically just contains the RS9113.NBZ.NL.GENR.LNX.1.6.1 driver with minor modifications (see wifi/README.md for details) and scripts to build and launch it.

1. Change directory into this repository and run: `git remote add resin <your_resin_username>@git.resin.io:<your_resin_username>/wifitest.git`

1. Run: `git push resin master`. You may be prompted to set up SSH keys the very first time you try this. To do so, click on your name on the top right corner of resin.io, click on preferences, and navigate to the SSH Keys tab. You can manually enter your SSH key here. Once this works, you will see apt packages get installed, the driver get built, and everything uploaded as a Docker container. Within a few minutes, your Raspberry Pi 3 will start downloading this container. The download progress can be tracked online on the devices page.

1. Once the download finishes, you will see the container get launched in the logs window on resin.io, and error messages from the driver indicating that no redpine module was not found. You can also look at dmesg logs and other details by SSHing into the Host OS through their web interface. If you want to SSH in locally, use port 22222 by running the command: `ssh root@<ip address> -p 22222`

1. Power off the Raspberry Pi 3, and connect the RS9113 EVK to the Pi as follows. This [image](https://pinout.xyz/pinout/sdio) might be helpful
   * USB  -> External USB Power
   * VDD  -> Pin 17 (3V3)
   * VSS  -> Pin 14, 20 (GND)
   * CLK  -> Pin 15 (BCM 22)
   * CMD  -> Pin 16 (BCM 23)
   * DAT0 -> Pin 18 (BCM 24)
   * DAT1 -> Pin 22 (BCM 25)
   * DAT2 -> Pin 37 (BCM 26)
   * DAT3 -> Pin 13 (BCM 27)
 
1. Power the EVK over USB, wait a few seconds, and then power the Raspberry Pi 3 back on. SSH in to look at dmesg logs. The wifi module should come up and function correctly.
