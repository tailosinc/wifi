FROM armv7/armhf-ubuntu:16.04

ENV DEVICES_WS="/home/devices" \
    LC_ALL=C.UTF-8 \
    DEBIAN_FRONTEND=noninteractive

WORKDIR $DEVICES_WS

# Install apt packages
COPY install/*-requirements.txt install/
RUN apt-get update && \
    apt-get -y -qq -o Dpkg::Use-Pty=0 install --no-install-recommends --fix-missing $(cat install/apt-requirements.txt) && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/

# Get linux headers for ResinOS
COPY install/get_resin_headers.sh install/
RUN ./install/get_resin_headers.sh raspberrypi3 '2.12.7+rev2.dev'

# Build wifi driver
COPY wifi wifi
RUN mv kernel_modules_headers/ wifi/RS9113.NBZ.NL.GENR.LNX.1.6.1/source/host/ && \
    cd wifi/RS9113.NBZ.NL.GENR.LNX.1.6.1/source/host && \
    make --quiet

# Run
COPY install install
CMD ["bash", "install/launch.bash"]
