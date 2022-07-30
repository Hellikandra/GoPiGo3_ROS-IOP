#!/bin/bash
# GoPiGo3 environment install
# Instruction comes from the Github/DexterInd
# Due to the fact that we are on our own OS (Ubuntu MATE), it is recommanded to do a git clone and not a curl
# Quick Install 
# # curl -kL dexterindustries.com/update_gopigo3 | bash # can be used to update the GoPiGo3
# Virtual environement Installation
# # curl -kL dexterindustries.com/update_gopigo3 | bash -s -- --user-local --bypass-gui-installation
echo "Install curl and git package"
sudo apt install curl git -y

# Installation of the GoPiGo3 Firmware and Drivers
curl -kL dexterindustries.com/update_gopigo3 | bash
curl -kL dexterindustries.com/update_sensors | bash

# Installation of the Virtual Environment
curl -kL dexterindustries.com/update_gopigo3 | bash -s -- --user-local --bypass-gui-installation

# Installation of the GoPiGo3 for own Operating System (not DexterOS)
git clone http://www.github.com/DexterInd/GoPiGo3.git /home/pi/Dexter/GoPiGo3
bash /home/pi/Dexter/GoPiGo3/Install/install.sh

# https://help.ubuntu.com/community/EnvironmentVariables
# in case of error with pigpio
# http://abyz.me.uk/rpi/pigpio/download.html
# https://github.com/joan2937/pigpio/wiki/Releases
# https://github.com/DexterInd/DI_Sensors