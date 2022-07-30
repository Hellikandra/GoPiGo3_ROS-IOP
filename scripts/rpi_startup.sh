#!/bin/bash
# command line to do at the start-up.
# put the script in /etc/init.d/
# chmod +x /etc/init.d/rpi_startup.sh
# watch out !!! ou need to create a simlink to be ready to launch the script at startup
# ln -s /etc/init.d/rpi_startup.sh /etc/rc.d/
# netstat -nctl
# export PIGPIO_ADDR=localhost ou 127.0.0.1
# export PIGPIO_PORT=8888
# printenv | grep PIGP
# https://github.com/guymcswain/pigpio-client/wiki/Install-and-configure-pigpiod

sudo killall pigpiod
if [ [-z "${PIGPIO_ADDR}"] || [-z "${PIGPIO_PORT}"] ]; then
	echo "var is unset"
	export PIGPIO_ADDR=localhost
	export PIGPIO_PORT=8888
else
	echo "var is set"
fi
printenv | grep PIGPIO
pigpiod -v
sudo pigpiod
# quick test
# pigs t