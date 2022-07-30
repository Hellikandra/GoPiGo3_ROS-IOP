#!/bin/sh

echo "Common installation for Raspberry-Pi 3/4 Ubuntu Mate 18.04/20.04"
OS_VERSION=`egrep "bionic|focal" /etc/os-release`
RP_VERSION=`egrep "Raspberry Pi 3| Raspberry Pi 4" /proc/cpuinfo`

# Check OS Version
if [ `echo $OS_VERSION | grep -c "bionic"` == 1]; then
	echo "Ubuntu MATE 18.04"
elif [ `echo $OS_VERSION | grep -c "focal"` == 1 ]; then
	echo "Ubunut MATE 20.04"
fi
else
	echo "This script stop now because you do not have the right version."
	exit 1
fi

# Check Raspberry Pi version
if [ `echo $OS_VERSION | grep -c "Raspberry Pi 3"` == 1]; then
	echo "Raspberry Pi 3"
elif [ `echo $OS_VERSION | grep -c "Raspberry Pi 4"` == 1 ]; then
	echo "Raspberry Pi 4"
fi
else
	echo "This script stop now because you do not have the right version."
	exit 1
fi
export OS_VERSION
export RP_VERSION
./rpi_setup.sh