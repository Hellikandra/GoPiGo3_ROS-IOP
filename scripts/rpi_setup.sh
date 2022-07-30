#!/bin/bash
# Shell script to set up the Rpi
# check permission : ls -l rpi-setup.sh
# set permission   : sudo chmod 774 rpi-setup.sh
# start script     : ./rpi-setup.sh
# log files        : ~/.bash_history

# GoPiGo-3 : Raspberry-Pi 3 / Ubuntu MATE 18.04 / ROS Melodic
# OCU      : Raspberry-Pi 4 / Ubuntu MATE 20.04 / ROS Noetic
echo "$RP_VERSION setup on $OS_VERSION"
## Update the Ubunutu version
sudo apt update && sudo apt upgrade -y
# sudo apt list --upgradable

# sudo apt install xrdp # it is an tigerVNC
echo "Install openssh server"
sudo apt install openssh-server -y

STATUS="$(systemctl is-active ssh)"
if [ "${STATUS}" = "active" ]; then
	echo "ssh service will be stopped"
	sudo systemctl stop ssh       ## "inactive" is a service that is currently stopped and may be disable, but it can be started and become active
	# sudo systemctl disable ssh  ## is a service that is configured to no start when the system boots
else
	echo "ssh service is stopped"
fi
# Generate the new host keys
echo "Generate ssh key, enable and start service"
sudo /usr/bin/ssh-keygen -A
sudo systemctl enable ssh   ## is a service that is configured to start when the system boots
sudo systemctl start ssh    ## "active" is a service that is currently running
systemctl status ssh
# Now get the IP address of the Rpi
# test with putty the availability to access to the pi

# VNC installation. The TigerVNC was chosen by the community due to the fact that tightvncserver does not support Xserver or OpenGL which rviz requires (for ROS)
echo "Installation of the Virtual Network"
echo "Install TigerVNC suite"
sudo apt install xfce4 xfce4-goodies xorg dbus-x11 x11-xserver-utils
sudo apt install tigervnc-standalone-server tigervnc-common
# vncserver configuration
vncserver
# passw only on 8 char : *sdf123 ; view-only password ? y
# Creating default startup script /home/username/.vnc/xstartup
# Starting applications specified in /home/usernmae/.vnc/xstartup
# Log file is /home/username/.vnc/raspberrypi:1.log
# edit vncpasswd to change the password of the VNC
vncserver -kill :1

echo "xstartup configuration : "
mv ~/.vnc/xstartup ~/.vnc/xstartup.bak
nano ~/.vnc/xstartup
# cat >> ~/.vnc/xstartup <<-EOF
# 	#!/bin/sh
# 	unset SESSION_MANAGER
# 	unset DBUS_SESSION_BUS_ADDRESS
# 	exec startxfce4
# 	EOF
chmod u+x ~/.vnc/xstartup

echo "config configuration : "
nano ~/.vnc/config
# cat >> ~/.vnc/config <<-EOF
# 	geometry=1920x1080
# 	dpi=96
#   EOF
chmod u+x ~/.vnc/config

sudo apt install net-tools

echo "End of Raspberry-Pi setup"
# https://linuxize.com/post/how-to-install-and-configure-vnc-on-ubuntu-20-04/
# sudo nano /etc/systemd/system/vncserver@.service
# # sudo cat >> /etc/systemd/system/vncserver@.service <<-EOF
# # 	[Unit]
# # 	Description=Remote desktop service (VNC)
# # 	After=syslog.target network.target

# # 	[Service]
# # 	Type=simple
# # 	User=linuxize # replace User by the actual user !!
# # 	PAMName=login
# # 	PIDFile=/home/%u/.vnc/%H%i.pid
# # 	ExecStartPre=/bin/sh -c '/usr/bin/vncserver -kill :%i > /dev/null 2>&1 || :'
# # 	ExecStart=/usr/bin/vncserver :%i -geometry 1440x900 -alwaysshared -fg
# # 	ExecStop=/usr/bin/vncserver -kill :%i

# # 	[Install]
# # 	WantedBy=multi-user.target

# # 	EOF

# sudo systemctl daemon-reload
# sudo systemctl enable vncserver@1.service # port used by TigerVNC = 5900 + 1 = 5901 due to the fact that the service start @1
# sudo systemctl start vncserver@1.service
# sudo systemctl status vncserver@1.service
# # netstat -nctl
# # You can now do a test :
# # On Linux / macOS :
# # ssh -L 5901:127.0.0.1:5901 -N -f -l <username> <server_ip_address>
# # On Windows  use putty :
# # particularity : on SSH-> Tunnels :
# # # # Source Port : 5901
# # # # Destination : server_ip_address:5901
# # # # Click on Add 
# on windows with Windows PowerShell :
# Test-NetConnection 192.168.0.124 - port 5901