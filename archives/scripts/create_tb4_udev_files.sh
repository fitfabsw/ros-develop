#!/bin/bash

echo
echo ====================================================================
echo Update udev rules
echo ====================================================================
echo "remap the device serial port(ttyUSBX) to  rplidar"
echo "rplidar usb connection as /dev/rplidar , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy turtlebot4.rules to  /etc/udev/rules.d/"
tb4_udev_lines=$(cat <<EOL
SUBSYSTEM=="i2c-dev", GROUP="i2c", MODE="0666"
SUBSYSTEM=="gpio*", GROUP="gpio", MODE="0666"
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK="RPLIDAR", MODE="0666"
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="RPLIDAR"
EOL
)

# files_to_append="/etc/udev/rules.d/turtlebot4.rules"
# if ! grep -qF "$commands" $files_to_append; then
#   echo "$commands" >> $files_to_append
#   echo "指令已附加到$files_to_append文件中"
# else
#   echo "指令已經存在於$files_to_appendbashrc文件中，不需要再次附加"
# fi

udev_files="/etc/udev/rules.d/turtlebot4.rules"
echo "$tb4_udev_lines" | sudo tee $udev_files

echo ====================================================================
echo Restarting udev
echo ====================================================================
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
