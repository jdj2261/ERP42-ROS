#!/bin/bash

cd /etc/udev/rules.d
sudo echo 'SUBSYSTEMS=="usb", KERNEL=="ttyUSB[0-9]*", ATTRS{idVendor}=="0557", ATTRS{idProduct}=="2008", MODE="0666",SYMLINK+="aten"' > 50-aten.rules
sudo cat 50-aten.rules
sleep 1
sudo service udev restart