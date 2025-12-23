#!/bin/bash
sudo rm   /etc/udev/rules.d/myarduino.rules
sudo rm   /etc/udev/rules.d/mycamera.rules
sudo rm   /etc/udev/rules.d/mycar.rules
sudo rm   /etc/udev/rules.d/mylidar.rules
sudo service udev reload
sudo service udev restart
