#!/bin/bash
sudo cp myarduino.rules  /etc/udev/rules.d
sudo cp mycamera.rules  /etc/udev/rules.d
sudo cp mycar.rules  /etc/udev/rules.d
sudo cp mylidar.rules  /etc/udev/rules.d

sudo service udev reload
sudo service udev restart
