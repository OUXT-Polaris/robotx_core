#!/bin/bash

echo KERNEL=='"'ttyACM*'"', ATTRS{idVendor}=='"'0d28'"', ATTRS{idProduct}=='"'0204'"', GROUP='"'dialout'"', MODE='"'0666'"' > mbed.rules
sudo mv ./mbed.rules /etc/udev/rules.d


