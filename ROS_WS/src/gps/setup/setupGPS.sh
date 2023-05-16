#!/bin/bash

# Add user to the 'dialout' group
sudo usermod -a -G dialout $USER

# Create a udev rule file
sudo tee /etc/udev/rules.d/99-rover-gps.rules > /dev/null <<EOF
SUBSYSTEM!="tty", GOTO="gps_rules_end"
ATTRS{idVendor}=="1546", MODE="0666", SYMLINK+="ttyACMUbloxGPS", GROUP="dialout"
LABEL="gps_rules_end"
EOF

# Reload udev rules
sudo udevadm control --reload-rules && sudo udevadm trigger