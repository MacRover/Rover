#!/bin/bash

# Reinstall ros-melodic-cv-bridge package
sudo apt-get update
sudo apt-get --reinstall install ros-melodic-cv-bridge -y

# Check if the reinstallation was successful
if [ $? -eq 0 ]; then
    echo "ros-melodic-cv-bridge successfully reinstalled."

    # Reinstall ros-melodic-aruco-detect package
    sudo apt-get --reinstall install ros-melodic-aruco-detect -y

    # Check if the reinstallation was successful
    if [ $? -eq 0 ]; then
        echo "ros-melodic-aruco-detect successfully reinstalled."
    else
        echo "Failed to reinstall ros-melodic-aruco-detect."
    fi
else
    echo "Failed to reinstall ros-melodic-cv-bridge."
fi

