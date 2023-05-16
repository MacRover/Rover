#!/bin/bash

file1_path="/opt/ros/melodic/share/cv_bridge/cmake/cv_bridgeConfig.cmake"
file2_path="/opt/ros/melodic/share/aruco_detect/cmake/aruco_detectConfig.cmake"
search_line='set(_include_dirs "include;/usr/include;/usr/include/opencv")'
replace_line='set(_include_dirs "include;/usr/include;/usr/include/opencv4")'

# Function to replace line in a file
replace_line_in_file() {
    local file_path=$1
    local search=$2
    local replace=$3

    # Check if the file exists
    if [ -f "$file_path" ]; then
        # Replace the line in the file
        sed -i "s|$search|$replace|" "$file_path"
        echo "Line replaced in $file_path"
    else
        echo "File not found: $file_path"
    fi
}

# Install packages
sudo apt-get update
sudo apt-get install ros-melodic-cv-bridge -y
sudo apt-get install ros-melodic-aruco-detect -y

# Check if the installation was successful
if [ $? -eq 0 ]; then
    # Replace line in cv_bridgeConfig.cmake
    replace_line_in_file "$file1_path" "$search_line" "$replace_line"

    # Replace line in aruco_detectConfig.cmake
    replace_line_in_file "$file2_path" "$search_line" "$replace_line"
else
    echo "Package installation failed."
fi

echo "Aruco setup complete!"
