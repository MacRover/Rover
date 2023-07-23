Launch Rover Bringup:
========================================================================
The purpose of bringup.launch is to allow the rover to launch robot state publisher and all the sensor drivers needed by autonomy tasks like navigation.

    roslaunch bringup bringup.launch
    
The launch file has the following parameters:
| Parameter | Description                               | Default Value |
| --------- | ----------------------------------------- | ------------- |
| `rviz`    | Whether to enable RViz visualization      | `false`       |
| `imu`     | Whether to enable IMU sensor driver       | `true`        |
| `gps`     | Whether to enable GPS sensor driver       | `true`        |



Launching Realsense Cameras Independently:
============================================================================
If needed, the t265 and d435 can be launched independently with the following:
    
    roslaunch bringup t265.launch
    roslaunch bringup d435.launch


