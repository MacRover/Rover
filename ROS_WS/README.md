This is a ros workspace, the first thing you want to do is set up ros environments variables to point to this workspace so the ros commands work use it.

The command for it is:
====================================================

    source ROS_WS/devel/setup.bash
            
You can add this command to your bashrc if you don't want to do it everytime you open terminal.


Ros packages that have to be installed:

- move_base
- rtabmap_ros
- stereo_image_proc
- robot_state_publisher
- ros_control
- ros_controllers
