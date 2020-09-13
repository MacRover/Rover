Launch ARM simulation with:
========================================================================

1. Launch the world with the arm model first:
                        
       roslaunch simulation test_world.launch model:=arm

2. If you want to control the arm, then you have to run the joint_state_controller under control package. This create the controllers for the joints, which will take in angle commands. The command for it is:

       roslaunch control arm_controller.launch

If it complains about missing effort_controller, you might need to install the proper ros controller packages.(ros-melodic-ros-controllers)

3. Finally to control the ARM, launch the python script under ROS_WS/src/control/src which takes in keyboard input to send commands to the joint_state_controller.

Launch Rover simulation with:
============================================================================
    roslaunch simulation test_world.launch


Launch Stereo simulation with:
=======================================
1. Launch the world with the rover model first

    roslaunch simulation test_world.launch

2. The stereo_outdoor.launch launches the rtabmap node for mapping, and the stereo_image_proc for image rectification. Various ros packages need to be installed (rtabmap, stereo_image_proc).

The command for it is:

    roslaunch simulation stereo_outdoor.launch

3. Then drive the rover around with:


    rosrun control drive_control.py

Going through the tutorial given on the gazebo website is helpful if you want to learn gazebo simulation in detail.

The models are to be defined using URDF. You will also include some extra elements(ie: Inertial) from Gazebo so that you can simulate it properly in Gazebo. Because plain xml for URDF is inconvenient, xacro macro is used.

Read more at: [Gazebo URDF](http://gazebosim.org/tutorials/?tut=ros_urdf)

You can used premade 3d models by importing them as meshes. There are lots of compatible types of 3d model files, but the most common would be STL. If you want proper texture, use blender to convert the stl to dae file types. You can also modify the origin of the models in blender such as moving the origin to the middle of the object. This is super useful because when you import meshes in the URDF, its not oriented properly, and you have to modify the origin position and orientation values. 

To attach different components(links) to each other, you use joints and these jonts can be fixed, or catered to specific movements.

Before you orient and position objects, just try to make sure you orient all components facing the X axis. This is a huge mistake I made where I thought orientation relative to axis didn't matter. The rover will work properly in simulations, but when you check the odometry data, you will see that the odometry direction are all wrong. This is because it is assumed the forward direction is oriented towards the positive X direction.

So to build models, you can try making them using Gazebo, but I highly recommend using RVIZ. Starting up Gazebo and running it takes a lot of processing power thus taking more time compared to RVIZ. RVIZ also allows you to measure distances, which you can use in you model to see where you position something.

Gazebo is connected to ROS using Gazebo plugins, there are components that will create topics and either subscribe and publish to them. For example the rover is using the Skid steering plugin, which allows you to move the wheels using cmd_vel messages, and it also publishes odometry data. The various plugins can be found on the Gazebo website.

Read more at: [Gazebo ROS plugins](http://gazebosim.org/tutorials?tut=ros_gzplugins&cat=connect_ros)
