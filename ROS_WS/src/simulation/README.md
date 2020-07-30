So these are my experience and thoughts on how simulation with gazebo and ROS works. 

I highly recommend going through the tutorial given on the gazebo website. Don't skip parts cause all the minor details matter.

You can used premade 3d models by importing them as meshes. There are lots of compatible types of 3d model files, but the most common would be STL.

To attach different components(links) to each other, you use joints and these jonts can be fixed, or catered to specific movements.

Before you orient and position objects, just try to make sure you orient all components facing the X axis. This is a huge mistake I made where I thought orientation relative to axis didn't matter. The rover will work properly in simulations, but when you check the odometry data, you will see that the odometry direction are all wrong. This is because it is assumed the forward direction is oriented towards the positive X direction.

So to build models, you can try making them using Gazebo, but I highly recommend using RVIZ. Starting up Gazebo and running it takes a lot of 
processing power thus taking more time compared to RVIZ. RVIZ also allows you to measure distances, which you can use in you model to see where you position something.

Gazebo is connected to ROS using Gazebo plugins, there are components that will create topics and either subscribe and publish to them. For example the rover is using the Skid steering plugin, which allows you to move the wheels using cmd_vel messages, and it also publishes odometry data. The various plugins can be found on the Gazebo website.

I have decided to keep the models of the rover for the test one, and just added new cylinders as the wheels.