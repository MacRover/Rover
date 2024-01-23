import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ruamel.yaml import YAML
import math
import time

#? GOALS
#* test rover outside
#* verify all aspects of telemetry and odometry are working as expected
#* determine state of encoders and decide if we need to reupload firmware
#* implement and test a complete heading calibration procedure 

# TODO:
#* Send drive command for certain time (e.g. .5m/s for 10s)
#* calculate heading error
#* write to file
#* drive back roughly same amount
#* $$?

#* NOTES
#* do a general inspection of the rover prior to driving
#* confirm state of batteries regularly
#* remember to turn on cmd_vel_repeater
#* verify all related sensors are acting as expected before testing
#* verify gps error is <10m
#* use mapviz and/or rviz to help with visualizing the issue
#* use the back T265
#* install ruamel [pip install ruamel.yaml]

#* What do we need to bridge between ROS1 and 2?
#* cmd_vel to send drive commands
#* odometry/global to recieve location 

#* HEADING CALIBRATION PROCEDURE
#* 1. bringup rover
#* 2. start navstack
#* 3. start calibrate_heading procedure
#* 4. stop navstack
#* 5. start navstack



# roscore
# rover_prestart
# rosrun control cmd_vel_repeater
# rosrun control xbox_control
# roslaunch bringup bringup.launch
# roslaunch navigation navigation.launch


class HeadingCalculatorNode:
    def __init__(self):
        self.node = rclpy.create_node('heading_calculator_node')

        self.subscription_odom = self.node.create_subscription(
            Odometry,
            '/odometry/filtered_map',
            self.odom_callback,
            10  # QoS profile
        )

        self.publisher_cmd_vel = self.node.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Initialize ruamel.yaml
        self.yaml = YAML()

        # Path to the YAML file
        self.yaml_file_path = '/home/mmrt/Rover/ROS_WS/src/localization/params/ekf_t265_gps.yaml'

        self.taken_headingoff = False
        self.timer = None
        self.timer_callback_count = 0
        self.x_pos = 0
        self.y_pos = 0
        self.error1=1.2

        
    def odom_callback(self, msg):
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
        self.error1 = math.atan2(self.y_pos, self.x_pos)
        print(5)
#        print(math.atan2(self.y_pos, self.x_pos))
    
    def calibrate_heading(self):
       # error = math.atan2(self.y_pos, self.x_pos)
        print(self.error1)
        print(f'Rover heading error: {self.error1} radians')
        self.write_to_yaml(self.error1)

    def send_cmd_vel(self, linear_x, duration, rate):
        rate = self.node.create_rate(rate)  # Hz
        start_time = time.time()
        print(f'start_time {start_time} sec')

        while True:
           
            print(time.time())
            twist_msg = Twist()
            twist_msg.linear.x = linear_x
            self.publisher_cmd_vel.publish(twist_msg)
            rate.sleep()
            rclpy.spin_once(self)
            

    def write_to_yaml(self, heading_error):
        try:
            with open(self.yaml_file_path, 'r') as file:
                data = self.yaml.load(file)
                
                # Update the magnetic_declination_radians field
                data['navsat_transform']['magnetic_declination_radians'] = heading_error

            with open(self.yaml_file_path, 'w') as file:
                self.yaml.dump(data, file)
                print(f'Updated magnetic_declination_radians: {heading_error} in {self.yaml_file_path}')

        except Exception as e:
            print(f'Error updating YAML file: {e}')


def main():
    rclpy.init()
    node = HeadingCalculatorNode()

    # Drive forward for 10 seconds at 0.5m/s
    #node.send_cmd_vel(linear_x=0.5, duration=10.0, rate=10)

    node.send_cmd_vel(linear_x=0.0, duration=400.0, rate=10)

    
    node.calibrate_heading()

    #node.send_cmd_vel(linear_x=-0.5, duration=10.0, rate=10)

    rclpy.spin(node.node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()