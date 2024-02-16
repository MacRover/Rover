import rclpy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time  # Import the time module
from nav_msgs.msg import Odometry
from ruamel.yaml import YAML
import threading #needed to run drive fxn once 
import tf2_ros
from tf_transformations import euler_from_quaternion
#need sudo apt install ros-iron-tf-transformations


#for possible mathematic reference in code ref: https://www.movable-type.co.uk/scripts/latlong.html 
#https://www.igismap.com/formula-to-find-bearing-or-heading-angle-between-two-points-latitude-longitude/

class HeadingCalculatorNode:
    def __init__(self):
        self.node = rclpy.create_node('heading_calculator_node')
        
        self.subscription_odom = self.node.create_subscription(
            Odometry,
            '/odometry/global',
            self.odom_callback,
            10  # QoS profile
        )

        # local odom subscription for quanternion transforms for post-start heading updates
        # self.subscription_odom = self.node.create_subscription(
        #     Odometry,
        #     '/odometry/local',
        #     self.odom_local_callback,
        #     10  # QoS profile
        # )

        self.publisher_cmd_vel = self.node.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )


        # Initialize ruamel.yaml
        self.yaml = YAML()

        # Initialize x and y position
        self.x_pos = 0
        self.y_pos = 0

        #start x_pose and y_pose for multiple runs 
        rclpy.spin_once(self.node)
        self.x_init = self.x_pos
        self.y_init = self.y_pos


        # Start a separate thread for driving forward
        self.drive_thread = threading.Thread(target=self.send_cmd_vel(0.5, 10.0, 10))
        self.drive_thread.start()

        # Path to the YAML file
        self.yaml_file_path = '/home/mmrt/Rover/ROS_WS/src/localization/params/ekf_t265_gps.yaml'

        self.taken_headingoff = False


    def odom_callback(self, msg):
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y

    # def odom_local_callback(self, msg):
    #     orientation_list = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]

    #     (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    #     self.pose_yaw = yaw
    
    
    def calibrate_heading(self):
        rclpy.spin_once(self.node)
        error = math.atan2(self.y_pos - self.y_init, self.x_pos - self.x_init)
        print(f'Rover heading error: {error*180/math.pi} deg from y_pos of {self.y_pos } and x_pos of {self.x_pos }')
        self.write_to_yaml(error)

    def send_cmd_vel(self, linear_x, duration, rate):
        start_time = time.time()
        print(f'start_time {start_time} sec')

        while time.time() - start_time < duration and rclpy.ok():
            twist_msg = Twist()
            twist_msg.linear.x = linear_x
            self.publisher_cmd_vel.publish(twist_msg)
            time.sleep(0.1) #slow down the loop

        
        #NEED TO STOP THE ROVER
        print("before rover stop")
        twist_msg = Twist()
        twist_msg.linear.x = 0.0
        self.publisher_cmd_vel.publish(twist_msg)
        print("after rover stop")
            

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

    # Drive forward for x seconds at 0.5m/s
    # node.send_cmd_vel(linear_x=0.5, duration=7.0+time.time(), rate=10)
    # node.sleep(8)
    # node.send_cmd_vel(linear_x=0.0, duration=400.0, rate=10)

    
    node.calibrate_heading()

    #node.send_cmd_vel(linear_x=-0.5, duration=10.0, rate=10)

    rclpy.spin(node.node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
