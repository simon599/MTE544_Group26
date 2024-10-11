# Useful Resources:
# Tutorial for publisher and subscriber https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#write-the-subscriber-node
# About QOS https://discuss.px4.io/t/qos-profile-for-ros2-in-python/33043
# IMU https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html 
# Odometry https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html 
# Laser https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html 
# Moving the robot https://docs.ros2.org/galactic/api/geometry_msgs/msg/Twist.html 

# Imports
import rclpy # ROS2 Python API
from rclpy.node import Node # Contains subscription/publisher methods
from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile

# TODO Part 3: Import message types needed: 
    # For sending velocity commands to the robot: Twist
    # For the sensors: Imu, LaserScan, and Odometry
# Check the online documentation to fill in the lines below
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.time import Time

# Define constants for the different motion types.
CIRCLE=0; SPIRAL=1; ACC_LINE=2
# Define labels for motion types.
motion_types=['circle', 'spiral', 'line']

# Define the motion executioner class inheriting from Node.
class motion_executioner(Node):
    
    def __init__(self, motion_type=0):
        
        # Initialize the node with the name 'motion_types'
        super().__init__("motion_types")
        
        self.type=motion_type
        
        self.radius_=0.0
        
        # Flags for checking sensor intialization status, set to False.
        self.successful_init=False
        self.imu_initialized=False
        self.odom_initialized=False
        self.laser_initialized=False

        # TODO Part 3: Create the QoS profile by setting the proper parameters in (...)
        # Create a QoS profile with specified reliability, durability, history, and depth.
        qos=QoSProfile(reliability=2,
                       durability=2,
                       history=1,
                       depth=10)
            
        # TODO Part 3: Create a publisher to send velocity commands by setting the proper parameters in (...)
        # Create a publisher for velocity commands to the '/cmd_vel' topic using the QoS profile.
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', qos_profile=qos)
                
        # Establish loggers for each sensor to record data in CSV files.
        self.imu_logger=Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        # Note: updated Odometry data headers to record quaternion components (x, y, z, w) instead of procesing euler angles during logging.
        self.odom_logger=Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y","quat_x", "quat_y","quat_z", "quat_w","stamp"])
        # Note: updated LaserScan data headers to record start angle (in radians) of scan measurements.
        self.laser_logger=Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["start_angle", "ranges", "angle_increment", "stamp"])

        # TODO Part 5: Create below the subscription to the topics corresponding to the respective sensors

        # IMU subscription
        # Create a subscriber node for IMU: Sensor command name, imu is the name of the topic, imu_callback gets called as soon as it receives a message.
        self.create_subscription(Imu, '/imu', self.imu_callback, qos_profile=qos)
        
        # ENCODER subscription
        # Create a subscriber node for odom: Sensor command name, odom is the name of the topic, odom_callback gets called as soon as it receives a message.
        self.create_subscription(Odometry, '/odom', self.odom_callback, qos_profile=qos)
        
        # LaserScan subscription 
        # Create a subscriber node for laser: Sensor command name, scan is the name of the topic, laser_callback gets called as soon as it receives a message.
        self.create_subscription(LaserScan, '/scan', self.laser_callback, qos_profile=qos)
        
        # Establish timer to periodically execute velocity commands (every 0.1 seconds).
        self.create_timer(0.1, self.timer_callback)

    # TODO Part 5: Callback functions: complete the callback functions of the three sensors to log the proper data.
    # To also log the time you need to use the rclpy Time class, each ros msg will come with a header, and then
    # inside the header you have a stamp that has the time in seconds and nanoseconds, you should log it in nanoseconds as 
    # such: Time.from_msg(imu_msg.header.stamp).nanoseconds
    # You can save the needed fields into a list, and pass the list to the log_values function in utilities.py


    # IMU callback function, logs IMU data to a CSV file.
    def imu_callback(self, imu_msg: Imu):
        # Extract necessary data.
        acc_x = imu_msg.linear_acceleration.x
        acc_y = imu_msg.linear_acceleration.y
        angular_z = imu_msg.angular_velocity.z
        timestamp = Time.from_msg(imu_msg.header.stamp).nanoseconds
    
        # Log the values.
        self.imu_logger.log_values([acc_x, acc_y, angular_z, timestamp])
        
        # Mark IMU as initialized.
        self.imu_initialized = True


    # Odom callback function, logs Odometry data to a CSV file.
    def odom_callback(self, odom_msg: Odometry):
        # Extract necessary data.
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        quat = odom_msg.pose.pose.orientation
        timestamp = Time.from_msg(odom_msg.header.stamp).nanoseconds

        # Log the values.
        self.odom_logger.log_values([x, y, quat.x, quat.y, quat.z, quat.w, timestamp])
        
        # Mark Odometry as initialized.
        self.odom_initialized = True


    # Laser callback function, logs LaserScan data to a CSV file.
    def laser_callback(self, laser_msg: LaserScan):
        # Extract necessary data.
        start_angle = laser_msg.angle_min # starting angle of the scan [rad]
        ranges = list(laser_msg.ranges) # distance measurements [m]
        angle_increment = laser_msg.angle_increment # angle between measurements [rad]
        timestamp = Time.from_msg(laser_msg.header.stamp).nanoseconds
        
        # Log the values.
        self.laser_logger.log_values([start_angle, ranges, angle_increment, timestamp])
        
        # Mark LaserScan as initialized.
        self.laser_initialized = True


    # Timer callback function to publish velocity commands based on motion type.
    def timer_callback(self):
        
        # Check if all sensors are initialized.
        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            self.successful_init=True
        if not self.successful_init:
            return  # Exit if sensors not initialized.
        
        # Generate velocity command based on the selected motion type.
        cmd_vel_msg=Twist()
        if self.type==CIRCLE:
            cmd_vel_msg=self.make_circular_twist()
        elif self.type==SPIRAL:
            cmd_vel_msg=self.make_spiral_twist()         
        elif self.type==ACC_LINE:
            cmd_vel_msg=self.make_acc_line_twist()   
        else:
            print("Invalid motion type. Available options are: 0 - CIRCLE, 1 - SPIRAL, 2 - ACCELERATED LINE")
            raise SystemExit
        
        # Publish the velocity command
        self.vel_publisher.publish(cmd_vel_msg)  
    

    # TODO Part 4: Motion functions: complete the functions to generate the proper messages corresponding to the desired motions of the robot
    # Create the circular motion velocity command.
    def make_circular_twist(self):    
        msg=Twist()
        msg.linear.x = 0.5 # Move forward
        msg.angular.z = 2.0 # Rotate
        return msg


    # Create the spiral motion velocity command.
    def make_spiral_twist(self):
        msg=Twist()
        
        self.radius_ += 0.01
        msg.linear.x = self.radius_ # Increase linear velocity

        msg.angular.z = 2.0 # Set a constant rotational speed
        return msg
    
    # Create the accelerating straight line motion velocity command.
    def make_acc_line_twist(self):
        msg=Twist()
        
        self.radius_ += 0.01
        msg.linear.x = self.radius_ # Increase linear velocity
        
        msg.angular.z=0.0 # No rotation
        return msg

# Package for reading input commands.
import argparse

# Main method to handle command-line arguments and start the ROS2 node.
if __name__=="__main__":
    argParser=argparse.ArgumentParser(description="input the motion type")
    argParser.add_argument("--motion", type=str, default="circle")
    rclpy.init()
    args = argParser.parse_args()

    # Instantiate motion_executioner node based on the selected motion type.
    if args.motion.lower() == "circle":
        ME=motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        ME=motion_executioner(motion_type=ACC_LINE)
    elif args.motion.lower() =="spiral":
        ME=motion_executioner(motion_type=SPIRAL)
    else:
        print(f"we don't have {args.motion.lower()} motion type")
    
    # Spin the node (keep it alive) until interrupted in the terminal.
    try:
        rclpy.spin(ME)
    except KeyboardInterrupt:
        print("Exiting")
