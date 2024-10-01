#Simon's notes: tutorial for publisher and subscriber https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html#write-the-subscriber-node

# Imports
import rclpy

from rclpy.node import Node

from utilities import Logger, euler_from_quaternion
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# TODO Part 3: Import message types needed: 
    # For sending velocity commands to the robot: Twist
    # For the sensors: Imu, LaserScan, and Odometry
# Check the online documentation to fill in the lines below
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

from rclpy.time import Time

# You may add any other imports you may need/want to use below
# import ...

CIRCLE=0; SPIRAL=1; ACC_LINE=2
motion_types=['circle', 'spiral', 'line']

class motion_executioner(Node):
    
    def __init__(self, motion_type=0):
        
        super().__init__("motion_types")
        
        self.type=motion_type
        
        self.radius_=0.0
        
        self.successful_init=False
        self.imu_initialized=False
        self.odom_initialized=False
        self.laser_initialized=False
        
        # TODO Part 3: Create a publisher to send velocity commands by setting the proper parameters in (...)
        #Simon's notes: Twist means velocity, "velocity" is the name of the topic, queue size=10
        self.vel_publisher = self.create_publisher(Twist, '/velocity', 10)
                
        # loggers
        self.imu_logger=Logger('imu_content_'+str(motion_types[motion_type])+'.csv', headers=["acc_x", "acc_y", "angular_z", "stamp"])
        self.odom_logger=Logger('odom_content_'+str(motion_types[motion_type])+'.csv', headers=["x","y","th", "stamp"])
        self.laser_logger=Logger('laser_content_'+str(motion_types[motion_type])+'.csv', headers=["ranges", "angle_increment", "stamp"])
        
        # TODO Part 3: Create the QoS profile by setting the proper parameters in (...)
        #Simon's notes: not sure if this is correct or all necessary, since I think only the depth parameter is mandatory
        #https://discuss.px4.io/t/qos-profile-for-ros2-in-python/33043
        qos=QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10)

        # TODO Part 5: Create below the subscription to the topics corresponding to the respective sensors
        # IMU subscription
        #Simon's notes: Sensor command name, imu/data is the name of the topic, imu_callback gets called as soon as it receives a message, queue size=10
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        
        # ENCODER subscription
        #Simon's notes: Sensor command name, odom is the name of the topic, odom_callback gets called as soon as it receives a message, queue size=10
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # LaserScan subscription 
        #Simon's notes: Sensor command name, scan is the name of the topic, laser_callback gets called as soon as it receives a message, queue size=10
        self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        
        self.create_timer(0.1, self.timer_callback)


    # TODO Part 5: Callback functions: complete the callback functions of the three sensors to log the proper data.
    # To also log the time you need to use the rclpy Time class, each ros msg will come with a header, and then
    # inside the header you have a stamp that has the time in seconds and nanoseconds, you should log it in nanoseconds as 
    # such: Time.from_msg(imu_msg.header.stamp).nanoseconds
    # You can save the needed fields into a list, and pass the list to the log_values function in utilities.py

    # Simon's notes: https://docs.ros2.org/foxy/api/sensor_msgs/msg/Imu.html
    def imu_callback(self, imu_msg: Imu):
        # Extract necessary data, not sure which ones we need but here's a few main ones
        acc_x = imu_msg.linear_acceleration.x
        acc_y = imu_msg.linear_acceleration.y
        angular_z = imu_msg.angular_velocity.z
        timestamp = Time.from_msg(imu_msg.header.stamp).nanoseconds
    
        # Log the values
        self.imu_logger.log_values([acc_x, acc_y, angular_z, timestamp])
        
        # Mark as initialized, not sure if needed, but there is an imu_initialized = False up top so might be useful
        self.imu_initialized = True
        
    # Simon's notes: https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html 
    def odom_callback(self, odom_msg: Odometry):
        # Extract necessary data, not sure which ones we need but here's a few main ones
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        roll, pitch, yaw = euler_from_quaternion(odom_msg.pose.pose.orientation) #not sure which one we are supposed to be implementing in utilities.py, modify as needed
        timestamp = Time.from_msg(odom_msg.header.stamp).nanoseconds

        # Log the values
        self.odom_logger.log_values([x, y, yaw, timestamp])
        
        # Mark as initialized, not sure if needed, but there is an imu_initialized = False up top so might be useful
        self.odom_initialized = True

    # Simon's notes: https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html 
    def laser_callback(self, laser_msg: LaserScan):
        # Extract necessary data, not sure which ones we need but here's a few main ones
        ranges = list(laser_msg.ranges)
        angle_increment = laser_msg.angle_increment
        timestamp = Time.from_msg(laser_msg.header.stamp).nanoseconds
        
        # Log the values
        self.laser_logger.log_values([ranges, angle_increment, timestamp])
        
        # Mark as initialized, not sure if needed, but there is an imu_initialized = False up top so might be useful
        self.laser_initialized = True
                
    def timer_callback(self):
        
        if self.odom_initialized and self.laser_initialized and self.imu_initialized:
            self.successful_init=True
            
        if not self.successful_init:
            return
        
        cmd_vel_msg=Twist()
        
        if self.type==CIRCLE:
            cmd_vel_msg=self.make_circular_twist()
        
        elif self.type==SPIRAL:
            cmd_vel_msg=self.make_spiral_twist()
                        
        elif self.type==ACC_LINE:
            cmd_vel_msg=self.make_acc_line_twist()
            
        else:
            print("type not set successfully, 0: CIRCLE 1: SPIRAL and 2: ACCELERATED LINE")
            raise SystemExit 

        self.vel_publisher.publish(cmd_vel_msg)
        
    
    # TODO Part 4: Motion functions: complete the functions to generate the proper messages corresponding to the desired motions of the robot
    #Simon's notes: https://docs.ros2.org/galactic/api/geometry_msgs/msg/Twist.html 
    def make_circular_twist(self):
        
        msg=Twist()
        msg.linear.x = 0.5 #move forward
        msg.angular.z = 0.5  #rotate
        return msg

    def make_spiral_twist(self):
        msg=Twist()
        
        self.radius_+=0.01
        msg.linear.x = self.radius_ #increasing linear velocity

        msg.angular.z=0.5 #constant rotational speed
        return msg
    
    def make_acc_line_twist(self):
        msg=Twist()
        
        self.radius_+=0.01
        msg.linear.x = self.radius_ #increasing linear velocity
        
        msg.angular.z=0.0 #no rotation
        return msg

import argparse

if __name__=="__main__":
    

    argParser=argparse.ArgumentParser(description="input the motion type")


    argParser.add_argument("--motion", type=str, default="circle")



    rclpy.init()

    args = argParser.parse_args()

    if args.motion.lower() == "circle":

        ME=motion_executioner(motion_type=CIRCLE)
    elif args.motion.lower() == "line":
        ME=motion_executioner(motion_type=ACC_LINE)

    elif args.motion.lower() =="spiral":
        ME=motion_executioner(motion_type=SPIRAL)

    else:
        print(f"we don't have {arg.motion.lower()} motion type")


    
    try:
        rclpy.spin(ME)
    except KeyboardInterrupt:
        print("Exiting")
