import sys

from utilities import Logger

from rclpy.time import Time

from utilities import euler_from_quaternion, calculate_angular_error, calculate_linear_error
from rclpy.node import Node
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry as odom

from sensor_msgs.msg import Imu
from kalman_filter import kalman_filter

from rclpy import init, spin, spin_once

import numpy as np
import message_filters

rawSensors=0
kalmanFilter=1
odom_qos=QoSProfile(reliability=2, durability=2, history=1, depth=10)

class localization(Node):
    
    def __init__(self, type, dt, loggerName="robotPose.csv", loggerHeaders=["imu_ax", "imu_ay", "kf_ax", "kf_ay","kf_vx","kf_w","kf_x", "kf_y","stamp"]):

        super().__init__("localizer")

        self.loc_logger=Logger( loggerName , loggerHeaders)
        self.pose=None
        
        if type==rawSensors:
            self.initRawSensors()
        elif type==kalmanFilter:
            self.initKalmanfilter(dt)
        else:
            print("We don't have this type for localization", sys.stderr)
            return  

    def initRawSensors(self):
        self.create_subscription(odom, "/odom", self.odom_callback, qos_profile=odom_qos)
        
    def initKalmanfilter(self, dt):
        
        # TODO Part 3: Set up the quantities for the EKF (hint: you will need the functions for the states and measurements)
        
        x= np.zeros(6) # initial states, all starting from 0
        
        q_val = 0.5 # we tune this value in the lab
        Q = q_val * np.eye(6)

        r_val = 0.5 # also tune in-lab
        R = r_val * np.eye(4)
        
        P = np.eye(6) # initial covariance (covariance of estimation)
        
        self.kf=kalman_filter(P,Q,R, x, dt)
        
        # TODO Part 3: Use the odometry and IMU data for the EKF
        # message_filters.Subscriber(node, msg_type, topic_name, qos_profile=...), qos_profile is optional
        self.odom_sub=message_filters.Subscriber(self, odom, "/odom") # the node should be 'self'
        self.imu_sub=message_filters.Subscriber(self, Imu, "/imu")
        
        time_syncher=message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.imu_sub], queue_size=10, slop=0.1)
        time_syncher.registerCallback(self.fusion_callback)
    
    def fusion_callback(self, odom_msg: odom, imu_msg: Imu):
        
        # TODO Part 3: Use the EKF to perform state estimation
        # Take the measurements
        # your measurements are the linear velocity and angular velocity from odom msg
        # and linear acceleration in x and y from the imu msg
        # the kalman filter should do a proper integration to provide x,y and filter ax,ay
        v = odom_msg.twist.twist.linear.x
        w = odom_msg.twist.twist.angular.z
        ax = imu_msg.linear_acceleration.x
        ay = imu_msg.linear_acceleration.y 

        z=np.array([v, w, ax, ay])
        
        # Implement the two steps for estimation 
        # predict function from kalman_filter.py
        # estimate function from kalman_filter.py
        self.kf.predict()
        self.kf.update(z)

        # Get the estimate
        xhat=self.kf.get_states()

        # Update the pose estimate to be returned by getPose
        # extract valyes from xhat
        self.pose=np.array([xhat[0], xhat[1], xhat[2], odom_msg.header.stamp])

        # TODO Part 4: log your data
        # just add loggerHeaders from tut 6, use ay equation
        # loggerHeaders=["imu_ax", "imu_ay", "kf_ax", "kf_ay","kf_vx","kf_w","kf_x", "kf_y","stamp"]
        self.loc_logger.log_values([
            ax, 
            ay,
            xhat[5], 
            xhat[3] * xhat[4],
            xhat[4],
            xhat[3],
            xhat[0],
            xhat[1],
            odom_msg.header.stamp
        ])
      
    def odom_callback(self, pose_msg):
        
        self.pose=[ pose_msg.pose.pose.position.x,
                    pose_msg.pose.pose.position.y,
                    euler_from_quaternion(pose_msg.pose.pose.orientation),
                    pose_msg.header.stamp]

    # Return the estimated pose
    def getPose(self):
        return self.pose


if __name__=="__main__":
    
    init()
    
    LOCALIZER=localization()
    
    spin(LOCALIZER)
