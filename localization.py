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

        elf.loc_logger=Logger( loggerName , loggerHeaders)
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
        
        # Initialize the state vector x = [x, y, th, w, v, vdot]
        x = np.zeros(6)

        # Process noise covariance Q (6x6)
        q_value = 0.5  # You can adjust this value for tuning
        Q = q_value * np.eye(6)

        # Measurement noise covariance R (4x4)
        r_value = 0.5  # You can adjust this value for tuning
        R = r_value * np.eye(4)

        # Initial estimate covariance P (6x6)
        P = np.eye(6)  # Starting with an identity matrix
        
        self.kf=kalman_filter(P,Q,R, x, dt)
        
        # Use the odometry and IMU data for the EKF
        self.odom_sub = message_filters.Subscriber(self, odom, "/odom")
        self.imu_sub = message_filters.Subscriber(self, Imu, "/imu")
        
        time_syncher=message_filters.ApproximateTimeSynchronizer([self.odom_sub, self.imu_sub], queue_size=10, slop=0.1)
        time_syncher.registerCallback(self.fusion_callback)
    
    def fusion_callback(self, odom_msg: odom, imu_msg: Imu):
        
        # Take the measurements from odometry and IMU
        v_meas = odom_msg.twist.twist.linear.x          # Linear velocity from odometry
        w_meas = odom_msg.twist.twist.angular.z         # Angular velocity from odometry
        ax_meas = imu_msg.linear_acceleration.x         # Linear acceleration x from IMU
        ay_meas = imu_msg.linear_acceleration.y         # Linear acceleration y from IMU

        # Construct the measurement vector z
        z = np.array([v_meas, w_meas, ax_meas, ay_meas])

        # Perform the prediction and update steps of the EKF
        self.kf.predict()
        self.kf.update(z)
        
        # Get the estimate
        xhat=self.kf.get_states()

        # Update the pose estimate to be returned by getPose
        self.pose = np.array([xhat[0], xhat[1], xhat[2], odom_msg.header.stamp])

        # Log your data
        self.loc_logger.log_values([
            ax_meas,          # imu_ax
            ay_meas,          # imu_ay
            xhat[5],          # kf_ax (estimated vdot)
            xhat[4] * xhat[3],# kf_ay (estimated v * w)
            xhat[4],          # kf_vx (estimated linear velocity)
            xhat[3],          # kf_w (estimated angular velocity)
            xhat[0],          # kf_x (estimated x position)
            xhat[1],          # kf_y (estimated y position)
            odom_msg.header.stamp  # Timestamp
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
