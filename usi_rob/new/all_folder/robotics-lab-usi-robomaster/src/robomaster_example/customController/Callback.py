import rclpy
from rclpy.node import Node
from transforms3d._gohlketransforms import euler_from_quaternion

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

import sys
from math import sqrt, atan2, sin, cos, pi
from enum import Enum

from robomaster_example.model.TwoDPose import TwoDPose
from robomaster_example.model.RobotState import RobotState
from robomaster_example.model.RoboMasterModel import RoboMasterModel


class Callback():
    def __init__(self, model: RoboMasterModel):
        self.model = model
        self.node = model.node

    def odom_callback(self, msg):
        self.model.odom_pose = msg.pose.pose
        self.model.odom_velocity = msg.twist.twist
        
        pose2d = self.model.poseHelper.pose3d_to_2d(self.model.odom_pose)
        
        """
        self.node.get_logger().info(
            f"odometry: received pose x: {pose2d.x}, y: {pose2d.y}, theta: {pose2d.theta}",
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )
        """
        self.model.current_pose = pose2d
    
    
        
    def update_callback(self):
        self.model.stateController.update()

    def front_left_range_callback(self, msg):
        self.model.sensor.front_left_distance = msg.range

    def front_right_range_callback(self, msg):
        self.model.sensor.front_right_distance = msg.range

    def back_left_range_callback(self, msg):
        self.model.sensor.back_left_distance = msg.range

    def back_right_range_callback(self, msg):
        self.model.sensor.back_right_distance = msg.range
    

