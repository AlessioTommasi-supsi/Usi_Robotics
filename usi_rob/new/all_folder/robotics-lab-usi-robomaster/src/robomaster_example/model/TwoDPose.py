import sys
import rclpy
from rclpy.node import Node
from transforms3d._gohlketransforms import euler_from_quaternion

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

import sys
from math import sqrt, atan2, sin, cos, pi
from enum import Enum


class TwoDPose():
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta
    

    def pose3d_to_2d(self, pose3):
        quaternion = (
            pose3.orientation.w,
            pose3.orientation.x,
            pose3.orientation.y,
            pose3.orientation.z
        )
        
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        
        pose2 = (
            pose3.position.x,  # x position
            pose3.position.y,  # y position
            yaw                # theta orientation
        )
        
        # Convert from ROS2 coordinate system (x up, y left) to Coppelia coordinate system (x right, y up)
        return TwoDPose(pose2[0], pose2[1], pose2[2])
    
    def euclidean_distance(self, goal_pose, current_pose):
        """Euclidean distance between current pose and the goal."""
        return sqrt((goal_pose.x - current_pose.x) ** 2 +
                    (goal_pose.y - current_pose.y) ** 2)

    def angular_difference(self, goal_theta, current_theta):
        """Compute shortest rotation from orientation current_theta to orientation goal_theta"""
        return atan2(sin(goal_theta - current_theta), cos(goal_theta - current_theta))

    def linear_vel(self, goal_pose, current_pose, constant=1.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose, current_pose)

    def steering_angle(self, goal_pose, current_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - current_pose.y, goal_pose.x - current_pose.x)

    def angular_vel(self, goal_pose, current_pose, constant=0.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        goal_theta = self.steering_angle(goal_pose, current_pose)
        return constant * self.angular_difference(goal_theta, current_pose.theta)
