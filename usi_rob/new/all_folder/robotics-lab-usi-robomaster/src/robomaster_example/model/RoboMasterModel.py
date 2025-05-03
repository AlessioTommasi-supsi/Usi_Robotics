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
from robomaster_example.model.RobotState import RobotStateManager
from robomaster_example.customController.SensorHelper import SensorHelper

class RoboMasterModel():
    def __init__(self, node: Node):
        self.stateController = RobotStateManager(self)
        self.node = node

        # attributi legati a esercizio 1
        self.current_pose = None
        self.current_velocity = None
        
        self.goal_tolerance = 0.1
        

        self.poseHelper = TwoDPose(0, 0, 0)
        self.waypoints = self.get_waypoints()
        self.waypoint_index = 0
        self.goal_pose = self.waypoints[self.waypoint_index]


        # attributes for odometry 
        self.odom_pose = None
        self.odom_velocity = None

        self.subscription_controller = None

        #Attributi legati ai sensori
        self.sensor = SensorHelper(self)

    def get_waypoints(self): #in teoria questi devono essere aggiunti a random!
        waypoints = []
        waypoints.append(TwoDPose(0,0,0))
        waypoints.append(TwoDPose(1,1,0))
        
        return waypoints


