import rclpy
from rclpy.node import Node
from transforms3d._gohlketransforms import euler_from_quaternion

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range

import sys
from math import sqrt, atan2, sin, cos, pi
from enum import Enum
from robomaster_example.customController.Callback import Callback
from robomaster_example.model.RoboMasterModel import RoboMasterModel

class SubscriptionController():
    def __init__(self, model: RoboMasterModel):
        self.node = model.node
        
        self.callback = Callback(model)
        self.model = model
        self.model.subscription_controller = self

        self.front_left_range_subscriber = None
        self.front_right_range_subscriber = None
        self.back_left_range_subscriber = None
        self.back_right_range_subscriber = None


        self.subscribe(self.node)

    
    def subscribe(self, node: Node):
        
        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber to the topic 'odom', which will call
        self.odom_subscriber = self.node.create_subscription(Odometry, 'odom', self.callback.odom_callback, 10)

        # Sottoscrizioni a sensori
        self.front_left_range_subscriber = self.node.create_subscription(Range, 'range_3', self.callback.front_left_range_callback, 10)
        self.front_right_range_subscriber = self.node.create_subscription(Range, 'range_1', self.callback.front_right_range_callback, 10)
        self.back_left_range_subscriber = self.node.create_subscription(Range, 'range_2', self.callback.back_left_range_callback, 10)
        self.back_right_range_subscriber = self.node.create_subscription(Range, 'range_0', self.callback.back_right_range_callback, 10)

        
        