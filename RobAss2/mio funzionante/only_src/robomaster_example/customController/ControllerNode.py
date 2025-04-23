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

from robomaster_example.model.TwoDPose import TwoDPose
from robomaster_example.model.RobotState import RobotState
from robomaster_example.model.RoboMasterModel import RoboMasterModel

from robomaster_example.customController.SubscriptionController import SubscriptionController




class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node_ex4')
        self.future = None
        self.model = RoboMasterModel(self) #Dependency injection
        self.subscribe_controller = SubscriptionController(self.model)
        
        
    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(1/60, self.subscribe_controller.callback.update_callback)
    
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.subscribe_controller.vel_publisher.publish(cmd_vel)
    
    


    

