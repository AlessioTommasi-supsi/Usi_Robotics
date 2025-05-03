import rclpy
from rclpy.node import Node
from transforms3d._gohlketransforms import euler_from_quaternion

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry

from robomaster_msgs.msg import GimbalCommand

from cv_bridge import CvBridge
import cv2

import sys
from math import sqrt, atan2, sin, cos


class TwoDPose():
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

class GimbalPose():
    def __init__(self, yawn, pitch):
        self.yawn = yawn
        self.pitch = pitch


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller')
        
        self.current_pose = None
        self.current_gimbal_pose = None
                
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.gumbal_publisher = self.create_publisher(GimbalCommand, 'cmd_gimbal', 10)

        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        self.joint_states_subscriber = self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)

        self.image_subscriber = self.create_subscription(Image, 'camera/image_color', self.image_callback, 10)


    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(1/60, self.update_callback)
    
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)

    def pose3d_to_2d(self, pose3):
        quaternion = (
            pose3.orientation.w,
            pose3.orientation.x,
            pose3.orientation.y,
            pose3.orientation.z
        )
        
        _, _, yaw = euler_from_quaternion(quaternion)
        
        pose2 = (
            pose3.position.x,  # x position
            pose3.position.y,  # y position
            yaw                # theta orientation
        )
        
        return TwoDPose(pose2[0], pose2[1], pose2[2])

    def odom_callback(self, msg):
        pose2d = self.pose3d_to_2d(msg.pose.pose)
        
        self.get_logger().info(
            f"odometry: received pose x: {pose2d.x}, y: {pose2d.y}, theta: {pose2d.theta}",
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )
        self.current_pose = pose2d
    
    def joint_states_callback(self, msg):
        yawn_index = -1
        pitch_index = -1
        
        for index, name in enumerate(msg.name):
            if "gimbal_joint" in name:
                yawn_index = index
            elif "blaster_joint" in name:
                pitch_index = index
        
        self.current_gimbal_pose = GimbalPose(msg.position[yawn_index], msg.position[pitch_index])

        self.get_logger().info(
            f"gimbal: received pose yawn: {self.current_gimbal_pose.yawn}, pitch: {self.current_gimbal_pose.pitch}, ",
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )
    
    def image_callback(self, msg):
        bridge = CvBridge()

        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("ROS Image", cv_image)
        cv2.waitKey(1)  # Needed to update the display window
        
    def update_callback(self):
        gimbal_cmd = GimbalCommand() 
        gimbal_cmd.yaw_speed = 0.0
        gimbal_cmd.pitch_speed = -0.05

        self.gumbal_publisher.publish(gimbal_cmd)
  


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


def main():
    # Initialize the ROS client library
    rclpy.init(args=sys.argv)
    
    # Create an instance of your node class
    node = ControllerNode()
    node.start()
   
    # Create a future that we will complete once the last waypoint is reached
    future = rclpy.task.Future()
    node.future = future

    # Start the node
    node.start()

    try:
        rclpy.spin_until_future_complete(node, future)
    except KeyboardInterrupt:
        pass
    
    # Ensure the RoboMaster is stopped before exiting
    node.stop()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
