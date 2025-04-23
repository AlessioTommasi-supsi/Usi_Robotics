import rclpy
from rclpy.node import Node
from transforms3d._gohlketransforms import euler_from_quaternion

from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

import sys
from math import sqrt, atan2, sin, cos, pi

class TwoDPose():
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node_ex1')
        
        self.current_pose = None

        self.goal_tolerance = 0.1

        self.waypoints = self.get_waypoints()
        self.waypoint_index = 0
        self.goal_pose = self.waypoints[self.waypoint_index]

        # Create attributes to store odometry pose and velocity
        self.odom_pose = None
        self.odom_velocity = None
                
        # Create a publisher for the topic 'cmd_vel'
        self.vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Create a subscriber to the topic 'odom', which will call 
        # self.odom_callback every time a message is received
        self.odom_subscriber = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # NOTE: we're using relative names to specify the topics (i.e., without a 
        # leading /). ROS resolves relative names by concatenating them with the 
        # namespace in which this node has been started, thus allowing us to 
        # specify which RoboMaster should be controlled.

    def get_waypoints(self):
        waypoints = []
        waypoints.append(TwoDPose(0,1.8,0))
        waypoints.append(TwoDPose(-1,1,0))
        waypoints.append(TwoDPose(0,0,0))
        waypoints.append(TwoDPose(1,-1,0))
        waypoints.append(TwoDPose(0,-1.8,0))
        waypoints.append(TwoDPose(-1,-1,0))
        waypoints.append(TwoDPose(0,0,0))
        waypoints.append(TwoDPose(1,1,0))
        waypoints.append(TwoDPose(0,1.8,0))

        return waypoints

    def start(self):
        # Create and immediately start a timer that will regularly publish commands
        self.timer = self.create_timer(1/60, self.update_callback)
    
    def stop(self):
        # Set all velocities to zero
        cmd_vel = Twist()
        self.vel_publisher.publish(cmd_vel)
    
    def odom_callback(self, msg):
        self.odom_pose = msg.pose.pose
        self.odom_valocity = msg.twist.twist
        
        pose2d = self.pose3d_to_2d(self.odom_pose)
        
        self.get_logger().info(
            f"odometry: received pose x: {pose2d.x}, y: {pose2d.y}, theta: {pose2d.theta}",
             throttle_duration_sec=0.5 # Throttle logging frequency to max 2Hz
        )
        self.current_pose = pose2d
    
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
        
    def update_callback(self):
        if self.goal_pose is None or self.current_pose is None:
            # No goal right now, do nothing
            return

        if self.euclidean_distance(self.goal_pose, self.current_pose) >= self.goal_tolerance:
            # We still haven't reached the goal pose. Use a proportional 
            # controller to compute velocities that will move the turtle 
            # towards the goal.
            cmd_vel = Twist() 
            cmd_vel.linear.x = 0.25  # [m/s]
            cmd_vel.angular.z = self.angular_vel(self.goal_pose, self.current_pose)

            self.vel_publisher.publish(cmd_vel)
        else:
            # If the goal pose is reached and we are not at the last waypoint
            if self.waypoint_index < len(self.waypoints) - 1:
                self.get_logger().info(f"vel_controller: reached waypoint {self.waypoint_index}")
                self.waypoint_index += 1
                self.goal_pose = self.waypoints[self.waypoint_index]
            else:
                # Stop after reaching the last waypoint
                self.get_logger().info(f"Reached last waypoint with index{self.waypoint_index}, stopping.")
                self.stop()

                if self.future:
                    self.future.set_result(True)  # Mark the future as complete


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


if __name__ == '__main__':
    main()
