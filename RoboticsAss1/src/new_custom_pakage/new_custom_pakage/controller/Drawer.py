

################################################################################
#                           CLASSE Drawer
################################################################################

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class Drawer(Node):
    """
    Il Drawer legge i segmenti della lettera (passata come parametro) e invia
    i comandi a Turtlesim per disegnarla.
    """
    def __init__(self, turtle_name: str, letter):
        super().__init__(f'drawer_{turtle_name.lower()}')
        self.turtle_name = turtle_name
        self.letter = letter

        # Publisher per i messaggi di movimento (Twist)
        self.vel_publisher = self.create_publisher(Twist, f'/{self.turtle_name}/cmd_vel', 10)
        # Subscriber per monitorare la posa della tartaruga (debug)
        self.pose_subscriber = self.create_subscription(Pose, f'/{self.turtle_name}/pose', self.pose_callback, 10)

        # Timer per inviare comandi ogni 0.1 s
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.segments = self.letter.segments
        self.current_segment_index = 0
        self.segment_tick_count = 0
        self.done = False

    def pose_callback(self, msg: Pose):
        self.get_logger().debug(f"[{self.turtle_name}] Posizione: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}")

    def timer_callback(self):
        if self.current_segment_index >= len(self.segments):
            self.get_logger().info(f"Lettera '{self.letter.name}' completata per {self.turtle_name}.")
            msg_stop = Twist()
            self.vel_publisher.publish(msg_stop)
            self.timer.cancel()
            self.done = True
            return

        segment = self.segments[self.current_segment_index]
        msg = Twist()
        msg.linear.x = segment["linear_speed"]
        msg.angular.z = segment["angular_speed"]
        self.vel_publisher.publish(msg)

        self.segment_tick_count += 1
        if self.segment_tick_count >= segment["duration"]:
            self.get_logger().info(f"[{self.turtle_name}] Segmento {self.current_segment_index + 1} completato.")
            self.current_segment_index += 1
            self.segment_tick_count = 0
