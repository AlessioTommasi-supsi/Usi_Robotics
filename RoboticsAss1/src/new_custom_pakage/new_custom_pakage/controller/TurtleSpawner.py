import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from std_msgs.msg import String
from rclpy.qos import QoSProfile, DurabilityPolicy
from new_custom_pakage.controller.KeyboardController import KeyboardController

import sys
import select
import json


class TurtleSpawner(Node):
    def __init__(self):
        super().__init__('turtle_spawner')
        
        # Client per il servizio spawn e kill di Turtlesim
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client  = self.create_client(Kill, 'kill')

        # Inizializza il controller per la tastiera
        self.keyboard_controller = KeyboardController(self)
        
        # Imposta un publisher con QoS transient_local per conservare l’ultimo messaggio
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.offender_publisher = self.create_publisher(String, 'offenders_topic', qos_profile)
        
        # Lista delle tartarughe spawnate sono stringhe che rappresentano il nome delle tartarughe! non oggetti di tipo tourtle per semplicita!
        self.offender = []

        self.get_logger().info("TurtleSpawner avviato. Premi 'n' per spawnare, 'k' per killare, 'q' per uscire.")

    def publish_offenders(self):
        msg = String()
        # Codifica la lista in formato JSON
        msg.data = json.dumps(self.offender)
        self.offender_publisher.publish(msg)
        self.get_logger().info("Lista offender pubblicata: " + str(self.offender))

    def spawn_turtle(self, name: str, x: float, y: float, theta: float = 0.0):
        self.get_logger().info(f"Spawning turtle '{name}' at (x={x}, y={y}, theta={theta}).")
        # Attende che il servizio /spawn sia disponibile
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Attendo il servizio /spawn...")
        request = Spawn.Request()
        request.name  = name
        request.x     = x
        request.y     = y
        request.theta = theta

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            spawned_name = future.result().name
            self.get_logger().info(f"Turtle '{spawned_name}' spawnata con successo.")
            self.offender.append(spawned_name)
            self.publish_offenders()  # Pubblica subito l'aggiornamento
        else:
            self.get_logger().error(f"Errore nello spawning della turtle '{name}'.")

    def kill_turtle(self, name: str):
        self.get_logger().info(f"Eliminazione della turtle '{name}' in corso...")
        # Attende che il servizio /kill sia disponibile
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Attendo il servizio /kill...")
        request = Kill.Request()
        request.name = name

        future = self.kill_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Turtle '{name}' killata con successo.")
            if name in self.offender:
                self.offender.remove(name)
                self.publish_offenders()  # Pubblica subito l'aggiornamento
        else:
            self.get_logger().error(f"Errore nella kill della turtle '{name}'.")

def main(args=None):
    """Main loop per il nodo TurtleSpawner."""
    rclpy.init(args=args)
    node = TurtleSpawner()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if not node.keyboard_controller.listen_for_input():
                break  # Esce dal loop se l'utente preme 'q'
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

if __name__ == '__main__':
    main()
