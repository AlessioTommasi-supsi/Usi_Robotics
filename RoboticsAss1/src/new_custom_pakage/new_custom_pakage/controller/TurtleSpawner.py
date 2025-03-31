# =============================================================================
#                    CONTROLLER: TurtleSpawner
# =============================================================================


import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill

class TurtleSpawner(Node):
    """
    TurtleSpawner si occupa di spawnare e, se necessario, rimuovere le tartarughe in Turtlesim.
    Utilizza i servizi ROS /spawn e /kill.
    """
    def __init__(self):
        super().__init__('turtle_spawner')
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client  = self.create_client(Kill, 'kill')

    def spawn_turtle(self, name: str, x: float, y: float, theta: float = 0.0):
        self.get_logger().info(f"Spawning turtle '{name}' at (x={x}, y={y}, theta={theta}).")
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Attendo il servizio /spawn...")
        request = Spawn.Request()
        request.name = name
        request.x = x
        request.y = y
        request.theta = theta

        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Turtle '{future.result().name}' spawnata con successo.")
        else:
            self.get_logger().error(f"Errore nello spawning della turtle '{name}'.")

    def kill_turtle(self, name: str):
        self.get_logger().info(f"Eliminazione della turtle '{name}' in corso...")
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Attendo il servizio /kill...")
        request = Kill.Request()
        request.name = name

        future = self.kill_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"Turtle '{name}' rimossa con successo.")
        else:
            self.get_logger().error(f"Errore nella rimozione della turtle '{name}'.")
