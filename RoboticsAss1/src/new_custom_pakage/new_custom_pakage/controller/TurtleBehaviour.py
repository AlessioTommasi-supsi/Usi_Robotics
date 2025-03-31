import rclpy
from rclpy.node import Node
from new_custom_pakage.controller.Drawer import Drawer

class TurtleBehaviour(Node):
    """
    Il controller TurtleBehaviour gestisce il comportamento della tartaruga.
    Viene creato automaticamente quando si crea una Turtle tramite USI_drawer.
    Offre il metodo draw(drawer) per eseguire il disegno della lettera,
    e il metodo chase(target_turtle) per in futuro eseguire l'inseguimento.
    """
    def __init__(self, turtle, letter):
        super().__init__(f'turtle_behaviour_{turtle.name.lower()}')
        self.turtle = turtle
        self.letter = letter

    def draw(self, drawer: Drawer):
        self.get_logger().info(f"Inizio disegno della lettera '{self.letter.name}' con turtle {self.turtle.name}.")
        while rclpy.ok() and not drawer.done:
            rclpy.spin_once(drawer, timeout_sec=0.1)
        self.get_logger().info(f"Disegno completato per la lettera '{self.letter.name}' con turtle {self.turtle.name}.")

    def chase(self, target_turtle):
        self.get_logger().info(f"Turtle {self.turtle.name} insegue la turtle {target_turtle.name}.")
        # Qui andrà la logica di inseguimento
