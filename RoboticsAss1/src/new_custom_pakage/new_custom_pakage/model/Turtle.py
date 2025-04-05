from turtlesim.srv import SetPen
import rclpy

class Turtle:
    """
    Il modello Turtle contiene le informazioni relative a una tartaruga:
      - nome
      - posizione iniziale (coordinate)
      - comportamento (l'attributo 'behaviour') che verrà impostato dal controller.
      - spawner: riferimento al controller TurtleSpawner per effettuare il despawn.
    """
    def __init__(self, name: str, position: tuple = None):
        self.name = name
        self.position = position
        self.behaviour = None  # verrà impostato automaticamente (vedi USI_drawer)
        self.spawner = None    # da assegnare dal main

    def destroy(self):
        """
        Esegue il despawn della tartaruga utilizzando lo spawner (se assegnato)
        e distrugge il nodo del comportamento.
        """
        if self.spawner is not None:
            self.spawner.get_logger().info(f"Despawning turtle {self.name} tramite lo spawner.")
            self.spawner.kill_turtle(self.name)
        else:
            if self.behaviour is not None:
                self.behaviour.get_logger().error("Spawner non assegnato per la turtle!")
        if self.behaviour:
            self.behaviour.get_logger().info(f"Destroying behaviour node for turtle {self.name}")
            self.behaviour.destroy_node()


    def set_pen(self, node, r=0, g=0, b=0, width=3, off=0):
        """
        Controlla la penna della tartaruga tramite il servizio /<turtle_name>/set_pen.
        
        :param node: il nodo (es. lo spawner o il behaviour) che viene utilizzato per chiamare il servizio.
        :param r: rosso (0-255)
        :param g: verde (0-255)
        :param b: blu (0-255)
        :param width: larghezza della penna.
        :param off: 0 per pen down (disegna), 1 per pen up (non disegna).
        """
        client = node.create_client(SetPen, f'/{self.name}/set_pen')
        if not client.wait_for_service(timeout_sec=2.0):
            node.get_logger().error("Servizio set_pen non disponibile!")
            return
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future)
        node.get_logger().info(f"Pen {'disattivata' if off else 'attivata'} per {self.name}")