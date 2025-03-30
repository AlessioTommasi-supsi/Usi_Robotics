import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
import math
import time

################################################################################
#                           CLASSE Drawer
################################################################################

class Drawer(Node):
    def __init__(self, turtle_name, segments):
        """
        :param turtle_name: nome della tartaruga (stringa)
        :param segments: lista di dizionari, es:
            [
              {
                "type": "move" or "rotate",
                "duration": 20,              # numero di 'tick' (0.1s ciascuno)
                "linear_speed": 1.0,
                "angular_speed": 0.5
              },
              ...
            ]
        """
        super().__init__(f'drawer_{turtle_name.lower()}')
        self.turtle_name = turtle_name

        # Publisher per inviare Twist a /<turtle_name>/cmd_vel
        self.vel_publisher = self.create_publisher(
            Twist,
            f'/{self.turtle_name}/cmd_vel',
            10
        )

        # (Opzionale) Subscriber per monitorare la pose
        self.pose_subscriber = self.create_subscription(
            Pose,
            f'/{self.turtle_name}/pose',
            self.pose_callback,
            10
        )

        # Timer di controllo (0.1 s per tick)
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Lista dei segmenti di disegno
        self.segments = segments
        self.current_segment_index = 0
        self.segment_tick_count = 0  # Contatore per il segmento corrente

        # Flag che indica se il disegno della lettera è completato
        self.done = False

    def pose_callback(self, pose):
        # Debug: stampiamo la posizione (livello di log debug)
        self.get_logger().debug(
            f"[{self.turtle_name}] Pose: x={pose.x:.2f}, y={pose.y:.2f}, theta={pose.theta:.2f}"
        )

    def timer_callback(self):
        # Se abbiamo finito tutti i segmenti, fermiamo il timer e segniamo il completamento
        if self.current_segment_index >= len(self.segments):
            self.get_logger().info(f"Lettera completata ({self.turtle_name}).")
            # Ferma il movimento
            stop_msg = Twist()
            self.vel_publisher.publish(stop_msg)
            self.timer.cancel()
            self.done = True
            return

        # Recupera il segmento corrente
        seg = self.segments[self.current_segment_index]
        vel_msg = Twist()
        vel_msg.linear.x = seg["linear_speed"]
        vel_msg.angular.z = seg["angular_speed"]
        self.vel_publisher.publish(vel_msg)

        self.segment_tick_count += 1

        # Se il segmento corrente è terminato, passa al successivo
        if self.segment_tick_count >= seg["duration"]:
            self.get_logger().info(
                f"[{self.turtle_name}] Segmento {self.current_segment_index + 1} completato."
            )
            self.current_segment_index += 1
            self.segment_tick_count = 0


################################################################################
#                         FUNZIONI DI SUPPORTO: SPAWN
################################################################################

def spawn_turtle(node, name, x, y, theta=0.0):
    """
    Spawna una tartaruga in turtlesim chiamando il servizio /spawn.
    :param node: nodo rclpy per fare la call
    :param name: nome della nuova tartaruga
    :param x, y, theta: posizione e orientamento
    """
    client = node.create_client(Spawn, 'spawn')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Attendo servizio /spawn...')

    req = Spawn.Request()
    req.name = name
    req.x = x
    req.y = y
    req.theta = theta

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        node.get_logger().info(f"Tartaruga '{future.result().name}' spawnata con successo.")
    else:
        node.get_logger().error(f"Errore nello spawning della tartaruga '{name}'.")


################################################################################
#                   DEFINIZIONE DEI SEGMENTI PER U, S, I
################################################################################

# Lettera U:
# 1) Ruota di -90° (verso il basso)
# 2) Muove 3 unità in giù
# 3) Ruota di +90° (verso destra)
# 4) Muove 3 unità a destra
# 5) Ruota di +90° (verso l'alto)
# 6) Muove 3 unità in su
segments_U = [
    {"type": "rotate", "duration": 10, "linear_speed": 0.0, "angular_speed": -math.radians(90)},
    {"type": "move",   "duration": 30, "linear_speed": 1.0, "angular_speed": 0.0},
    {"type": "rotate", "duration": 10, "linear_speed": 0.0, "angular_speed": math.radians(90)},
    {"type": "move",   "duration": 30, "linear_speed": 1.0, "angular_speed": 0.0},
    {"type": "rotate", "duration": 10, "linear_speed": 0.0, "angular_speed": math.radians(90)},
    {"type": "move",   "duration": 30, "linear_speed": 1.0, "angular_speed": 0.0},
]

# Lettera S:
# Due movimenti curvi: prima verso destra, poi verso sinistra
segments_S = [
    {"type": "move", "duration": 20, "linear_speed": 1.0, "angular_speed": -0.5},
    {"type": "move", "duration": 20, "linear_speed": 1.0, "angular_speed": 0.5},
]

# Lettera I:
# Ruota verso il basso e disegna una linea verticale (3 unità)
segments_I = [
    {"type": "rotate", "duration": 10, "linear_speed": 0.0, "angular_speed": -math.radians(90)},
    {"type": "move",   "duration": 30, "linear_speed": 1.0, "angular_speed": 0.0},
]


################################################################################
#                                 MAIN
################################################################################

def main(args=None):
    rclpy.init(args=args)

    # Nodo principale per spawnare le tartarughe
    main_node = rclpy.create_node('main_controller')

    # Posizioni per le lettere (centrate verticalmente a y=5.5)
    letter_positions = {
        'U': (2.0, 5.5),
        'S': (6.0, 5.5),
        'I': (10.0, 5.5),
    }

    # Elenco delle lettere e dei relativi segmenti
    letters = [
        ('TURTLE_U', segments_U, letter_positions['U']),
        ('TURTLE_S', segments_S, letter_positions['S']),
        ('TURTLE_I', segments_I, letter_positions['I']),
    ]

    for turtle_name, segments, (x, y) in letters:
        # Spawn della tartaruga per la lettera corrente
        spawn_turtle(main_node, turtle_name, x, y, theta=0.0)
        # Creiamo il Drawer per disegnare la lettera
        drawer = Drawer(turtle_name, segments)
        main_node.get_logger().info(f"Inizio disegno della lettera con '{turtle_name}'.")
        # Usiamo un ciclo per attendere che il Drawer termini il disegno
        while rclpy.ok() and not drawer.done:
            rclpy.spin_once(drawer, timeout_sec=0.1)
        # Distruggiamo il nodo Drawer, lasciando la tartaruga visibile
        drawer.destroy_node()
        # Piccola pausa prima di passare alla lettera successiva
        time.sleep(0.5)

    main_node.get_logger().info("Disegno 'USI' completato (tartarughe multiple rimaste a schermo).")
    main_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
