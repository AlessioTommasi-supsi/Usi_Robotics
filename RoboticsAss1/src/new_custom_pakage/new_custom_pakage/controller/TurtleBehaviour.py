
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleBehaviour(Node):
    """
    Il controller TurtleBehaviour gestisce il comportamento della tartaruga, che ha due modalità:
      - "writing": la tartaruga disegna una lettera.
      - "chasing": se una tartaruga offender si avvicina a meno di k2 metri,
                     la tartaruga interrompe il disegno e inizia ad inseguirla.
                      
    Per individuare gli offender, TurtleBehaviour usa la discovery dei topic.
    In particolare, controlla periodicamente tutti i topic che terminano con '/pose'
    e con un nome che inizia con "/turtle_" (escluso il proprio).
    Quando ne viene trovato uno nuovo, crea una sottoscrizione e ne conserva l’ultima posizione.
    """
    def __init__(self, turtle, letter):
        """
        :param turtle: Oggetto Turtle (model) della tartaruga corrente.
        :param letter: Oggetto Letter (model) da disegnare.
        """
        super().__init__(f'turtle_behaviour_{turtle.name.lower()}')
        self.turtle = turtle
        self.letter = letter

        # Stato iniziale: "writing"
        self._state = "writing"
        # Soglia di distanza (k2 in metri)
        self._k2 = 200.0  

        self._my_pose = None                  # Posizione della nostra tartaruga
        self._other_poses = {}                # Dizionario: {turtle_name: Pose} per gli offender
        self._other_subscriptions = {}        # Dizionario: {turtle_name: subscription object}

        # Sottoscrizione per ricevere la nostra propria pose:
        self.create_subscription(Pose, f'/{self.turtle.name}/pose', self.my_pose_callback, 10)
        # Creiamo un timer per aggiornare dinamicamente i sottoscrittori degli offender ogni 0.1 secondo.
        self.create_timer(1.0, self.update_offender_subscriptions)

        # Publisher per comandare i movimenti (per l'inseguimento)
        self.cmd_pub = self.create_publisher(Twist, f'/{self.turtle.name}/cmd_vel', 10)

    def update_offender_subscriptions(self):
        """
        Il metodo controlla periodicamente i topic attivi (usando get_topic_names_and_types)
        per cercare quelli relativi alla posizione di altre tartarughe.
        Se viene trovato un topic che corrisponde a "/turtle*/pose" e NON è il proprio,
        crea una sottoscrizione (se non esiste già) per aggiornare la sua posizione.
        """
        topics = self.get_topic_names_and_types()  # Restituisce una lista di tuple (topic, [types])
        self.get_logger().info(f"Topic correnti: {[t for t, types in topics]}")
    
        for topic, types in topics:
            # Verifica che il topic finisca con '/pose'
            if topic.endswith('/pose'):
                # Controlla che inizi con "/turtle" e non sia il proprio topic
                if topic.startswith('/turtle') and topic != f'/{self.turtle.name}/pose':
                    # Estrae il nome della tartaruga, ad esempio, da "/turtle1/pose":
                    turtle_name = topic.split('/')[1]  # per "/turtle1/pose" -> "turtle1"
                    if turtle_name not in self._other_subscriptions:
                        # Crea un sottoscrittore per questo topic e aggiorna il dizionario
                        sub = self.create_subscription(
                            Pose,
                            topic,
                            lambda msg, tn=turtle_name: self.other_pose_callback(tn, msg),
                            10
                        )
                        self._other_subscriptions[turtle_name] = sub
                        self.get_logger().info(f"Subscribed to offender pose: {topic}")

    def other_pose_callback(self, turtle_name, msg: Pose):
        """
        Callback che aggiorna la posizione di un’altra tartaruga (offender).
        """
        self._other_poses[turtle_name] = msg
        self.check_offenders()

    def my_pose_callback(self, msg: Pose):
        """
        Callback per aggiornare la nostra posizione.
        """
        self._my_pose = msg
        self.check_offenders()

    def check_offenders(self):
        """
        Controlla, per ogni tartaruga monitorata, se la distanza dalla nostra
        è minore di k2. Se sì e lo stato corrente è "writing", passa alla modalità "chasing".
        """
        if self._my_pose is None:
            return

        for name, pose in self._other_poses.items():
            dx = pose.x - self._my_pose.x
            dy = pose.y - self._my_pose.y
            distance = math.sqrt(dx * dx + dy * dy)
            if distance < self._k2:
                if self._state != "chasing":
                    self.get_logger().info(
                        f"Turtle '{name}' si è avvicinata (d={distance:.2f}m). Cambio stato in 'chasing'!"
                    )
                    self._state = "chasing"
                    # Interrompi eventuali loop di disegno se attivi.
                return  # Esci se ne trovi almeno uno

    def draw(self, drawer):
        """
        Esegue il ciclo di disegno (modalità 'writing'). Se in qualsiasi momento lo stato
        diventa 'chasing', interrompe il disegno e passa all'inseguimento (chase).
        """
        self.get_logger().info(
            f"Avvio disegno della lettera '{self.letter.name}' in modalità '{self._state}'."
        )
        while rclpy.ok() and not drawer.done:
            rclpy.spin_once(drawer, timeout_sec=0.1)
            if self._state == "chasing":
                self.get_logger().info("Interruzione disegno: attivata modalità 'chasing'.")
                self.chase()
                return
        if drawer.done:
            self.get_logger().info(
                f"Disegno completato per la lettera '{self.letter.name}'."
            )

    def chase(self):
        """
        Esegue l'inseguimento dell'offender più vicino, usando un semplice
        controllore proporzionale per calcolare i comandi lineari e angolari.
        """
        self.get_logger().info("Avvio comportamento 'chase'.")
        while rclpy.ok():
            if self._my_pose is None or not self._other_poses:
                rclpy.spin_once(self, timeout_sec=0.1)
                continue

            # Cerca l'offender più vicino
            offender_name = None
            min_distance = float('inf')
            offender_pose = None
            for name, pose in self._other_poses.items():
                dx = pose.x - self._my_pose.x
                dy = pose.y - self._my_pose.y
                distance = math.sqrt(dx * dx + dy * dy)
                if distance < min_distance:
                    min_distance = distance
                    offender_name = name
                    offender_pose = pose

            if offender_pose is None:
                break

            desired_angle = math.atan2(offender_pose.y - self._my_pose.y,
                                         offender_pose.x - self._my_pose.x)
            angular_error = desired_angle - self._my_pose.theta

            # Normalizza l'errore a [-pi, pi]
            while angular_error > math.pi:
                angular_error -= 2 * math.pi
            while angular_error < -math.pi:
                angular_error += 2 * math.pi

            twist = Twist()
            twist.linear.x = 0.5 * min_distance    # guadagno lineare
            twist.angular.z = 2.0 * angular_error    # guadagno angolare
            self.cmd_pub.publish(twist)

            # Se la distanza diventa molto piccola, supponiamo di aver "raggiunto" l'offender
            if min_distance < 0.5:
                self.get_logger().info(f"Offender '{offender_name}' raggiunto!")
                break

            time.sleep(0.1)
