import math
import time
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, DurabilityPolicy
import json

from new_custom_pakage.controller.move2goal_node import Move2GoalNode

class TurtleBehaviour(Node):
    """
    Controller per la tartaruga che esegue due modalità:
      - "writing": disegna .
      - "chasing": se la tartaruga bersaglio (turtle1) si avvicina, 
                    interrompe il disegno e inizia ad inseguirla usando
                    un controllo proporzionale (move2goal).
    """
    def __init__(self, turtle):
        super().__init__(f'turtle_behaviour_{turtle.name.lower()}')
        self.turtle = turtle

        # Stato iniziale e parametri
        self._state = "writing"
        self._k2 = 2.0  # Soglia per passare in "chasing"
        self._my_pose = None
        self._target_pose = None

        # Sottoscrizioni fisse per la propria pose e quella del bersaglio (turtle1)
        self.create_subscription(Pose, f'/{self.turtle.name}/pose', self.my_pose_callback, 10)
        self.create_subscription(Pose, '/turtle1/pose', self.target_pose_callback, 10)
        
        # Sottoscrizione al topic degli offender (con QoS TRANSIENT_LOCAL)
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, 'offenders_topic', self.offenders_callback, qos_profile)
        
        # Lista dei nomi di offender già registrati, e archivio delle sottoscrizioni dinamiche
        
        self.offender_subs = {}
        
        # Publisher per comandare i movimenti della tartaruga inseguitrice
        self.cmd_pub = self.create_publisher(Twist, f'/{self.turtle.name}/cmd_vel', 10)
        
        self.get_logger().info("TurtleBehaviour avviato, in attesa degli aggiornamenti degli offender.")

    def offenders_callback(self, msg):
        """
        Callback per gli aggiornamenti della lista degli offender (in formato JSON).
        Per ogni offender nuovo non ancora registrato, crea una sottoscrizione al topic /<nome_offender>/pose.
        """
        try:
            offenders_received = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Errore nel decodificare il messaggio: {e}")
            return

        self.get_logger().info("Lista degli offender ricevuta: " + str(offenders_received))
        
        # Controlla ogni elemento della lista: per i nuovi offender crea la sottoscrizione
        for offender_name in offenders_received:
            if offender_name not in self.turtle.spawner.offender:
                self.get_logger().info(f"Nuovo offender rilevato: '{offender_name}'. Creazione sottoscrizione per /{offender_name}/pose")
                self.turtle.spawner.offender.append(offender_name)
                # Crea la sottoscrizione per ricevere la pose del nuovo offender
                sub = self.create_subscription(
                    Pose,
                    f'/{offender_name}/pose',
                    self.new_offender_pose_callback, #diventera target_pose_callback
                    10
                )
                self.offender_subs[offender_name] = sub  # Salva il riferimento per evitare il garbage collection

    def new_offender_pose_callback(self, msg):
        """
        Callback per gestire le pose dai nuovi offender.
        Qui puoi aggiungere la logica per modificare il comportamento in base alle pose ricevute.
        """
        #self.get_logger().info("Ricevuta nuova pose da offender: " + str(msg))


    def my_pose_callback(self, msg: Pose):
        """Callback per aggiornare la nostra posizione. """
        #self.get_logger().info(f"Posizione tartaruga: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}")
        self._my_pose = msg
        self._my_pose.x = round(self._my_pose.x, 4)
        self._my_pose.y = round(self._my_pose.y, 4)

    def target_pose_callback(self, msg: Pose):
        """Callback per aggiornare la posizione del bersaglio (turtle1). """
        #self.get_logger().info(f"Posizione bersaglio: x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}")
        self._target_pose = msg
        self._target_pose.x = round(self._target_pose.x, 4)
        self._target_pose.y = round(self._target_pose.y, 4)

    
    def draw(self, set_of_point_to_draw):
        self.get_logger().info("Avvio disegno")
        # Crea un'istanza del controller di inseguimento
        self.chaseController = Move2GoalNode(self._target_pose, 0.01, self.turtle.name)
        
        

        for letter, points in set_of_point_to_draw.items():
            self.get_logger().info(f"Lettera: {letter}")
            
            penOffState = 1  # Inizialmente la penna è disattivata
            for point in points:
                self.get_logger().info(f"Punto: {point}")
                write_target_pose = Pose()
                write_target_pose.x = float(point[0])
                write_target_pose.y = float(point[1])
                
                self.go_to(write_target_pose, isPenOff=penOffState)  # Muovi la tartaruga verso il punto
                self.get_logger().info(f"end of go_to: {point} ")
                # Una volta raggiunto il punto, puoi inserire una breve pausa per vedere il movimento
                time.sleep(1)
                penOffState = 0  # Attiva la penna per disegnare
                
                

        self.get_logger().info("Disegno completato.")

    def go_to(self, target_pose, isPenOff=1):
        """Funzione per muovere la tartaruga verso un target specificato."""
        self.get_logger().info(f"Muovo la tartaruga verso: x={target_pose.x:.2f}, y={target_pose.y:.2f}, theta={target_pose.theta:.2f}")
        self.turtle.set_pen(self, off=isPenOff)

        self.chaseController = Move2GoalNode(target_pose, 0.1, self.turtle.name)
        self.chaseController.start_moving()

        # Aspetta finché il punto non viene raggiunto
        while rclpy.ok():
            rclpy.spin_once(self.chaseController, timeout_sec=0.1)
            rclpy.spin_once(self, timeout_sec=0.1)

            # Se durante il disegno viene attivata la modalità 'chasing', interrompi il disegno
            if self._my_pose is not None:
                if self._target_pose is not None:
                    dx = self._target_pose.x - self._my_pose.x
                    dy = self._target_pose.y - self._my_pose.y
                    distance = math.sqrt(dx * dx + dy * dy)
                    if distance < self._k2:
                        if self._state == "writing":
                            self.onStopWritingPose = Pose()
                            self.onStopWritingPose.x = self._my_pose.x
                            self.onStopWritingPose.y = self._my_pose.y
                        self.get_logger().info("Interruzione disegno: attivata modalità 'chasing'.")
                        self.last_writing_pose = self._my_pose
                        self._state = "chasing"
                        self.chase()
                        self.get_logger().info("fine inseguimento")
                        break
                        
                
            # Calcola la distanza dalla posizione corrente al punto target
            if self._my_pose is not None:
                dx = target_pose.x - self._my_pose.x
                dy = target_pose.y - self._my_pose.y
                dist_to_point = math.sqrt(dx * dx + dy * dy)
                if dist_to_point < 0.1:  # toleranza per considerare il punto raggiunto
                    self.get_logger().info("   Punto raggiunto")
                    break



    def chase(self):
        self.get_logger().info(f"Inizio inseguimento del bersaglio (turtle1) da parte di '{self.turtle.name}'.")
        self.get_logger().info(f"Posizione iniziale bersaglio: x={self._target_pose.x:.2f}, y={self._target_pose.y:.2f}, theta={self._target_pose.theta:.2f}")
        self.get_logger().info(f"Posizione iniziale tartaruga: x={self._my_pose.x:.2f}, y={self._my_pose.y:.2f}, theta={self._my_pose.theta:.2f}")
        self.turtle.set_pen(self, off=1)  # Disattiva la penna

        # Crea un'istanza del controller di inseguimento
        self.chaseController = Move2GoalNode(self._target_pose, 0.1, self.turtle.name)
        
        # Avvia il timer del controller; qui non usiamo il Future per bloccare il nodo, così da aggiornare dinamicamente il goal.
        self.chaseController.start_moving()

        # Ciclo continuo per aggiornare il goal dinamicamente in base alla nuova posizione del bersaglio.
        while rclpy.ok() and self._state == "chasing":
            # Aggiorna il goal del controller con la posizione attuale del bersaglio.
            self.chaseController.goal_pose = self._target_pose

            # Esegui spin sul controller e sul nodo corrente per far scattare i callback
            rclpy.spin_once(self.chaseController, timeout_sec=0.1)
            rclpy.spin_once(self, timeout_sec=0.1)

            # logica per uscire dal chasing
            # Calcola la distanza dalla posizione corrente al punto target
            if self._my_pose is not None and self._target_pose is not None:
                dx = self._target_pose.x - self._my_pose.x
                dy = self._target_pose.y - self._my_pose.y
                dist_to_point = math.sqrt(dx * dx + dy * dy)
                if dist_to_point < 0.1:  # toleranza per considerare il punto raggiunto
                    self.get_logger().info("   Tartaruga raggiunta ritorno a scrittura")
                    self.turtle.spawner.kill_turtle("turtle1")  # kill the turtle that i have chased
                    self._target_pose = None  # reset target pose
                    self.go_to(self.onStopWritingPose, isPenOff=1)  # Muovi la tartaruga verso il punto
                    self._state = "writing"
                    break



        self.get_logger().info("Inseguimento terminato.")

        """
        from geometry_msgs.msg import Twist
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_pub.publish(stop_cmd)
        """
