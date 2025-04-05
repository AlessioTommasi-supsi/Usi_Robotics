import math
import time
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

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
        """
        :param turtle: oggetto Turtle (model) della tartaruga corrente (inseguitrice).
        
        """
        super().__init__(f'turtle_behaviour_{turtle.name.lower()}')
        self.turtle = turtle

        # Stato iniziale: "writing"
        self._state = "writing"
        # Soglia per passare in "chasing" (in metri)
        self._k2 = 2.0

        # Pose della tartaruga inseguitrice e del bersaglio (turtle1)
        self._my_pose = None
        self._target_pose = None

        

        # Sottoscrizioni:
        # La nostra pose
        self.create_subscription(Pose, f'/{self.turtle.name}/pose', self.my_pose_callback, 10)
        
        
        # Pose del bersaglio (sempre turtle1) per ora poi si vedra
        self.create_subscription(Pose, '/turtle1/pose', self.target_pose_callback, 10)

        # Publisher per comandare i movimenti della tartaruga inseguitrice
        self.cmd_pub = self.create_publisher(Twist, f'/{self.turtle.name}/cmd_vel', 10)

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
        self.chaseController = Move2GoalNode(self._target_pose, 0.1, self.turtle.name)
        
        for letter, points in set_of_point_to_draw.items():
            self.get_logger().info(f"Lettera: {letter}")
            self.turtle.set_pen(self, off=1)  # Disattiva la penna
            for point in points:
                self.get_logger().info(f"Punto: {point}")
                write_target_pose = Pose()
                write_target_pose.x = float(point[0])
                write_target_pose.y = float(point[1])
                
                # Imposta il nuovo goal
                self.chaseController.goal_pose = write_target_pose
                
                # Avvia il controller per questo punto
                done = self.chaseController.start_moving()
                
                # Aspetta finché il punto non viene raggiunto
                while rclpy.ok():
                    rclpy.spin_once(self.chaseController, timeout_sec=0.1)
                    rclpy.spin_once(self, timeout_sec=0.1)

                    # Se durante il disegno viene attivata la modalità 'chasing', interrompi il disegno
                    if self._my_pose is not None and self._target_pose is not None:
                        dx = self._target_pose.x - self._my_pose.x
                        dy = self._target_pose.y - self._my_pose.y
                        distance = math.sqrt(dx * dx + dy * dy)
                        if distance < self._k2:
                            self.get_logger().info("Interruzione disegno: attivata modalità 'chasing'.")
                            self._state = "chasing"
                            self.chase()
                            return
                        
                    # Calcola la distanza dalla posizione corrente al punto target
                    if self._my_pose is not None:
                        dx = write_target_pose.x - self._my_pose.x
                        dy = write_target_pose.y - self._my_pose.y
                        dist_to_point = math.sqrt(dx * dx + dy * dy)
                        if dist_to_point < 0.1:  # toleranza per considerare il punto raggiunto
                            self.get_logger().info("   Punto raggiunto")
                            break
                
                # Una volta raggiunto il punto, puoi inserire una breve pausa per vedere il movimento
                time.sleep(1)
                self.turtle.set_pen(self, off=0)  # Attiva la penna
                
                

        self.get_logger().info("Disegno completato.")



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

            # (Opzionale) Puoi inserire qui una logica per uscire dal chasing, ad esempio se il bersaglio si ferma per un certo tempo.

        self.get_logger().info("Inseguimento terminato.")
        # Ferma la tartaruga inviando un comando di stop
        from geometry_msgs.msg import Twist
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0
        self.cmd_pub.publish(stop_cmd)
