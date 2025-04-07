import math
import time
import sys

import random

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, DurabilityPolicy

from new_custom_pakage.model.Offender import Offender


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

        self.chaseController = None  # Inizializza il controller di inseguimento a None

        # Stato iniziale e parametri
        self._state = "writing"
        self._k2 = 2.0  # Soglia per passare in "chasing"
        self._k1 = 0.5  # Soglia per eliminare l'offender
        self._my_pose = None
        self._target_pose = None
        self.offender_list = []

        # Sottoscrizioni fisse per la propria pose e quella del bersaglio (turtle1)
        self.create_subscription(Pose, f'/{self.turtle.name}/pose', self.my_pose_callback, 10)
        self.turtle1_pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.target_pose_callback, 10)
        self._target_velocity = Twist()  # Inizializza la velocità del bersaglio a None
        self._target_velocity.linear.x = 0.0
        self._target_velocity.angular.z = 0.0
        
         
        # Sottoscrizione al topic degli offender (con QoS TRANSIENT_LOCAL)
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(String, 'offenders_topic', self.offenders_callback, qos_profile)
        
        # Lista dei nomi di offender già registrati, e archivio delle sottoscrizioni dinamiche
        
        self.offender_subs = {}
        
        
        self.get_logger().info("TurtleBehaviour avviato, in attesa degli aggiornamenti degli offender.")

    def offenders_callback(self, msg):
        """
        Callback per aggiornare la lista degli offender ricevuta (in formato JSON).
        La lista ricevuta contiene i nomi degli offender.
        Per ogni offender nuovo non presente in offender_list,
        crea una sottoscrizione a '/<offender_name>/pose' e aggiungi un nuovo oggetto Offender.
        """
        try:
            offenders_received = json.loads(msg.data)  # presupponiamo una lista di nomi
            self.get_logger().info("Lista degli offender ricevuta: " + str(offenders_received))
            
            for offender_name in offenders_received:
                # Controlla se l'offender è già presente nella nostra lista
                if not any(off.name == offender_name for off in self.offender_list) and offender_name != "" and offender_name != "turtle1" and offender_name != self.turtle.name:
                    self.get_logger().info(f"Nuovo offender rilevato: '{offender_name}'. Creazione sottoscrizione per /{offender_name}/pose")
                    
                    # Crea e aggiungi l'oggetto Offender alla lista.
                    new_offender = Offender(name=offender_name)
                    self.offender_list.append(new_offender)
                    
                    #NON FUNZIONA!! se la decommento si rompe se non ce spowner attivo Crea la sottoscrizione per ricevere la pose dell'offender.
                    # Utilizziamo lambda per passare offender_name al callback.
                    
                    
                    sub = self.create_subscription(
                        Pose,
                        f'/{offender_name}/pose',
                        self.create_offender_callback(offender_name),
                        10
                    )
                    
                    
                    
                    
                    self.offender_subs[offender_name] = sub
        except Exception as e:
            self.get_logger().error(f"Errore nel decodificare il messaggio: {e}")
            return
                
    def create_offender_callback(self, offender_name):
        def callback(msg: Pose):
            self.new_offender_pose_callback(offender_name, msg)
        return callback  

    def new_offender_pose_callback(self, offender_name, msg: Pose):
        """
        Callback per gestire le pose dai nuovi offender.
        Aggiorna continuamente il campo target_position dell'oggetto Offender corrispondente.
        """
        
        try:
            # Cerca l'offender in offender_list usando il nome passato.
            for offender in self.offender_list:
                if offender.name == offender_name:
                    offender.target_position = msg  # aggiorna sempre, non solo se è None
                    
                    self.get_logger().info(
                        f"Aggiornata target_position per offender {offender.name}: "
                        f"x={msg.x:.2f}, y={msg.y:.2f}, theta={msg.theta:.2f}")
                    
                    break
        except Exception as e:
            self.get_logger().error(f"Errore nella callback dell'offender {offender_name}: {e}")
            return
        


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

    def go_to(self, target, isPenOff=1):
        

        """Funzione per muovere la tartaruga verso un target specificato."""
        self.get_logger().info(f"Muovo la tartaruga verso: x={target.x:.2f}, y={target.y:.2f}, theta={target.theta:.2f}")
        self.turtle.set_pen(self, off=isPenOff)

        """
        if self.chaseController is not None:
            self.chaseController.stop_moving()
            self.chaseController.destroy_node()
            self.chaseController = None  # Reset del chaseController
        """

        self.chaseController = Move2GoalNode(target, 0.1, self.turtle.name)
        self.chaseController.start_moving()


        # Aspetta finché il punto non viene raggiunto
        while rclpy.ok():
            rclpy.spin_once(self.chaseController, timeout_sec=0.1)
            rclpy.spin_once(self, timeout_sec=0.1)

            # Se durante il disegno viene attivata la modalità 'chasing', interrompi il disegno
            if self._target_pose is not None: #dovra essere mofificata con un for per ogni offender
                if self._my_pose is not None:
                    euclidean_distance = self.chaseController.euclidean_distance(self._target_pose, self._my_pose)
               
                    if euclidean_distance < self._k2:  # Se la tartaruga bersaglio è vicina
                        if self._state == "writing":
                            self.onStopWritingPose = Pose()
                            self.onStopWritingPose.x = self._my_pose.x
                            self.onStopWritingPose.y = self._my_pose.y
                        self.get_logger().info("Interruzione: attivata modalità 'chasing'.")
                        self.last_writing_pose = self._my_pose
                        self._state = "chasing"
                        self.chaseController.stop_moving()
                        self.chaseController.destroy_node()
                        self.chase()
                        self.get_logger().info("sono tornato a scrivere dal punto un cui ho interrotto")
                        
                        self.go_to(target, isPenOff) #riprendo scrittura da dove ho interrotto
                        return
                
            # Calcola la distanza dalla posizione corrente al punto target
            self.chaseController.tolerance = 0.1
            if self._my_pose is not None:
                euclidean_distance = self.chaseController.euclidean_distance(target, self._my_pose)  
                if euclidean_distance < self.chaseController.tolerance:
                    self.get_logger().info("   Punto raggiunto")
                    self.chaseController.stop_moving()
                    self.chaseController.destroy_node()
                    break


    def chase(self):
        self.get_logger().info(f"Inizio inseguimento del bersaglio (turtle1) da parte di '{self.turtle.name}'.")
        self.get_logger().info(f"Posizione iniziale bersaglio: x={self._target_pose.x:.2f}, y={self._target_pose.y:.2f}, theta={self._target_pose.theta:.2f}")
        self.get_logger().info(f"Posizione iniziale tartaruga: x={self._my_pose.x:.2f}, y={self._my_pose.y:.2f}, theta={self._my_pose.theta:.2f}")
        self.turtle.set_pen(self, off=1)  # Disattiva la penna

        # Crea un'istanza del controller di inseguimento
        self.chaseController = Move2GoalNode(self._target_pose, 0.1, self.turtle.name)
        self.chaseController.tolerance = self._k1  # Soglia per considerare il bersaglio raggiunto

        # Avvia il timer del controller; in questo modo il goal può essere aggiornato dinamicamente
        self.chaseController.start_moving()

        # Costante per il calcolo dell'intercetta
        k_intercept = 0.1

        # Ciclo continuo per aggiornare il goal dinamicamente in base alla nuova posizione e velocità del bersaglio.
        while rclpy.ok() and self._state == "chasing":
            if self._my_pose is not None and self._target_pose is not None and self._target_velocity is not None:
                # Calcola la distanza attuale tra la tartaruga e il bersaglio
                distance_current = self.chaseController.euclidean_distance(self._target_pose, self._my_pose)
                # Calcola m, proporzionale alla velocità lineare del target e alla distanza corrente
                m = k_intercept * self._target_velocity.linear.x * distance_current

                # Calcola il punto di intercettazione
                intercept_pose = Pose()
                intercept_pose.x = self._target_pose.x + m * math.cos(self._target_pose.theta)
                intercept_pose.y = self._target_pose.y + m * math.sin(self._target_pose.theta)
                intercept_pose.theta = self._target_pose.theta  # oppure personalizza l'orientamento se necessario

                # Imposta il goal del controller come il punto di intercettazione
                self.chaseController.goal_pose = intercept_pose
            else:
                # Se non sono disponibili tutte le informazioni, usa direttamente la posizione target
                self.chaseController.goal_pose = self._target_pose

            # Esegue spin per processare i callback del controller e del nodo corrente
            rclpy.spin_once(self.chaseController, timeout_sec=0.1)
            rclpy.spin_once(self, timeout_sec=0.1)

            # Logica per uscire dal chasing: controlla se la tartaruga ha raggiunto il punto di intercettazione
            if self._my_pose is not None and self._target_pose is not None:
                current_goal = self.chaseController.goal_pose
                euclidean_distance = self.chaseController.euclidean_distance(current_goal, self._my_pose)

                if euclidean_distance < self.chaseController.tolerance:  # punto raggiunto
                    self.chaseController.stop_moving()
                    self.chaseController.destroy_node()
                    self.get_logger().info("   Tartaruga raggiunta: ritorno a 'writing'.")
                    self.turtle.spawner.kill_turtle("turtle1")  # elimina il bersaglio inseguito
                    self.destroy_subscription(self.turtle1_pose_sub)  # distruggi la subscription al bersaglio
                    self._target_pose = None  # reset target pose
                    break

        # Creo di nuovo il target turtle1 in una posizione casuale
        posizione_random = Pose()
        posizione_random.x = random.uniform(1.0, 10.0)
        posizione_random.y = random.uniform(1.0, 10.0)
        posizione_random.theta = random.uniform(-math.pi, math.pi)

        self.turtle.spawner.spawn_turtle("turtle1", posizione_random.x, posizione_random.y, posizione_random.theta)  # spawn turtle1 in una posizione random
        self.get_logger().info(f"Posizione bersaglio spawnato: x={posizione_random.x:.2f}, y={posizione_random.y:.2f}, theta={posizione_random.theta:.2f}")
        
        # Ricrea la subscription per il bersaglio
        self.turtle1_pose_sub = self.create_subscription(Pose, '/turtle1/pose', self.target_pose_callback, 10)
        self._target_pose = Pose()

        # Crea un publisher per i comandi di velocità a turtle1
        self.turtle1_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Crea il messaggio Twist con velocità random (sia lineare che angolare)
        vel_msg = Twist()
        vel_msg.linear.x = random.uniform(1.0, 3.0)
        vel_msg.angular.z = random.uniform(-1.0, 1.0)

        self._target_velocity = vel_msg

        # Pubblica il comando di velocità
        self.turtle1_vel_pub.publish(vel_msg)

        self.get_logger().info("Inseguimento terminato.")
        self._state = "back to writing"
        self.go_to(self.onStopWritingPose, isPenOff=1)  # Muove la tartaruga verso il punto di ripresa
        self._state = "writing"
