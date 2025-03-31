# =============================================================================
#                                  MAIN
# =============================================================================


import rclpy
import math
import time

from new_custom_pakage.model.USI_drawer import USI_drawer
from new_custom_pakage.controller.Drawer import Drawer
from new_custom_pakage.controller.TurtleSpawner import TurtleSpawner

def main(args=None):
    # Inizializza la comunicazione con ROS2. Questa chiamata configura il sistema
    # per poter creare nodi, utilizzare servizi, topic, timer, etc.
    rclpy.init(args=args)

    # Istanza del controller spawner, usata per spawnare e poi rimuovere le tartarughe.
    spawner = TurtleSpawner()
    
    # Crea l'oggetto USI_drawer che gestisce la sequenza delle lettere.
    # USI_drawer implementa l'iteratore, quindi possiamo ciclare direttamente su di esso.
    usi_drawer = USI_drawer()

    # Per ogni tartaruga/lettera nell'iteratore (le lettere vengono elaborate in sequenza: U, S, I)
    for turtle in usi_drawer:  
        # La lettera corrente è già associata al comportamento della tartaruga,
        # memorizzata nel nodo TurtleBehaviour referenziato in turtle.behaviour.
        letter = turtle.behaviour.letter  
        
        # Imposta il riferimento allo spawner nel modello Turtle, così che quando chiamerà destroy()
        # quest'ultimo possa eliminare la tartaruga tramite il servizio /kill.
        turtle.spawner = spawner

        # Recupera la posizione
        x, y = turtle.position
        
        # Imposta l'orientamento iniziale (per esempio, ruota di 180° per la lettera S)
        theta = 0.0
        if letter.name == 'S':
            theta = math.radians(180)

        # Spawn della tartaruga in Turtlesim
        spawner.spawn_turtle(turtle.name, x, y, theta)
        
        # Crea un Drawer per gestire il disegno della lettera.
        drawer = Drawer(turtle.name, letter)
        
        # Avvia il disegno della lettera. La chiamata blocca fino a che il disegno non è completato.
        turtle.behaviour.draw(drawer)
        
        # Una volta disegnata, chiama il metodo destroy() sul modello Turtle, che:
        #   - Richiama il metodo kill_turtle() sul Controller Spawner per rimuovere fisicamente la tartaruga.
        #   - Distrugge il nodo di comportamento.
        turtle.destroy()
        
        # Attesa breve fra una lettera e l'altra.
        time.sleep(0.5)

    spawner.get_logger().info("Disegno completato per tutte le lettere.")
    spawner.destroy_node()
    
    # Chiude l'ecosistema di rclpy, liberando tutte le risorse.
    rclpy.shutdown()

if __name__ == '__main__':
    main()
