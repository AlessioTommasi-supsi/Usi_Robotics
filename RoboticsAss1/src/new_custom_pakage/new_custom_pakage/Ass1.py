# =============================================================================
#                                  MAIN
# =============================================================================


import rclpy
import math
import time

from new_custom_pakage.controller.TurtleSpawner import TurtleSpawner
from new_custom_pakage.model.USI_write import USI_write
from new_custom_pakage.model.Turtle import Turtle
from new_custom_pakage.controller.TurtleBehaviour import TurtleBehaviour



def main(args=None):
    # Inizializza la comunicazione con ROS2. Questa chiamata configura il sistema
    # per poter creare nodi, utilizzare servizi, topic, timer, etc.
    rclpy.init(args=args)

    # Istanza del controller spawner, usata per spawnare e poi rimuovere le tartarughe.
    spawner = TurtleSpawner()
    
    set_of_point_to_draw = USI_write().point

    
    spawner.spawn_turtle("USI_turtle", 0.0, 0.0, 0.0)
    spawner.get_logger().info("Tartaruga USI_turtle spawnata.")

    USI_turtle = Turtle("USI_turtle", position=(0.0, 0.0))
    USI_turtle.spawner = spawner  # Assegna lo spawner alla tartaruga
    
    USI_turtle.behaviour = TurtleBehaviour(USI_turtle)

    USI_turtle.behaviour.draw(set_of_point_to_draw)

    spawner.kill_turtle("USI_turtle")
    USI_turtle.destroy()
    spawner.get_logger().info("Disegno completato per tutte le lettere.")
    spawner.destroy_node()
    
    # Chiude l'ecosistema di rclpy, liberando tutte le risorse.
    rclpy.shutdown()

if __name__ == '__main__':
    main()
