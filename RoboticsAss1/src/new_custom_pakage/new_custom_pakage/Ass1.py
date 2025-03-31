# =============================================================================
#                                  MAIN
# =============================================================================


import rclpy
import math
import time

# Importiamo le classi definite nei file separati
from new_custom_pakage.model.Letter import Letter
from new_custom_pakage.model.USI_write import USI_write
from new_custom_pakage.controller.TurtleSpawner import TurtleSpawner
from new_custom_pakage.controller.Drawer import Drawer

def main(args=None):
    rclpy.init(args=args)

    # Istanza del model che associa le posizioni alle lettere
    usi_model = USI_write()
    
    # Nodo per gestire lo spawning e la rimozione delle tartarughe
    spawner = TurtleSpawner()

    # Elenco delle lettere da eseguire in sequenza: U, poi S, poi I.
    for letter_key in ['U', 'S', 'I']:
        # Creiamo l'oggetto Letter (i segmenti sono definiti in Letter.py)
        letter = Letter(letter_key)
        # Otteniamo la posizione dal modello USI_write
        x, y = usi_model.positions[letter_key]
        turtle_name = f"TURTLE_{letter_key}"
        
        # Impostiamo l'orientamento iniziale: per S ad esempio ruotiamo di 180°
        theta = 0.0
        if letter_key == 'S':
            theta = math.radians(180)

        # Spawn della tartaruga per la lettera corrente
        spawner.spawn_turtle(turtle_name, x, y, theta)

        # Creiamo il Drawer dedicato a far disegnare la lettera.
        drawer = Drawer(turtle_name, letter)

        # Aspettiamo (spin) finché il Drawer non segnala che il disegno è completato.
        while rclpy.ok() and not drawer.done:
            rclpy.spin_once(drawer, timeout_sec=0.1)

        # Una volta completato il disegno, uccidiamo la tartaruga
        spawner.kill_turtle(turtle_name)

        # Distruggiamo il nodo Drawer per pulire le risorse
        drawer.destroy_node()

        # Piccola pausa prima di procedere con la lettera successiva
        time.sleep(0.5)

    # Terminiamo il nodo spawner e shutdown
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
