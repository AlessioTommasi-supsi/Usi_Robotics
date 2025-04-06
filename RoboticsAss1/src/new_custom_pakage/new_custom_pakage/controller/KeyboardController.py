import sys
from geometry_msgs.msg import Twist  # Necessario per inviare i comandi di movimento
from new_custom_pakage.model.Turtle import Turtle
class KeyboardController:
    """Classe per gestire l'input da tastiera e muovere le tartarughe offender."""
    
    def __init__(self, node):
        self.node = node
        self.selected_offender = None  # Offender attivo per il controllo
        self.velocity_pubs = {}       # Dizionario di publisher per ogni offender

        # Istruzioni da stampare all'avvio
        message = (
            "Istruzioni di controllo:\n"
            "   n -> Spawn nuova tartaruga (e seleziona quella appena aggiunta)\n"
            "   k -> Kill offender selezionato; dopo l'uccisione, seleziona automaticamente l'ultimo offender della lista\n"
            "   o -> Selezione manuale dell'offender (inserisci numero)\n"
            "   w -> Muovi avanti\n"
            "   s -> Muovi indietro\n"
            "   a -> Ruota a sinistra\n"
            "   d -> Ruota a destra\n"
            "   q -> Esci\n"
        )
        self.node.get_logger().info(message)
    
    def update_offender_selection(self):
        """Seleziona manualmente un offender da muovere (inserendo il suo numero)."""
        self.node.get_logger().info(f"Offender disponibili: {self.node.offender}")
        print("Inserisci il numero dell'offender da controllare:")

        # Usa readline per acquisire l'intera riga di input
        num_line = sys.stdin.readline().strip()
        try:
            num = int(num_line)
            if 1 <= num <= len(self.node.offender):
                self.selected_offender = self.node.offender[num - 1]
                msg = f"Controllo cambiato su: {self.selected_offender}"
                self.node.get_logger().info(msg)
                print(msg)
            else:
                msg = "Numero non valido."
                self.node.get_logger().info(msg)
                print(msg)
        except ValueError:
            msg = "Input non valido."
            self.node.get_logger().info(msg)
            print(msg)
    
    def send_velocity_command(self, linear_x=0.0, angular_z=0.0):
        """Invia un comando di velocità all'offender selezionato mediante Twist."""
        if not self.selected_offender:
            # Se nessun offender è selezionato, di default prendi l'ultimo spawnato, se disponibile.
            if self.node.offender:
                self.selected_offender = self.node.offender[-1]
            else:
                msg = "Nessun offender da controllare."
                self.node.get_logger().info(msg)
                print(msg)
                return
        
        # Se il publisher per questo offender non esiste già, crealo.
        if self.selected_offender not in self.velocity_pubs:
            self.velocity_pubs[self.selected_offender] = self.node.create_publisher(
                Twist, f'/{self.selected_offender}/cmd_vel', 10)
        
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.velocity_pubs[self.selected_offender].publish(twist)
        
        msg = (f"Movimento inviato a {self.selected_offender}: "
               f"linear_x={linear_x}, angular_z={angular_z}")
        self.node.get_logger().info(msg)
        print(msg)
    
    def kill_selected_offender(self):
        """
        Uccide l'offender attualmente selezionato (o, se non selezionato, l'ultimo offender della lista)
        e aggiorna la selezione al nuovo ultimo offender presente, se la lista non è vuota.
        """
        if self.node.offender:
            # Se nessun offender è selezionato, seleziona l'ultimo.
            if not self.selected_offender:
                self.selected_offender = self.node.offender[-1]
            
            offender_to_kill = self.selected_offender
            self.node.kill_turtle(offender_to_kill)
            # Dopo la kill, la lista degli offender viene aggiornata nel nodo.
            if self.node.offender:
                # Imposta come selected offender l'ultimo offender della lista
                self.selected_offender = self.node.offender[-1]
                msg = f"Offender killato: {offender_to_kill}. Selezionato: {self.selected_offender}"
            else:
                self.selected_offender = None
                msg = f"Offender killato: {offender_to_kill}. Nessun offender rimasto."
            self.node.get_logger().info(msg)
            print(msg)
        else:
            msg = "Nessun offender da killare."
            self.node.get_logger().info(msg)
            print(msg)
    
    def listen_for_input(self):
        """Ascolta gli input da tastiera e gestisce i comandi.
        
        Comandi:
         - 'n': Spawn una nuova tartaruga e seleziona quella appena aggiunta.
         - 'k': Kill l'offender selezionato (se non selezionato, kill l'ultimo offender) e poi aggiorna la selezione.
         - 'o': Selezione manuale dell'offender (richiede di inserire il numero)
         - 'w': Muovi avanti
         - 's': Muovi indietro
         - 'a': Ruota a sinistra
         - 'd': Ruota a destra
         - 'q': Esci dal nodo
        """
        cmd = sys.stdin.readline().strip()
        
        if cmd == 'n':
            turtle_name = f"offender_{len(self.node.offender)+1}"
            self.node.spawn_turtle(turtle_name, 1.0, 1.0, 0.0) #sara uno spawner il nodo!
            # Dopo lo spawn, seleziona automaticamente il nuovo offender
            self.selected_offender = turtle_name

            tmp_turtle = Turtle(turtle_name, position=(1.0, 1.0))
            tmp_turtle.set_pen(self.node, r=255, g=0, b=0, width=3, off=1)  # Penna disattivata

        elif cmd == 'k':
            self.kill_selected_offender()
        elif cmd == 'o':
            self.update_offender_selection()
        elif cmd == 'w':
            self.send_velocity_command(linear_x=1.0)
        elif cmd == 's':
            self.send_velocity_command(linear_x=-1.0)
        elif cmd == 'a':
            self.send_velocity_command(angular_z=1.0)
        elif cmd == 'd':
            self.send_velocity_command(angular_z=-1.0)
        elif cmd == 'q':
            self.node.get_logger().info("Uscita richiesta. Chiusura del nodo.")
            print("Uscita richiesta. Chiusura del nodo.")
            return False
        
        return True
