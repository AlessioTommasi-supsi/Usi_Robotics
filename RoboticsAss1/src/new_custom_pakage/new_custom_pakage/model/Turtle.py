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
