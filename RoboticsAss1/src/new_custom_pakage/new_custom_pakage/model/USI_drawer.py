

####################################################################################################
# USI_drawer.py
# Questo file contiene la classe USI_drawer, che gestisce la sequenza delle lettere da disegnare.
# La classe implementa il pattern iterator, permettendo di ciclare attraverso le lettere.
# Ogni lettera è associata a una tartaruga (Turtle) e un comportamento (TurtleBehaviour).
####################################################################################################

from new_custom_pakage.model.Letter import Letter
from new_custom_pakage.model.Turtle import Turtle
from new_custom_pakage.model.USI_write import USI_write
from new_custom_pakage.controller.TurtleBehaviour import TurtleBehaviour

class USI_drawer:
    """
    Questa classe gestisce la sequenza delle lettere da disegnare.
      - getCurrentLetter() restituisce l'oggetto Letter corrente.
      - getCurrentTurtle() crea una Turtle associata alla lettera corrente, assegnandole automaticamente
        un comportamento (TurtleBehaviour) e incrementa il contatore.
    
    Implementa il pattern iterator, così possiamo ciclare con un for.
    """
    def __init__(self):
        self.lettersList = ['U', 'S', 'I']
        self.index = 0
        self.usi_write = USI_write()
        self.lastLetter = None

    def getCurrentLetter(self):
        return self.lastLetter

    def getCurrentTurtle(self):
        if self.index < len(self.lettersList):
            letter_key = self.lettersList[self.index]
            pos = self.usi_write.positions.get(letter_key, (0.0, 0.0))
            turtle_name = f"TURTLE_{letter_key}"
            turtle_obj = Turtle(turtle_name, position=pos)
            letter_obj = Letter(letter_key)
            # Assegna automaticamente il comportamento al turtle (TurtleBehaviour viene creato internamente)
            turtle_obj.behaviour = TurtleBehaviour(turtle_obj, letter_obj)
            self.lastLetter = letter_obj
            self.index += 1
            return turtle_obj
        else:
            return None

    # Metodi per implementare l'iterator protocol
    def __iter__(self):
        return self

    def __next__(self):
        """Restituisce la prossima Turtle, oppure lancia StopIteration."""
        turtle_obj = self.getCurrentTurtle()
        if turtle_obj is None:
            raise StopIteration
        return turtle_obj

