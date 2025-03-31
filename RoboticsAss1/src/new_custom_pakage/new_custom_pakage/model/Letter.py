

# =============================================================================
#                           MODEL: Lettere del Alfabeto
# =============================================================================

import math

class Letter:
    """
    La classe Letter contiene la definizione dei segmenti per ciascuna lettera.
    I segmenti sono definiti in un dizionario statico.
    """
    LETTERS = {
        'U': [
            {"type": "rotate", "duration": 10, "linear_speed": 0.0, "angular_speed": -math.radians(90)},
            {"type": "move",   "duration": 30, "linear_speed": 1.0,  "angular_speed": 0.0},
            {"type": "rotate", "duration": 10, "linear_speed": 0.0, "angular_speed": math.radians(90)},
            {"type": "move",   "duration": 15, "linear_speed": 1.0,  "angular_speed": 0.0},
            {"type": "rotate", "duration": 10, "linear_speed": 0.0, "angular_speed": math.radians(90)},
            {"type": "move",   "duration": 30, "linear_speed": 1.0,  "angular_speed": 0.0},
        ],
        'S': [
            {"type": "move", "duration": 20, "linear_speed": 1.0, "angular_speed": 1.5708},
            {"type": "move", "duration": 20, "linear_speed": 1.0, "angular_speed": -1.5708},
        ],
        'I': [
            {"type": "rotate", "duration": 10, "linear_speed": 0.0, "angular_speed": -math.radians(90)},
            {"type": "move",   "duration": 20, "linear_speed": 1.0, "angular_speed": 0.0},
        ]
    }

    def __init__(self, name: str):
        self.name = name
        self.segments = Letter.LETTERS[name]
