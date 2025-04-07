from dataclasses import dataclass
from turtlesim.msg import Pose  # Assumendo che tu usi turtlesim.msg.Pose

@dataclass
class Offender:
    name: str
    target_position: Pose = None  # inizialmente None, da aggiornare con la callback
