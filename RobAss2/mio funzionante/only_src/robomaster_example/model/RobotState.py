
from enum import Enum
from robomaster_example.customController.SensorHelper import SensorHelper
from geometry_msgs.msg import Twist, Pose
from typing import TYPE_CHECKING


if TYPE_CHECKING:
    # risolizione importo circolare
    from robomaster_example.model.RoboMasterModel import RoboMasterModel

class RobotState(Enum):
    INITIALIZING = 0
    MOVE_FORWARD = 1
    ROTATE = 2
    STOP = 3

class RobotStateManager:
    def __init__(self,model: "RoboMasterModel"):
        self.model = model
        self.state = RobotState.INITIALIZING

    def set_state(self, state: RobotState):
        if self.state == state:
            #print(f"State already set to: {self.state.name}")
            return
        self.state = state
        print(f"Robot state changed to: {self.state.name}")

    def get_state(self) -> RobotState:
        return self.state

    def is_state(self, state: RobotState) -> bool:
        return self.state == state
    
    def update(self):
        if self.model.sensor.check_forward_obstacle():
            self.set_state(RobotState.ROTATE)
        else:
            self.set_state(RobotState.MOVE_FORWARD)

        if self.state == RobotState.MOVE_FORWARD or self.state == RobotState.INITIALIZING:
            self.move_forward()
        elif self.state == RobotState.ROTATE:
            self.rotate()
        elif self.state == RobotState.STOP:
            self.model.node.stop()
        else:
            print("Unknown state, no action taken.")
    
    def move_forward(self):
        self.set_state(RobotState.MOVE_FORWARD)
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.1  # Velocità costante in avanti

        # Aggiungiamo una componente angolare oscillante per alternare leggeri turni
        # Importa sin e pi se non sono già importati
        from math import sin, pi
        # Ottieni l'orario corrente (in secondi)
        current_time = self.model.node.get_clock().now().nanoseconds / 1e9

        # Parametri per l'oscillazione: regola frequenza e ampiezza come desideri
        frequency = 0.5  # Oscillazioni per secondo (0.5 -> una oscillazione ogni 2 secondi)
        amplitude = pi / 3      # 60 gradi in radianti (≈1.0472 rad)
        
        # Calcola una velocità angolare che oscilla nel tempo
        cmd_vel.angular.z = amplitude * sin(2 * pi * frequency * current_time)
        
        # Pubblica il comando
        self.model.subscription_controller.vel_publisher.publish(cmd_vel)

    def rotate(self):
        self.set_state(RobotState.ROTATE)
        cmd_vel = Twist() 
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 0.5
        self.model.subscription_controller.vel_publisher.publish(cmd_vel)


        
        