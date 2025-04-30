

class SensorHelper:
    
    def __init__(self,model):
        self.model = model
        self.node = model.node
        self.sensor_range = 0.25 # 2 cm
        self.front_left_distance = None
        self.front_right_distance = None
        self.back_left_distance = None
        self.back_right_distance = None

    def printSensorData(self):
        self.node.get_logger().info(f"Front Left Distance: {self.front_left_distance}")
        self.node.get_logger().info(f"Front Right Distance: {self.front_right_distance}")
        self.node.get_logger().info(f"Back Left Distance: {self.back_left_distance}")
        self.node.get_logger().info(f"Back Right Distance: {self.back_right_distance}")

    def check_obstacle(self):
        is_obstacle_detected = False
        #self.node.get_logger().info("Checking for obstacles...")
        #self.printSensorData()
        if self.front_left_distance is not None and self.front_left_distance < self.sensor_range:
            self.node.get_logger().info(f"Obstacle detected in front left: {self.front_left_distance}")
            is_obstacle_detected =True
        if self.front_right_distance is not None and self.front_right_distance < self.sensor_range:
            self.node.get_logger().info(f"Obstacle detected in front right: {self.front_right_distance}")
            is_obstacle_detected =True
        if self.back_left_distance is not None and self.back_left_distance < self.sensor_range:
            self.node.get_logger().info(f"Obstacle detected in back left: {self.back_left_distance}")
            is_obstacle_detected =True
        if self.back_right_distance is not None and self.back_right_distance < self.sensor_range:
            self.node.get_logger().info(f"Obstacle detected in back right: {self.back_right_distance}")
            is_obstacle_detected =True
       
        return is_obstacle_detected
    def check_forward_obstacle(self):
        is_obstacle_detected = False
        #self.node.get_logger().info("Checking for obstacles...")
        #self.printSensorData()
        if self.front_left_distance is not None and self.front_left_distance < self.sensor_range:
            #self.node.get_logger().info(f"Obstacle detected in front left: {self.front_left_distance}")
            is_obstacle_detected =True
        if self.front_right_distance is not None and self.front_right_distance < self.sensor_range:
            #self.node.get_logger().info(f"Obstacle detected in front right: {self.front_right_distance}")
            is_obstacle_detected =True
        
       
        return is_obstacle_detected