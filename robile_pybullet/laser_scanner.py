import math
import pybullet as p

class LaserScanner:
    def __init__(self, robot_id, num_beams=90, max_range=10, laser_height=0.1, laser_front_distance=0.4):
        self.robot_id = robot_id
        self.num_beams = num_beams
        self.max_range = max_range
        self.laser_height = laser_height
        self.laser_front_distance = laser_front_distance
        self.laser_angle_step = math.pi / self.num_beams  # Step size for angle (evenly spaced, 180 degrees total)

    def calculate_distance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0]) ** 2 +
                         (point1[1] - point2[1]) ** 2 +
                         (point1[2] - point2[2]) ** 2)

    def simulate(self):
        laser_readings = []
        
        # Get robot position and orientation
        robot_position, robot_orientation = p.getBasePositionAndOrientation(self.robot_id)
        
        # Calculate the front position of the robot
        front_position = [
            robot_position[0] + self.laser_front_distance * math.cos(p.getEulerFromQuaternion(robot_orientation)[2]),
            robot_position[1] + self.laser_front_distance * math.sin(p.getEulerFromQuaternion(robot_orientation)[2]),
            robot_position[2] + self.laser_height
        ]
        
        # Simulate 180-degree laser scanner (from -90 to +90 degrees)
        for i in range(self.num_beams):
            # Calculate the angle for this laser beam
            angle = -math.pi / 2 + i * self.laser_angle_step
            
            # Calculate laser beam endpoint
            laser_x = front_position[0] + self.max_range * math.cos(angle)
            laser_y = front_position[1] + self.max_range * math.sin(angle)
            laser_z = front_position[2]  # Laser height remains the same
            
            # Raycast to simulate the laser scan
            ray_result = p.rayTest(front_position, [laser_x, laser_y, laser_z])
            
            if ray_result[0][0] != -1:  # Collision detected
                hit_position = ray_result[0][3]
                distance = self.calculate_distance(front_position, hit_position)
                laser_readings.append(distance)
                p.addUserDebugLine(front_position, hit_position, [1, 0, 0], lineWidth=2)  # Draw laser in red
            else:
                laser_readings.append(self.max_range)  # Max range if no collision
                p.addUserDebugLine(front_position, [laser_x, laser_y, laser_z], [0, 1, 0], lineWidth=2)  # Draw laser in green
        
        return laser_readings
