import math
import pybullet as p

class LaserScanner:
    def __init__(self, robot_id,laser_pointers=False, num_beams=90, max_range=10, laser_height=0.1, laser_front_distance=0.4):
        self.robot_id = robot_id
        self.num_beams = num_beams
        self.max_range = max_range
        self.laser_pointers=laser_pointers
        self.laser_height = laser_height
        self.laser_front_distance = laser_front_distance
        self.laser_angle_step = math.pi / self.num_beams  # Step size for angle (evenly spaced, 180 degrees total)
        
        # Precompute the laser angles
        self.laser_angles = [-math.pi / 2 + i * self.laser_angle_step for i in range(self.num_beams)]

    def calculate_distance(self, point1, point2):
        return math.sqrt((point1[0] - point2[0]) ** 2 +
                         (point1[1] - point2[1]) ** 2 +
                         (point1[2] - point2[2]) ** 2)

    def simulate(self):
        laser_readings = []

        # Get robot position and orientation (call once for both position and orientation)
        robot_position, robot_orientation = p.getBasePositionAndOrientation(self.robot_id)
        euler_orientation = p.getEulerFromQuaternion(robot_orientation)
        
        # Calculate the front position of the robot
        front_position = [
            robot_position[0] + self.laser_front_distance * math.cos(euler_orientation[2]),
            robot_position[1] + self.laser_front_distance * math.sin(euler_orientation[2]),
            robot_position[2] + self.laser_height
        ]

        # Precompute laser beam endpoints
        laser_endpoints = [
            [front_position[0] + self.max_range * math.cos(angle),
             front_position[1] + self.max_range * math.sin(angle),
             front_position[2]] 
            for angle in self.laser_angles
        ]
        
        # Batch raycasting for laser beams
        ray_results = p.rayTestBatch([front_position] * self.num_beams, laser_endpoints)

        for i in range(self.num_beams):
            if ray_results[i][0] != -1:  # Collision detected
                hit_position = ray_results[i][3]
                distance = self.calculate_distance(front_position, hit_position)
                laser_readings.append(distance)
                if self.laser_pointers:
                    p.addUserDebugLine(front_position, hit_position, [1, 0, 0], lineWidth=2)  # Draw laser in red
            else:
                laser_readings.append(self.max_range)  # Max range if no collision
                if self.laser_pointers:
                    p.addUserDebugLine(front_position, laser_endpoints[i], [0, 1, 0], lineWidth=2)  # Draw laser in green
        
        return laser_readings
