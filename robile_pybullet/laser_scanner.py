import math
import pybullet as p
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
# Created with the help of Chat-GPT

class LaserScanner:
    def __init__(self, robot_id, node, laser_pointers_freq=False, num_beams=90, 
                 max_range=10, laser_height=0.1, laser_front_distance=0.4, beam_skip_rate=10):
        # PyBullet-specific parameters
        self.robot_id = robot_id
        self.num_beams = num_beams
        self.max_range = float(max_range)  # Ensure max_range is a float
        self.laser_pointers_freq = laser_pointers_freq
        self.laser_height = laser_height
        self.laser_front_distance = laser_front_distance
        self.beam_skip_rate = beam_skip_rate  # Control the number of debug lines by skipping beams
        
        # ROS2 node for publishing
        self.node = node
        
        # Create ROS2 publisher
        self.laser_publisher = self.node.create_publisher(LaserScan, '/scan', 10)
        
        # Precompute laser angles
        self.laser_angle_step = math.pi / self.num_beams
        self.laser_angles = [-math.pi / 2 + i * self.laser_angle_step for i in range(self.num_beams)]

        self.freq_count = 0
        self.debug_lines = {}  # Dictionary to store debug line IDs for updating positions

    def calculate_distance(self, point1, point2):
        return math.sqrt(
            (point1[0] - point2[0]) ** 2 +
            (point1[1] - point2[1]) ** 2 +
            (point1[2] - point2[2]) ** 2
        )
    
    def simulate(self, freq_count):
        print(f"---------------Frequency count:{freq_count}------------------------------------")
        # Get robot position and orientation
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
            [
                front_position[0] + self.max_range * math.cos(angle + euler_orientation[2]),
                front_position[1] + self.max_range * math.sin(angle + euler_orientation[2]),
                front_position[2]
            ]
            for angle in self.laser_angles
        ]
        
        # Batch raycasting for laser beams
        ray_results = p.rayTestBatch([front_position] * self.num_beams, laser_endpoints)
        
        laser_readings = []
        self.freq_count += 1


        for i in range(0, self.num_beams): 
            if ray_results[i][0] != -1:  # Collision detected
                    hit_position = ray_results[i][3]
                    distance = float(min(self.calculate_distance(front_position, hit_position), self.max_range))
                    laser_readings.append(distance)
            else:
                    laser_readings.append(float("inf"))


        # Update debug lines only on a specified frequency and for reduced beams
        if freq_count % self.laser_pointers_freq == 0:
            for i in range(0, self.num_beams, self.beam_skip_rate):  # Skip beams to reduce debug lines
                if ray_results[i][0] != -1:  # Collision detected

                    if i in self.debug_lines:
                        p.removeUserDebugItem(self.debug_lines[i])  # Remove the old line
                    line_id = p.addUserDebugLine(front_position, hit_position, [1, 0, 0], lineWidth=2)
                    self.debug_lines[i] = line_id  # Store the new line ID
                else:

                    if i in self.debug_lines:
                        p.removeUserDebugItem(self.debug_lines[i])  # Remove the old line
                    line_id = p.addUserDebugLine(front_position, laser_endpoints[i], [0, 1, 0], lineWidth=2)
                    self.debug_lines[i] = line_id  # Store the new line ID
        
        return laser_readings
    
    def publish_laser_scan(self, laser_readings):
        """Publish LaserScan message with validated data"""
        # Validate and convert laser readings
        validated_ranges = []
        for reading in laser_readings:
            # Ensure reading is a float and within valid range
            reading = float(reading)
            if reading < 0:
                reading = float(self.max_range)
            elif reading > self.max_range:
                reading = float(self.max_range)
            validated_ranges.append(reading)
        
        # Create LaserScan message
        scan_msg = LaserScan()
        
        # Header
        scan_msg.header.frame_id = 'laser_frame'
        scan_msg.header.stamp = self.node.get_clock().now().to_msg()
        
        # Scan configuration
        scan_msg.angle_min = -math.pi / 2
        scan_msg.angle_max = math.pi / 2
        scan_msg.angle_increment = self.laser_angle_step
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 0.1
        scan_msg.range_min = 0.1
        scan_msg.range_max = float(self.max_range)
        
        # Ranges with type and range validation
        scan_msg.ranges = validated_ranges
        
        # Intensities (optional, set to zero)
        scan_msg.intensities = [0.0] * self.num_beams
        
        # Publish the message
        self.laser_publisher.publish(scan_msg)
