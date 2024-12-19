import pybullet as p
import pybullet_data
import time
import math
import numpy as np

class RobileController:
    def __init__(self, robot_id):

        self.robot_id = robot_id
        
        # Joint indices
        self.steering_joints = [1, 5, 11, 15]  # Steering joints
        self.wheel_joints = [2, 3, 6, 7, 12, 13, 16, 17]  # Wheel rotation joints
        
        # Wheel configuration (adjust based on your robot's dimensions)
        self.wheel_radius = 0.1
        self.wheel_distance_from_center = 0.5  # Distance from robot center to wheel
        
        # Initialize steering angles
        self.reset_steering()
    
    def reset_steering(self):
        """Reset all steering angles to zero"""
        for joint in self.steering_joints:
            p.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=joint,
                controlMode=p.POSITION_CONTROL,
                targetPosition=0
            )
    
    def set_steering_angles(self, angles):
        """
        Set individual steering angles for each corner
        angles: list of 4 angles in radians
        """
        for joint, angle in zip(self.steering_joints, angles):
            # Normalize angle to [-pi, pi]
            while angle > math.pi:
                angle -= 2 * math.pi
            while angle < -math.pi:
                angle += 2 * math.pi
                
            p.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=joint,
                controlMode=p.POSITION_CONTROL,
                targetPosition=angle
            )
    
    def set_wheel_velocities(self, velocities):
        """
        Set individual wheel velocities
        velocities: list of 8 velocities (one for each wheel)
        """
        for joint, velocity in zip(self.wheel_joints, velocities):
            p.setJointMotorControl2(
                bodyIndex=self.robot_id,
                jointIndex=joint,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=velocity
            )
    
    def move_omnidirectional(self, vx, vy, omega):
        """
        Move the robot omnidirectionally
        vx: forward velocity (m/s)
        vy: sideways velocity (m/s) - positive is left
        omega: angular velocity (rad/s) - positive is counterclockwise
        """
        # Calculate velocity vector direction
        movement_angle = math.atan2(vy, vx)
        speed = math.sqrt(vx**2 + vy**2)
        
        # Calculate steering angles and wheel velocities for each corner
        steering_angles = []
        wheel_velocities = []
        
        # Corner positions (FL, FR, RL, RR)
        corners = [
            ( 1,  1),  # Front Left
            ( 1, -1),  # Front Right
            (-1,  1),  # Rear Left
            (-1, -1)   # Rear Right
        ]
        
        for x, y in corners:
            # Calculate position vector for this corner
            corner_x = x * self.wheel_distance_from_center
            corner_y = y * self.wheel_distance_from_center
            
            # Calculate velocity components from rotation
            rot_vx = -omega * corner_y
            rot_vy = omega * corner_x
            
            # Combine translation and rotation velocities
            total_vx = vx + rot_vx
            total_vy = vy + rot_vy
            
            # Calculate steering angle for this corner
            steering_angle = math.atan2(total_vy, total_vx)
            steering_angles.append(steering_angle)
            
            # Calculate wheel speed
            wheel_speed = math.sqrt(total_vx**2 + total_vy**2) / self.wheel_radius
            # Add two identical speeds for the two wheels at this corner
            wheel_velocities.extend([wheel_speed, wheel_speed])
        
        # Apply steering angles and wheel velocities
        self.set_steering_angles(steering_angles)
        self.set_wheel_velocities(wheel_velocities)
    

