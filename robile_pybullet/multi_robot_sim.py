import pybullet as p
import pybullet_data
import time
import math
import numpy as np
from typing import List, Tuple

class MultiRobileController:
    def __init__(self, urdf_path: str, num_followers: int, formation_config: List[Tuple[float, float]]):
        """
        Initialize controller for multiple Robile robots
        Args:
            urdf_path: Path to robot URDF file
            num_followers: Number of follower robots
            formation_config: List of (x,y) positions relative to leader for each follower
        """
        # Connect to PyBullet
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # Load ground plane
        p.loadURDF("plane.urdf")
        
        # Load leader and follower robots
        self.leader_id = p.loadURDF(urdf_path, [0, 0, 0.1])
        self.follower_ids = []
        
        # Load follower robots in formation
        for i in range(num_followers):
            x, y = formation_config[i]
            follower_id = p.loadURDF(urdf_path, [x, y, 0.1])
            self.follower_ids.append(follower_id)
        
        # Joint configuration
        self.steering_joints = [1, 5, 11, 15]  # Steering joints
        self.wheel_joints = [2, 3, 6, 7, 12, 13, 16, 17]  # Wheel rotation joints
        
        # Robot configuration
        self.wheel_radius = 0.1
        self.wheel_distance_from_center = 0.5
        
        # Initialize PID controllers for each follower
        self.pid_controllers = [
            SimpleDistanceController(kp=1.0, ki=0.1, kd=0.1) 
            for _ in range(num_followers)
        ]
        
        # Store desired formation positions
        self.formation_config = formation_config
        
        # Initialize steering
        self.reset_steering_all_robots()

    def reset_steering_all_robots(self):
        """Reset steering angles for all robots"""
        # Reset leader
        self.reset_steering_single_robot(self.leader_id)
        
        # Reset all followers
        for follower_id in self.follower_ids:
            self.reset_steering_single_robot(follower_id)
    
    def reset_steering_single_robot(self, robot_id: int):
        """Reset steering angles for a single robot"""
        for joint in self.steering_joints:
            p.setJointMotorControl2(
                bodyIndex=robot_id,
                jointIndex=joint,
                controlMode=p.POSITION_CONTROL,
                targetPosition=0
            )

    def set_steering_angles(self, angles: List[float], robot_id: int):
        """Set steering angles for a single robot"""
        for joint, angle in zip(self.steering_joints, angles):
            # Normalize angle to [-pi, pi]
            angle = (angle + math.pi) % (2 * math.pi) - math.pi
            
            p.setJointMotorControl2(
                bodyIndex=robot_id,
                jointIndex=joint,
                controlMode=p.POSITION_CONTROL,
                targetPosition=angle
            )
    
    def set_wheel_velocities(self, velocities: List[float], robot_id: int):
        """Set wheel velocities for a single robot"""
        for joint, velocity in zip(self.wheel_joints, velocities):
            p.setJointMotorControl2(
                bodyIndex=robot_id,
                jointIndex=joint,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=velocity
            )
    
    def move_omnidirectional(self, vx: float, vy: float, omega: float, robot_id: int):
        """Move a single robot omnidirectionally"""
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
            corner_x = x * self.wheel_distance_from_center
            corner_y = y * self.wheel_distance_from_center
            
            # Calculate velocity components
            rot_vx = -omega * corner_y
            rot_vy = omega * corner_x
            total_vx = vx + rot_vx
            total_vy = vy + rot_vy
            
            # Calculate steering angle and wheel speed
            steering_angle = math.atan2(total_vy, total_vx)
            wheel_speed = math.sqrt(total_vx**2 + total_vy**2) / self.wheel_radius
            
            steering_angles.append(steering_angle)
            wheel_velocities.extend([wheel_speed, wheel_speed])
        
        self.set_steering_angles(steering_angles, robot_id)
        self.set_wheel_velocities(wheel_velocities, robot_id)

    def update_followers(self):
        """Update all follower robots to maintain formation"""
        leader_pos, leader_orient = p.getBasePositionAndOrientation(self.leader_id)
        leader_euler = p.getEulerFromQuaternion(leader_orient)
        
        for idx, (follower_id, pid_controller) in enumerate(zip(self.follower_ids, self.pid_controllers)):
            # Get current follower position and orientation
            follower_pos, follower_orient = p.getBasePositionAndOrientation(follower_id)
            follower_euler = p.getEulerFromQuaternion(follower_orient)
            
            # Calculate desired position based on formation configuration
            desired_x, desired_y = self.formation_config[idx]
            
            # Transform desired position based on leader's orientation
            leader_angle = leader_euler[2]  # yaw angle
            desired_world_x = leader_pos[0] + (desired_x * math.cos(leader_angle) - desired_y * math.sin(leader_angle))
            desired_world_y = leader_pos[1] + (desired_x * math.sin(leader_angle) + desired_y * math.cos(leader_angle))
            desired_pos = (desired_world_x, desired_world_y, leader_pos[2])
            
            # Calculate control inputs
            linear_vel = pid_controller.maintain_distance(desired_pos, follower_pos)
            angular_vel = self.compute_angular_velocity(leader_euler[2], follower_euler[2])
            
            # Apply control
            self.move_omnidirectional(linear_vel[0], linear_vel[1], angular_vel, follower_id)

    def compute_angular_velocity(self, target_yaw: float, current_yaw: float, kp: float = 1.0) -> float:
        """Compute angular velocity to align with target orientation"""
        angular_difference = target_yaw - current_yaw
        angular_difference = (angular_difference + math.pi) % (2 * math.pi) - math.pi
        
        if abs(angular_difference) < 0.01:  # tolerance
            return 0.0
            
        omega = kp * angular_difference
        return np.clip(omega, -1.0, 1.0)  # limit maximum angular velocity

    def run_simulation(self):
        """Run one simulation step"""
        p.stepSimulation()
        time.sleep(1./240.)

class SimpleDistanceController:
    def __init__(self, kp=1.0, ki=0.1, kd=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = np.zeros(3)
        self.dt = 1/240.0
        self.integral_limit = 10.0

    def maintain_distance(self, target_pos, current_pos):
        """Calculate control velocity to reach target position"""
        error_vector = np.array(target_pos) - np.array(current_pos)
        error_distance = np.linalg.norm(error_vector)
        
        if error_distance < 1e-6:
            return np.zeros(3)
            
        direction = error_vector / error_distance
        
        # PID control
        proportional = self.kp * error_vector
        self.integral += error_vector * self.dt
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)
        integral_term = self.ki * self.integral
        derivative = self.kd * (error_vector - self.prev_error) / self.dt
        
        self.prev_error = error_vector
        
        velocity = proportional + integral_term + derivative
        return velocity

def main():
    urdf_path = "/home/sunesh/ros2_ws/src/robile_pybullet/robile_pybullet/robile3_config.urdf"
    
    # Define formation configuration: positions relative to leader
    # Example V formation
    formation_config = [
        (-1.0, -1.0),  # First follower position
        (-1.0, 1.0),   # Second follower position
        (-2.0, -2.0),  # Third follower position
        (-2.0, 2.0),   # Fourth follower position
    ]
    
    # Initialize controller with 4 followers
    controller = MultiRobileController(urdf_path, num_followers=1, formation_config=formation_config)
    
    try:
        while True:
            # Example movement sequence for leader
            # Move forward
            for _ in range(1200):
                controller.move_omnidirectional(0.5, 0, 0, controller.leader_id)
                controller.update_followers()
                controller.run_simulation()
            
            # Move sideways
            for _ in range(1200):
                controller.move_omnidirectional(0, 0.5, 0, controller.leader_id)
                controller.update_followers()
                controller.run_simulation()
            
            # Rotate in place
            for _ in range(1200):
                controller.move_omnidirectional(0, 0, 0.5, controller.leader_id)
                controller.update_followers()
                controller.run_simulation()
            
            # Complex motion
            for _ in range(1200):
                controller.move_omnidirectional(0.3, 0.2, 0.1, controller.leader_id)
                controller.update_followers()
                controller.run_simulation()
                
    except KeyboardInterrupt:
        p.disconnect()

if __name__ == "__main__":
    main()