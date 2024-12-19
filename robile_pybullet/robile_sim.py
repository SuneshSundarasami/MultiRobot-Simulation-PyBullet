import os
import time
import pybullet as p
import pybullet_data
import math
import rclpy
import numpy as np
from .laser_scanner import LaserScanner
from rclpy.node import Node

class RobotSimulationNode(Node):
    def __init__(self):
        super().__init__('robot_simulation_node')

        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        self.load_environment()
        
        self.laser_scanner = LaserScanner(self.robot_id, self, laser_pointers_freq=1)
        p.setPhysicsEngineParameter(numSolverIterations=1000)
        p.setTimeStep(1. / 120.)
        
        self.timer = self.create_timer(1. / 20., self.simulation_step)

        self.laser_freq_count=0

    def load_environment(self):
        current_directory = os.getcwd()
        urdf_base_path = os.path.join(current_directory, "src","Simulation_pybullet", "robile_pybullet", "Worlds")
        
        ground_urdf_path = os.path.join(urdf_base_path, "closed_environment.urdf")
        groundId = p.loadURDF(ground_urdf_path)
        p.setGravity(0, 0, -9.8)
        p.changeDynamics(groundId, -1, 
                lateralFriction=1.0,
                spinningFriction=0.1,
                rollingFriction=0.1)

        robot_urdf_path = os.path.join(current_directory, "src", "Simulation_pybullet", "robile_pybullet", "robile3_config.urdf")
        self.robot_id = p.loadURDF(robot_urdf_path, basePosition=[0, 0, 0.05],useFixedBase=False)
        p.setPhysicsEngineParameter(enableConeFriction=1)
        p.setPhysicsEngineParameter(contactBreakingThreshold=0.001)
        self.add_wall()

    def add_wall(self):
        wall_half_extents = [3, 0.1, 1.0]  # Wall dimensions: 1.0 length, 0.1 width, 0.5 height

        # Create the first wall (vertical part of the L shape)
        wall1_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=wall_half_extents)
        wall1_position = [5, 0, 1]  # Position of the vertical wall
        wall1_orientation = p.getQuaternionFromEuler([0, 0, math.pi / 2])  # Rotate 90 degrees on the Z-axis
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall1_collision_shape, 
                        basePosition=wall1_position, baseOrientation=wall1_orientation)

        # Create the second wall (horizontal part of the L shape)
        wall2_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=wall_half_extents)
        wall2_position = [5, 0.5, 1]  # Position of the horizontal wall (shifted along the Y-axis)
        wall2_orientation = p.getQuaternionFromEuler([0, 0, 0])  # No rotation needed
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall2_collision_shape, 
                        basePosition=wall2_position, baseOrientation=wall2_orientation)

    def simulation_step(self):
        
        for joint_index in range(p.getNumJoints(self.robot_id)):
            p.changeDynamics(self.robot_id, joint_index,
                        lateralFriction=1.0,
                        spinningFriction=0.1,
                        rollingFriction=0.1)
        self.move_robot()
        p.stepSimulation()
        for joint_index in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, joint_index)
            joint_name = joint_info[1].decode('utf-8')
            if "wheel" in joint_name.lower():
                state = p.getJointState(self.robot_id, joint_index)
                print(f"{joint_name} velocity: {state[1]}")

        # Get the robot's current velocity
        #linear_velocity, angular_velocity = p.getBaseVelocity(self.robot_id)

    # Log the velocities
        #self.get_logger().info(f"Linear Velocity: {linear_velocity}, Angular Velocity: {angular_velocity}")

        self.laser_freq_count+=1
        
        laser_data = self.laser_scanner.simulate(self.laser_freq_count)
        self.laser_scanner.publish_laser_scan(laser_data)
        
        self.get_logger().info(f"Laser data : {laser_data}")


    def calculate_wheel_velocities(self, linear_velocity, angular_velocity):
        # Robot parameters
        wheel_radius = 0.0515  # Replace with your wheel radius
        wheel_separation = 0.105  # Replace with distance between left and right wheels
        
        # Calculate left and right wheel velocities
        left_velocity = (linear_velocity - (wheel_separation * angular_velocity) / 2.0) / wheel_radius
        right_velocity = (linear_velocity + (wheel_separation * angular_velocity) / 2.0) / wheel_radius
        
        return left_velocity, right_velocity


    def move_robot(self):
        # Reset all joint motors first
        num_joints = p.getNumJoints(self.robot_id)
        for joint_index in range(num_joints):
            p.setJointMotorControl2(self.robot_id, joint_index, p.VELOCITY_CONTROL, force=0)

        # Set wheel velocities (different for each wheel)
        linear_velocity = 3.0  # m/s
        angular_velocity = 30.0  # rad/s for straight motion
        max_force = 1000

        left_velocity, right_velocity = self.calculate_wheel_velocities(linear_velocity, angular_velocity)
        print("left_velocity ",left_velocity, "right_velocity ", right_velocity)
        # Dictionary of wheel pairs
        wheel_pairs = {
            'robile_1': ['robile_1_drive_left_hub_wheel_joint', 'robile_1_drive_right_hub_wheel_joint'],
            'robile_2': ['robile_2_drive_left_hub_wheel_joint', 'robile_2_drive_right_hub_wheel_joint'],
            'robile_5': ['robile_5_drive_left_hub_wheel_joint', 'robile_5_drive_right_hub_wheel_joint'],
            'robile_6': ['robile_6_drive_left_hub_wheel_joint', 'robile_6_drive_right_hub_wheel_joint']
        }

        # Set velocities for all wheels
        num_joints = p.getNumJoints(self.robot_id)
        for joint_index in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, joint_index)
            joint_name = joint_info[1].decode('utf-8')
            
            for wheel_pair in wheel_pairs.values():
                if joint_name == wheel_pair[0]:  # Left wheel
                    p.setJointMotorControl2(
                        self.robot_id,
                        joint_index,
                        controlMode=p.VELOCITY_CONTROL,
                        targetVelocity=left_velocity,
                        force=max_force
                    )
                elif joint_name == wheel_pair[1]:  # Right wheel
                    p.setJointMotorControl2(
                        self.robot_id,
                        joint_index,
                        controlMode=p.VELOCITY_CONTROL,
                        targetVelocity=right_velocity,
                        force=max_force
                    )


def main(args=None):
    rclpy.init(args=args)

    robot_sim_node = RobotSimulationNode()
    
    try:
        rclpy.spin(robot_sim_node)
    except KeyboardInterrupt:
        pass
    finally:
        robot_sim_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
