import os
import time
import pybullet as p
import pybullet_data
import math
import rclpy
from laser_scanner import LaserScanner
from rclpy.node import Node

class RobotSimulationNode(Node):
    def __init__(self):
        super().__init__('robot_simulation_node')

        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        self.load_environment()
        
        self.laser_scanner = LaserScanner(self.robot_id, self, laser_pointers_freq=1)
        
        p.setTimeStep(1. / 120.)
        
        self.timer = self.create_timer(1. / 20., self.simulation_step)

        self.laser_freq_count=0

    def load_environment(self):
        current_directory = os.getcwd()
        urdf_base_path = os.path.join(current_directory, "robile_pybullet", "Worlds")
        
        ground_urdf_path = os.path.join(urdf_base_path, "closed_environment.urdf")
        p.loadURDF(ground_urdf_path)
        p.setGravity(0, 0, -9.8)
        
        robot_urdf_path = os.path.join(current_directory, "robile_pybullet", "robile3_config.urdf")
        self.robot_id = p.loadURDF(robot_urdf_path, basePosition=[0, 0, 0.1])

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
        p.stepSimulation()

        # self.move_robot()

        # Get the robot's current velocity
        linear_velocity, angular_velocity = p.getBaseVelocity(self.robot_id)

    # Log the velocities
        self.get_logger().info(f"Linear Velocity: {linear_velocity}, Angular Velocity: {angular_velocity}")

        self.laser_freq_count+=1
        
        laser_data = self.laser_scanner.simulate(self.laser_freq_count)
        self.laser_scanner.publish_laser_scan(laser_data)
        
        self.get_logger().info(f"Laser data : {laser_data}")


    def move_robot(self):
        # Apply forward velocity and angular velocity
        linear_velocity = 2.0  # m/s
        angular_velocity = 5.0  # rad/s

        max_motor_force = 10000  # Maximum force for the motors

        # Actuator names as defined in the URDF
        right_wheel_motor = "robile_5_drive_right_hub_wheel_motor"
        left_wheel_motor = "robile_5_drive_left_hub_wheel_motor"

        num_joints = p.getNumJoints(self.robot_id)

        # Iterate through all joints to find actuators
        for joint_index in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, joint_index)
            joint_name = joint_info[1].decode('utf-8')  # Joint name

            # Check if the joint corresponds to the actuators defined in the URDF
            if joint_name == left_wheel_motor:
                self.get_logger().info(f"Controlling joint: {joint_name}")
                p.setJointMotorControl2(
                    self.robot_id,
                    joint_index,
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocity=linear_velocity,
                    force=max_motor_force
                )
            elif joint_name == right_wheel_motor:
                self.get_logger().info(f"Controlling joint: {joint_name}")
                p.setJointMotorControl2(
                    self.robot_id,
                    joint_index,
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocity=linear_velocity,
                    force=max_motor_force
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
