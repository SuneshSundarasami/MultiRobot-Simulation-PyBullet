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
        
        self.laser_scanner = LaserScanner(self.robot_id, self, laser_pointers=True)
        
        p.setTimeStep(1. / 120.)
        
        self.timer = self.create_timer(1. / 20., self.simulation_step)

    def load_environment(self):
        current_directory = os.getcwd()
        urdf_base_path = os.path.join(current_directory, "robile_pybullet", "Worlds")
        
        ground_urdf_path = os.path.join(urdf_base_path, "closed_environment.urdf")
        p.loadURDF(ground_urdf_path)
        p.setGravity(0, 0, -9.8)
        
        robot_urdf_path = os.path.join(current_directory, "robile_pybullet", "robile.urdf")
        self.robot_id = p.loadURDF(robot_urdf_path, basePosition=[0, 0, 0.1])

        self.add_wall()

    def add_wall(self):
        # Define the parameters for the cylindrical wall
        radius = 0.5  # Radius of the circular wall
        height = 1.0  # Height of the circular wall
        
        # Create a collision shape for the cylinder
        wall_collision_shape = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)
        
        # Set the position and orientation for the circular wall
        wall_position = [5, 0, height / 2]  # Position at x=5, y=0, and height/2 to center it
        wall_orientation = p.getQuaternionFromEuler([0, 0, 0])  # No rotation
        
        # Create the multi-body (wall) with the defined properties
        wall_id = p.createMultiBody(
            baseMass=0, 
            baseCollisionShapeIndex=wall_collision_shape, 
            basePosition=wall_position, 
            baseOrientation=wall_orientation
        )
        
        # Set the color of the circular wall to blue
        p.changeVisualShape(wall_id, -1, rgbaColor=[0, 0, 1, 1])  # Blue color (R, G, B, A)


    def simulation_step(self):
        p.stepSimulation()
        
        laser_data = self.laser_scanner.simulate()
        self.laser_scanner.publish_laser_scan(laser_data)
        
        self.get_logger().info(f"Laser data : {laser_data}")

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
