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
        
        self.laser_scanner = LaserScanner(self.robot_id, self, laser_pointers=False)
        
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
        wall_half_extents = [0.5, 0.1, 1.0]
        wall_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=wall_half_extents)
        wall_position = [5, 0, 1]
        wall_orientation = p.getQuaternionFromEuler([0, 0, math.pi / 2])
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, 
                           basePosition=wall_position, baseOrientation=wall_orientation)

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
