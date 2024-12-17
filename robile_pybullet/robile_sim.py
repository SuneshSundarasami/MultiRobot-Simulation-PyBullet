import pybullet as p
import os
import time
import pybullet_data
from laser_scanner import LaserScanner
import math


current_directory = os.getcwd()
# Start the simulation in GUI mode
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load ground plane and set gravity
urdf_base_path = os.path.join(current_directory, "robile_pybullet", "Worlds")
ground_urdf_path = os.path.join(urdf_base_path, "closed_environment.urdf")
p.loadURDF(ground_urdf_path)
p.setGravity(0, 0, -9.8)

# Load the robot URDF file
robot_urdf_path = os.path.join(current_directory, "robile_pybullet", "robile.urdf")
robot_id = p.loadURDF(robot_urdf_path, basePosition=[0, 0, 0.1])

# Create LaserScanner object
laser_scanner = LaserScanner(robot_id)

# Add a wall
def add_wall():
    wall_half_extents = [0.5, 0.1, 1.0]  # Wall size
    wall_collision_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=wall_half_extents)
    wall_position = [5, 0, 1]  # Wall position
    wall_orientation = p.getQuaternionFromEuler([0, 0, math.pi/2])  
    p.createMultiBody(baseMass=0, baseCollisionShapeIndex=wall_collision_shape, basePosition=wall_position, baseOrientation=wall_orientation)

add_wall()

# Set time step for faster simulation
p.setTimeStep(1./120.)

# Main loop
while True:
    p.stepSimulation()
    laser_data = laser_scanner.simulate()
    print(laser_data)
    time.sleep(1./20.)
