import os
import time
import pybullet as p
import pybullet_data
import math
import rclpy
import numpy as np
from laser_scanner import LaserScanner
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Quaternion, Twist
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from robot_controller import RobileController

class RobotSimulationNode(Node):
    def __init__(self):
        super().__init__('robot_simulation_node')

        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        self.load_environment()
        
        self.laser_scanner = LaserScanner(self.robot_id, self, laser_pointers_freq=1)
        p.setPhysicsEngineParameter(numSolverIterations=1000)
        self.laser_scanner = LaserScanner(self.robot_id, self, laser_pointers_freq=100)
        
        p.setTimeStep(1. / 120.)
        
        self.timer = self.create_timer(1. / 20., self.simulation_step)

        self.laser_freq_count=0

        self.wheel_radius = 0.05
        self.wheel_base = 0.6
        self.track_width = 0.4

        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.laser_offset = [0.4, 0.0, 0.0]  

        self.controller = RobileController(self.robot_id)


    def load_environment(self):
        current_directory = os.getcwd()
        urdf_base_path = os.path.join(current_directory, "robile_pybullet", "Worlds")
        
        ground_urdf_path = os.path.join(urdf_base_path, "closed_environment.urdf")
        groundId = p.loadURDF(ground_urdf_path)
        p.setGravity(0, 0, -9.8)
        p.changeDynamics(groundId, -1, 
                lateralFriction=1.0,
                spinningFriction=0.1,
                rollingFriction=0.1)

        robot_urdf_path = os.path.join(current_directory, "robile_pybullet", "robile3_config.urdf")
        print(robot_urdf_path)
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
        # self.move_robot()
        p.stepSimulation()
        for joint_index in range(p.getNumJoints(self.robot_id)):
            joint_info = p.getJointInfo(self.robot_id, joint_index)
            joint_name = joint_info[1].decode('utf-8')
            if "wheel" in joint_name.lower():
                state = p.getJointState(self.robot_id, joint_index)
                print(f"{joint_name} velocity: {state[1]}")

        self.publish_odometry()


        self.controller.move_omnidirectional(1, 0, 1)
        # linear_velocity, angular_velocity=1,-1
        # self.move_robot_with_front_wheels(self.robot_id, linear_velocity, angular_velocity, 
        #                               self.wheel_radius, self.wheel_base, self.track_width)

        # Get the robot's current velocity
        #linear_velocity, angular_velocity = p.getBaseVelocity(self.robot_id)

    # Log the velocities
        #self.get_logger().info(f"Linear Velocity: {linear_velocity}, Angular Velocity: {angular_velocity}")

        self.laser_freq_count+=1
        
        laser_data = self.laser_scanner.simulate(self.laser_freq_count)
        self.laser_scanner.publish_laser_scan(laser_data)
        
        self.get_logger().info(f"Laser data : {laser_data}")





    def publish_odometry(self):
        # Retrieve the robot's current position and orientation
        position, orientation = p.getBasePositionAndOrientation(self.robot_id)

        # Create Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"

        # Fill pose information
        odom_msg.pose.pose.position.x = position[0]
        odom_msg.pose.pose.position.y = position[1]
        odom_msg.pose.pose.position.z = position[2]
        odom_msg.pose.pose.orientation = Quaternion(
            x=orientation[0],
            y=orientation[1],
            z=orientation[2],
            w=orientation[3],
        )

        # Fill velocity information
        linear_velocity, angular_velocity = p.getBaseVelocity(self.robot_id)
        odom_msg.twist.twist.linear.x = linear_velocity[0]
        odom_msg.twist.twist.linear.y = linear_velocity[1]
        odom_msg.twist.twist.linear.z = linear_velocity[2]
        odom_msg.twist.twist.angular.x = angular_velocity[0]
        odom_msg.twist.twist.angular.y = angular_velocity[1]
        odom_msg.twist.twist.angular.z = angular_velocity[2]

        # Publish odometry message
        self.odom_publisher.publish(odom_msg)

    def publish_laser_transform(self):

        position, orientation = p.getBasePositionAndOrientation(self.robot_id)


        laser_position = [
            position[0] + self.laser_offset[0] * math.cos(p.getEulerFromQuaternion(orientation)[2]),
            position[1] + self.laser_offset[0] * math.sin(p.getEulerFromQuaternion(orientation)[2]),
            position[2] + self.laser_offset[2]
        ]


        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_laser_front_link"


        transform.transform.translation.x = laser_position[0]
        transform.transform.translation.y = laser_position[1]
        transform.transform.translation.z = laser_position[2]
        transform.transform.rotation.x = orientation[0]
        transform.transform.rotation.y = orientation[1]
        transform.transform.rotation.z = orientation[2]
        transform.transform.rotation.w = orientation[3]


        self.tf_broadcaster.sendTransform(transform)




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
