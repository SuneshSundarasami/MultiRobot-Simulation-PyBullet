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

        linear_velocity, angular_velocity=1,-1
        self.move_robot_with_front_wheels(self.robot_id, linear_velocity, angular_velocity, 
                                      self.wheel_radius, self.wheel_base, self.track_width)

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
    def move_robot_with_front_wheels(self, robot_id, linear_velocity, angular_velocity, wheel_radius, wheel_base, track_width):
        if angular_velocity != 0:
            turning_radius = linear_velocity / angular_velocity
            outer_radius = turning_radius + track_width / 2
            inner_radius = turning_radius - track_width / 2
            
            outer_velocity = outer_radius * angular_velocity
            inner_velocity = inner_radius * angular_velocity
        else:
            outer_velocity = inner_velocity = linear_velocity

        for group in [{'left': 2, 'right': 3}, {'left': 6, 'right': 7}]:
            p.setJointMotorControl2(
                bodyUniqueId=robot_id,
                jointIndex=group['left'],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=inner_velocity / wheel_radius if angular_velocity > 0 else outer_velocity / wheel_radius,
                force=100,
                positionGain=0.1,
                velocityGain=0.1
            )

            p.setJointMotorControl2(
                bodyUniqueId=robot_id,
                jointIndex=group['right'],
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=outer_velocity / wheel_radius if angular_velocity > 0 else inner_velocity / wheel_radius,
                force=100,
                positionGain=0.1,
                velocityGain=0.1
            )

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
