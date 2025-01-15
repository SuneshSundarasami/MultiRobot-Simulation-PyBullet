import pybullet as p
import pybullet_data
import time
import math
import numpy as np

class RobileController:
    def __init__(self, urdf_path):
        # Connect to PyBullet
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        
        # Load ground plane and robot
        p.loadURDF("plane.urdf")
        self.robot_id = p.loadURDF(urdf_path, [0, 0, 0.1])
        self.robot2_id = p.loadURDF(urdf_path,[0, -2, 0.1])
        
        # Joint indices
        self.steering_joints = [1, 5, 11, 15]  # Steering joints
        self.wheel_joints = [2, 3, 6, 7, 12, 13, 16, 17]  # Wheel rotation joints
        
        # Wheel configuration (adjust based on your robot's dimensions)
        self.wheel_radius = 0.1
        self.wheel_distance_from_center = 0.5  # Distance from robot center to wheel
        self.pid = SimpleDistanceController(kp=1.0, ki=0.1, kd=0.1)

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

        for joint in self.steering_joints:
            p.setJointMotorControl2(
                bodyIndex=self.robot2_id,
                jointIndex=joint,
                controlMode=p.POSITION_CONTROL,
                targetPosition=0
            )
    

    def set_steering_angles(self, angles, robot_id):
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
                bodyIndex=robot_id,
                jointIndex=joint,
                controlMode=p.POSITION_CONTROL,
                targetPosition=angle
            )
    
    def set_wheel_velocities(self, velocities,robot_id):
        """
        Set individual wheel velocities
        velocities: list of 8 velocities (one for each wheel)
        """
        for joint, velocity in zip(self.wheel_joints, velocities):
            p.setJointMotorControl2(
                bodyIndex=robot_id,
                jointIndex=joint,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=velocity
            )
    
    def move_omnidirectional(self, vx, vy, omega,robot_id):
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
        self.set_steering_angles(steering_angles,robot_id)
        self.set_wheel_velocities(wheel_velocities,robot_id)
    
    def run_simulation(self):
        """Run the simulation step"""
        p.stepSimulation()
        time.sleep(1./240.)

    def compute_position_velocity_for_robot_2(self):
        leader_pos,leader_orient = p.getBasePositionAndOrientation(self.robot_id)
        follower_pos,follower_orient = p.getBasePositionAndOrientation(self.robot2_id)
        """# Draw connection line
        visualize_points(leader_pos, follower_pos)

        # Mark positions
        draw_position_marker(leader_pos)
        draw_position_marker(follower_pos)

        # Add labels
        add_point_label(leader_pos, "Leader")
        add_point_label(follower_pos, "Follower")"""
        l_euler_angles = p.getEulerFromQuaternion(leader_orient)
        f_euler_angles = p.getEulerFromQuaternion(follower_orient)
        print(leader_pos,follower_pos, l_euler_angles,f_euler_angles)
        desired_linear_vel = self.pid.maintain_distance(leader_pos,follower_pos)
        # taking the yaw values of current orientation of leader and follower 
        angle_velocity = self.compute_angular_velocity(l_euler_angles[2],f_euler_angles[2])
        print("angular change ->", angle_velocity)
        print("desired linear velocity -> ", desired_linear_vel)
        self.move_omnidirectional(desired_linear_vel[0],desired_linear_vel[1],angle_velocity,self.robot2_id)
        self.run_simulation()
        #p.removeAllUserDebugItems()

    def compute_angular_velocity(self, leader_yaw, follower_yaw, kp=1.0):
        # Calculate angular difference
        angular_difference = leader_yaw - follower_yaw
        
        # Normalize angular difference to [-π, π]
        angular_difference = (angular_difference + math.pi) % (2 * math.pi) - math.pi
        
        # Add tolerance for small errors
        tolerance = 0.01  # radians

        if abs(angular_difference) < tolerance:
            return 0.0
            
        # Compute angular velocity using proportional control
        omega = kp * angular_difference
        
        # Add velocity limits to prevent too fast rotation
        max_omega = 1.0  # maximum allowed angular velocity
        omega = np.clip(omega, -max_omega, max_omega)
        
        return omega


def visualize_points(point1, point2):
    # Draw a line between two points
        p.addUserDebugLine(
        point1,  # Start point [x,y,z]
        point2,  # End point [x,y,z]
        lineColorRGB=[1, 0, 0],  # Red color
        lineWidth=3.0
    )
        
def draw_position_marker(position):
    # Draw axes at the point
    length = 0.2  # Length of axes
    p.addUserDebugLine(position, [position[0]+length, position[1], position[2]], [1,0,0])  # X axis (red)
    p.addUserDebugLine(position, [position[0], position[1]+length, position[2]], [0,1,0])  # Y axis (green)
    p.addUserDebugLine(position, [position[0], position[1], position[2]+length], [0,0,1])  # Z axis (blue)

def add_point_label(position, text):
    p.addUserDebugText(
        text,
        position,
        textColorRGB=[0, 0, 0],
        textSize=1.5
    )

class SimpleDistanceController:
    # uses PID controller to give the velocity needed to maintain the desired distance between two robots
    def __init__(self, kp=1.0, ki=0.1, kd=0.1):
        self.kp = kp # Proportional gain
        self.ki = ki # Integral gain
        self.kd = kd # Dervivative gain
        self.prev_error = 0.0
        self.integral = np.zeros(3) # For 3D position
        self.dt = 1/240.0 # Time step
        self.integral_limit = 10.0 # Add integral windup limit

    def maintain_distance(self,leader_pos, follower_pos, desired_distance=2.0):
        # calculate current distance
        current_distance = np.linalg.norm(np.array(leader_pos) - np.array(follower_pos))
        print("distance between the robots -> ", current_distance)
        # calculate error
        error = current_distance - desired_distance

        # Avoid division by zero
        if current_distance < 1e-6:
            direction = np.zeros(3)
        else:
        #calculate direction
            direction = (np.array(leader_pos) - np.array(follower_pos)) / current_distance

        #PID components
        proportional = self.kp * error * direction
        
        #Integral term with anti-windup
        self.integral += error*direction*self.dt
        self.integral = np.clip(self.integral, -self.integral_limit, self.integral_limit)
        integral_term = self.ki * self.integral

        #Derivative term
        derivative = self.kd * (error - self.prev_error) / self.dt * direction

        # Update previous error
        self.prev_error = error 

        # compute total velocity 
        velocity = proportional + integral_term + derivative
        
        return velocity

def main():
    # Initialize the controller with your URDF path
    urdf_path = "/home/saiga/ros2_ws/src/Simulation_pybullet/robile_pybullet/robile3_config.urdf"
    controller = RobileController(urdf_path)
    
    try:
        while True:
            # Example movement sequence demonstrating omnidirectional capabilities
            
            # Move forward
            print("Moving forward...")
            for _ in range(1200):  # 1 second at 240Hz
                controller.move_omnidirectional(0.5, 0, 0,controller.robot_id)
                controller.run_simulation()
                controller.compute_position_velocity_for_robot_2()
            
            # Move sideways (left)
            print("Moving left...")
            for _ in range(1200):
                controller.move_omnidirectional(0, 0.5, 0,controller.robot_id)
                controller.run_simulation()
                controller.compute_position_velocity_for_robot_2()
            
            # Move diagonally
            print("Moving diagonally...")
            for _ in range(1200):
                controller.move_omnidirectional(0.3, 0.3, 0,controller.robot_id)
                controller.run_simulation()
                controller.compute_position_velocity_for_robot_2()
            
            # Rotate in place
            print("Rotating in place...")
            for _ in range(1200):
                controller.move_omnidirectional(0, 0, -0.1,controller.robot_id)
                controller.run_simulation()
                controller.compute_position_velocity_for_robot_2()
            
            # Complex motion (moving + rotating)
            print("Complex motion...")
            for _ in range(1200):
                controller.move_omnidirectional(0.3, 0.2, 0.1,controller.robot_id)
                controller.run_simulation()
                controller.compute_position_velocity_for_robot_2()
                
    except KeyboardInterrupt:
        p.disconnect()

if __name__ == "__main__":
    main()
