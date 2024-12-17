import pybullet as p
import time
import pybullet_data

# Start the simulation
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load ground plane and set gravity
p.loadURDF("plane.urdf")
p.setGravity(0, 0, -9.8)

# Load the robile URDF file
robot_id = p.loadURDF(
    "/home/sunesh/ros2_ws/src/robile_pybullet/robile_pybullet/robile.urdf",
    basePosition=[0, 0, 0.1]
)


wheel_joints = [12, 13, 16, 17]  

print("Spinning the wheels...")
try:
    while True:
        for joint_index in wheel_joints:
            p.setJointMotorControl2(
                bodyUniqueId=robot_id,
                jointIndex=joint_index,
                controlMode=p.VELOCITY_CONTROL,
                targetVelocity=10.0,  # Set a positive velocity
                force=50.0  # Apply torque
            )
        p.stepSimulation()
        time.sleep(1.0 / 240.0)  # Simulate at 240 Hz
except KeyboardInterrupt:
    print("Simulation stopped.")

p.disconnect()
