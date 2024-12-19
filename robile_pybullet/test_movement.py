import pybullet
import pybullet_data
import time
import math

def move_robot_with_drive_wheels(robot_id, linear_velocity, angular_velocity, wheel_radius, wheel_base, track_width):
    if angular_velocity != 0:
        turning_radius = linear_velocity / angular_velocity
        outer_radius = turning_radius + track_width / 2
        inner_radius = turning_radius - track_width / 2
        
        outer_velocity = outer_radius * angular_velocity
        inner_velocity = inner_radius * angular_velocity
    else:
        outer_velocity = inner_velocity = linear_velocity

    for group in [{'left': 2, 'right': 3}, {'left': 6, 'right': 7}]:
        # If angular velocity is negative (clockwise), the right wheel should go slower and the left wheel faster
        if angular_velocity < 0:
            inner_velocity_target = outer_velocity / wheel_radius
            outer_velocity_target = inner_velocity / wheel_radius
        else:  # If angular velocity is positive (counter-clockwise)
            inner_velocity_target = inner_velocity / wheel_radius
            outer_velocity_target = outer_velocity / wheel_radius
        
        pybullet.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=group['left'],
            controlMode=pybullet.VELOCITY_CONTROL,
            targetVelocity=inner_velocity_target,
            force=100,
            positionGain=0.1,
            velocityGain=0.1
        )

        pybullet.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=group['right'],
            controlMode=pybullet.VELOCITY_CONTROL,
            targetVelocity=outer_velocity_target,
            force=100,
            positionGain=0.1,
            velocityGain=0.1
        )
def get_wheel_velocities(robot_id):
    # List of joint indices for the wheels
    wheel_joint_indices = [2, 3, 6, 7]  # Example indices for left and right wheels
    
    wheel_velocities = {}
    for joint_index in wheel_joint_indices:
        joint_state = pybullet.getJointState(robot_id, joint_index)
        wheel_velocities[joint_index] = joint_state[1]  # joint_state[1] is the velocity

    return wheel_velocities


def run_robot_with_caster_simulation():
    pybullet.connect(pybullet.GUI)
    pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

    plane_id = pybullet.loadURDF("plane.urdf")
    robot_id = pybullet.loadURDF("/home/sunesh/ros2_ws/src/robile_pybullet/robile_pybullet/robile3_config.urdf", [0, 0, 0.1])

    pybullet.setGravity(0, 0, -9.8)
    pybullet.setTimeStep(1. / 240)

    wheel_radius = 0.05
    wheel_base = 0.6
    track_width = 0.4

    try:
        print("Moving robot forward...")
        move_robot_with_drive_wheels(robot_id, linear_velocity=1.0, angular_velocity=0.0, wheel_radius=wheel_radius, wheel_base=wheel_base, track_width=track_width)

        for _ in range(1000):
            pybullet.stepSimulation()
            time.sleep(1. / 240)

            # Get and print wheel velocities
            wheel_velocities = get_wheel_velocities(robot_id)
            for joint_index, velocity in wheel_velocities.items():
                print(f"Joint {joint_index} Velocity: {velocity} rad/s")

        print("Turning robot...")
        move_robot_with_drive_wheels(robot_id, linear_velocity=1.0, angular_velocity=-1, wheel_radius=wheel_radius, wheel_base=wheel_base, track_width=track_width)

        for _ in range(1000):
            pybullet.stepSimulation()
            time.sleep(1. / 240)

            # Get and print wheel velocities
            wheel_velocities = get_wheel_velocities(robot_id)
            for joint_index, velocity in wheel_velocities.items():
                print(f"Joint {joint_index} Velocity: {velocity} rad/s")

        print("Stopping robot...")
        move_robot_with_drive_wheels(robot_id, linear_velocity=0.0, angular_velocity=0.0, wheel_radius=wheel_radius, wheel_base=wheel_base, track_width=track_width)

        for _ in range(1000):
            pybullet.stepSimulation()
            time.sleep(1. / 240)

            # Get and print wheel velocities
            wheel_velocities = get_wheel_velocities(robot_id)
            for joint_index, velocity in wheel_velocities.items():
                print(f"Joint {joint_index} Velocity: {velocity} rad/s")

    except KeyboardInterrupt:
        print("Simulation stopped by user.")
    finally:
        pybullet.disconnect()

def main():
    run_robot_with_caster_simulation()  


if __name__ == '__main__':
    main()
