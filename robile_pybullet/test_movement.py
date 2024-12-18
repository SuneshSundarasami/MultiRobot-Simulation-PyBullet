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
        pybullet.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=group['left'],
            controlMode=pybullet.VELOCITY_CONTROL,
            targetVelocity=inner_velocity / wheel_radius if angular_velocity > 0 else outer_velocity / wheel_radius,
            force=100,
            positionGain=0.1,
            velocityGain=0.1
        )

        pybullet.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=group['right'],
            controlMode=pybullet.VELOCITY_CONTROL,
            targetVelocity=outer_velocity / wheel_radius if angular_velocity > 0 else inner_velocity / wheel_radius,
            force=100,
            positionGain=0.1,
            velocityGain=0.1
        )

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

            linear_velocity, angular_velocity = pybullet.getBaseVelocity(robot_id)
            print(f"Linear Velocity: {linear_velocity[0]} m/s, Angular Velocity: {angular_velocity[2]} rad/s")

        print("Turning robot...")
        move_robot_with_drive_wheels(robot_id, linear_velocity=1.0, angular_velocity=1, wheel_radius=wheel_radius, wheel_base=wheel_base, track_width=track_width)

        for _ in range(1000):
            pybullet.stepSimulation()
            time.sleep(1. / 240)

            linear_velocity, angular_velocity = pybullet.getBaseVelocity(robot_id)
            print(f"Linear Velocity: {linear_velocity[0]} m/s, Angular Velocity: {angular_velocity[2]} rad/s")

        print("Stopping robot...")
        move_robot_with_drive_wheels(robot_id, linear_velocity=0.0, angular_velocity=0.0, wheel_radius=wheel_radius, wheel_base=wheel_base, track_width=track_width)

        for _ in range(1000):
            pybullet.stepSimulation()
            time.sleep(1. / 240)

            linear_velocity, angular_velocity = pybullet.getBaseVelocity(robot_id)
            print(f"Linear Velocity: {linear_velocity[0]} m/s, Angular Velocity: {angular_velocity[2]} rad/s")

    except KeyboardInterrupt:
        print("Simulation stopped by user.")
    finally:
        pybullet.disconnect()

def main():
    run_robot_with_caster_simulation()  


if __name__ == '__main__':
    main()
