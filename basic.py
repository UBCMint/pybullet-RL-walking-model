import pybullet as p
import pybullet_data
import time
import os

# Get the current directory and URDF path
current_dir = os.path.dirname(os.path.realpath(__file__))
urdf_path = os.path.join(current_dir, 'stridebot.urdf')


# Connect to PyBullet
physicsClient = p.connect(p.GUI)  # Use p.DIRECT for non-graphical version

p.setRealTimeSimulation(0)
# Set the search path to find built-in URDF files and load the plane
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

# Load a plane for the robot to walk on
planeId = p.loadURDF('plane.urdf')

# Set a larger time step to increase the speed of joint movements
p.setTimeStep(1./500.)  # Use 1/500 for a faster simulation speed

# Define initial position and orientation of the robot
cubeStartPos = [0, 0, 0.05]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])

# Load the custom quadruped URDF
boxId = p.loadURDF(urdf_path, cubeStartPos, cubeStartOrientation)

# Get the number of joints and print their information
num_joints = p.getNumJoints(boxId)
print(f"Number of joints: {num_joints}")

# Increase the friction for the ground to reduce slipping
p.changeDynamics(planeId, -1, lateralFriction=5.0)

def update_camera(robot_id, camera_distance=2):
    # Get the current position of the robot
    robot_pos, _ = p.getBasePositionAndOrientation(robot_id)

    # Set camera to follow the robot's base position
    p.resetDebugVisualizerCamera(
        cameraDistance=camera_distance,   # Adjust the zoom level
        cameraYaw=p.getDebugVisualizerCamera()[8],       # Camera angle around the Z-axis
        cameraPitch=-30,                  # Camera angle up/down
        cameraTargetPosition=robot_pos    # Set camera target to robot's position
    )

# Print joint information for debugging and mapping
for i in range(num_joints):
    joint_info = p.getJointInfo(boxId, i)
    print(f"Joint {i}: Name={joint_info[1].decode('utf-8')} Type={joint_info[2]}")

# Function to move joints with velocity control for faster motion
def move_joints_to_velocity(robot_id, joint_velocities, force=100):
    """
    Move the robot's joints using velocity control to speed up the movements.
    :param robot_id: ID of the robot in the simulation.
    :param joint_velocities: List of target velocities for each joint.
    :param force: Force applied to the joint motors.
    """
    for joint_index, velocity in enumerate(joint_velocities):
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=joint_index,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=velocity,
            force=force
        )

# Function to create a consistent walking motion in one direction with higher speed
def perform_fast_walking_motion(robot_id, num_joints, step_velocity=10.0, reset_velocity=0.0, force=100):
    """
    Perform a faster walking motion using velocity control for coordinated leg movements.
    :param robot_id: ID of the robot in the simulation.
    :param num_joints: Number of joints to control.
    :param step_velocity: Target velocity for the legs during each step.
    :param reset_velocity: Target velocity to reset the legs to the initial position.
    :param force: Force applied to the joint motors.
    """

    # Split the legs into two groups for alternating movements
    left_legs = [0, 1]  # Assuming joint 0 and 2 are left legs
    right_legs = [2, 3]  # Assuming joint 1 and 3 are right legs

    # Step 1: Move left legs forward, right legs backward using velocity control
    joint_velocities = [reset_velocity] * num_joints
    for leg in left_legs:
        joint_velocities[leg] = step_velocity  # Move left legs forward
    for leg in right_legs:
        joint_velocities[leg] = -step_velocity  # Move right legs backward

    move_joints_to_velocity(robot_id, joint_velocities, force)

    # Simulate for a few steps to see the effect
    for _ in range(100):  # Fewer simulation steps for faster movement
        p.stepSimulation()
        update_camera(robot_id)
        time.sleep(1./500.)

    # Step 2: Move left legs backward, right legs forward (opposite motion) using velocity control
    for leg in left_legs:
        joint_velocities[leg] = -step_velocity  # Move left legs backward
    for leg in right_legs:
        joint_velocities[leg] = step_velocity  # Move right legs forward

    move_joints_to_velocity(robot_id, joint_velocities, force)

    # Simulate for a few steps to see the effect
    for _ in range(100):
        p.stepSimulation()
        update_camera(robot_id)
        time.sleep(1./500.)

    # Step 3: Reset all legs to the initial position using zero velocity
    joint_velocities = [reset_velocity] * num_joints
    move_joints_to_velocity(robot_id, joint_velocities, force)

    # Simulate for a few steps to see the effect
    for _ in range(50):
        p.stepSimulation()
        update_camera(robot_id)
        time.sleep(1./500.)

# Run the simulation for a specified number of steps to move forward with faster motion
for _ in range(500):  # Reduce the number of iterations for quicker results
    perform_fast_walking_motion(boxId, num_joints, step_velocity=9.0, force=8)

# Get and print the final position and orientation of the robot
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(f"Final position: {cubePos}, Final orientation: {cubeOrn}")

# Properly disconnect from the physics server
p.disconnect()
