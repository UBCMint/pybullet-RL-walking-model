import pybullet as p
import pybullet_data
import time
import os
import math

# Get the current directory and URDF path
current_dir = os.path.dirname(os.path.realpath(__file__))
urdf_path = os.path.join(current_dir, 'stridebot.urdf')

# Connect to PyBullet
physicsClient = p.connect(p.GUI)  # Use p.DIRECT for non-graphical version

# Set the search path to find built-in URDF files and load the plane
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

# Load a plane for the robot to walk on
planeId = p.loadURDF('plane.urdf')

# Set a larger time step to increase the speed of joint movements
p.setTimeStep(1. / 240.)

# Define initial position and orientation of the robot
cubeStartPos = [0, 0, 0.2]  # Lift the robot a bit higher initially
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])

# Load the custom quadruped URDF
boxId = p.loadURDF(urdf_path, cubeStartPos, cubeStartOrientation)

# Get the number of joints and print their information
num_joints = p.getNumJoints(boxId)
print(f"Number of joints: {num_joints}")

# Increase the friction for the ground to reduce slipping
p.changeDynamics(planeId, -1, lateralFriction=10.0)  # Increased friction value

# Set up initial dynamics for the robot's legs to ensure good ground contact
for joint in range(num_joints):
    p.changeDynamics(boxId, joint, lateralFriction=5.0)  # Increased friction for legs
    p.changeDynamics(boxId, joint, jointDamping=0.5)  # Damping to reduce vibrations

# Print joint information for debugging and mapping
for i in range(num_joints):
    joint_info = p.getJointInfo(boxId, i)
    print(f"Joint {i}: Name={joint_info[1].decode('utf-8')} Type={joint_info[2]}")

# Function to move joints using position control for smoother motion
def move_joints_to_position(robot_id, joint_positions, max_force=50):
    """
    Move the robot's joints using position control.
    :param robot_id: ID of the robot in the simulation.
    :param joint_positions: List of target positions for each joint.
    :param max_force: Maximum force applied to each joint motor.
    """
    for joint_index, position in enumerate(joint_positions):
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=joint_index,
            controlMode=p.POSITION_CONTROL,
            targetPosition=position,
            force=max_force
        )

# Function to create a consistent walking motion with sinusoidal patterns
def perform_gait_motion(robot_id, num_joints, step_length=0.3, step_height=0.2, speed=0.1):
    """
    Perform a simple alternating gait motion using sinusoidal control for coordinated leg movements.
    :param robot_id: ID of the robot in the simulation.
    :param num_joints: Number of joints to control.
    :param step_length: Amplitude of the movement forward and backward.
    :param step_height: Amplitude of the upward and downward movement.
    :param speed: Speed of the leg movements.
    """
    phase_offset = math.pi  # Offset for alternating legs (180 degrees)

    # Assign indices of the legs to create a trot gait pattern
    left_legs = [0, 2]  # Left legs joint indices
    right_legs = [1, 3]  # Right legs joint indices

    time_step = 0

    # Run the gait for a specified number of steps
    for _ in range(1000):
        time_step += speed

        # Calculate joint positions based on a simple sinusoidal function for each leg
        joint_positions = [0] * num_joints
        for joint in left_legs:
            # Moving left legs in a sinusoidal pattern
            joint_positions[joint] = step_length * math.sin(time_step)  # Move forward/backward
            joint_positions[joint] += step_height * math.cos(time_step)  # Move up/down

        for joint in right_legs:
            # Moving right legs in the opposite phase of left legs
            joint_positions[joint] = step_length * math.sin(time_step + phase_offset)
            joint_positions[joint] += step_height * math.cos(time_step + phase_offset)

        # Apply the positions to the joints using position control
        move_joints_to_position(robot_id, joint_positions)

        # Apply a small forward force to the robot's base link to encourage movement
        p.applyExternalForce(robot_id, -1, [50, 0, 0], [0, 0, 0], p.WORLD_FRAME)  # Adjust force as needed

        # Step the simulation forward to see the effect
        p.stepSimulation()
        time.sleep(1. / 240.)

# Run the gait motion
perform_gait_motion(boxId, num_joints, step_length=0.3, step_height=0.1, speed=0.2)

# Get and print the final position and orientation of the robot
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(f"Final position: {cubePos}, Final orientation: {cubeOrn}")

# Properly disconnect from the physics server
p.disconnect()
