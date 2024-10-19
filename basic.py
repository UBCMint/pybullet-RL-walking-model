import pybullet as p
import pybullet_data
import time
import os
import math

# Set up PyBullet environment
current_dir = os.path.dirname(os.path.realpath(__file__))
urdf_path = os.path.join(current_dir, 'stridebot.urdf')

# Connect to PyBullet
physicsClient = p.connect(p.GUI)  
p.setRealTimeSimulation(1)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -70)

# Load a plane for the robot to walk on
planeId = p.loadURDF('plane.urdf')

# Define initial position and orientation of the robot
cubeStartPos = [0, 0, 0.3]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])

# Load the custom quadruped URDF
boxId = p.loadURDF(urdf_path, cubeStartPos, cubeStartOrientation)

num_joints = p.getNumJoints(boxId)
print(f"Number of joints: {num_joints}")

# Set up initial dynamics for the robot's legs to ensure good ground contact
for joint in range(num_joints):
    p.changeDynamics(boxId, joint, lateralFriction=5.0)

# Joint indices for each leg
rotational_joints = [0, 1, 2, 3]  # Indices of the four legs

# Function to update the camera to follow the robot
def update_camera(robot_id, camera_distance=4):
    robot_pos, _ = p.getBasePositionAndOrientation(robot_id)
    p.resetDebugVisualizerCamera(
        cameraDistance=camera_distance,
        cameraYaw=p.getDebugVisualizerCamera()[8],
        cameraPitch= -30,
        cameraTargetPosition=robot_pos
    )

# Function to move joints with velocity control for smoother motion
def move_joints_to_target_position(robot_id, target_positions, force=200):
    for joint_index, position in enumerate(target_positions):
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=rotational_joints[joint_index],  # Only controlling rotational joints
            controlMode=p.POSITION_CONTROL,
            targetPosition=position,  # Target position in radians
            force=force,
            maxVelocity=4
        )

# Perform a diagonal (trot) gait to maintain balance
def perform_trot_gait(robot_id, num_joints, step_length=0.3, step_height=0.2, speed=0.1):
    phase_offset = math.pi  # Offset for alternating legs

    # Indices for diagonal pairs of legs
    diagonal_1 = [0, 3]  # Front left and back right
    diagonal_2 = [1, 2]  # Front right and back left

    time_step = 0
    while True:
        time_step += speed
        joint_positions = [0] * num_joints

        # Move diagonal_1 (front left and back right) first
        for joint in diagonal_1:
            joint_positions[joint] = step_length * math.sin(time_step)  # Forward/backward motion
            if math.sin(time_step) > 0:
                joint_positions[joint] += step_height * math.sin(time_step)  # Lift legs when moving forward

        # Move diagonal_2 (front right and back left) with a phase offset
        for joint in diagonal_2:
            joint_positions[joint] = step_length * math.sin(time_step + phase_offset)
            if math.sin(time_step + phase_offset) > 0:
                joint_positions[joint] += step_height * math.sin(time_step + phase_offset)

        # Apply joint movements
        move_joints_to_target_position(robot_id, joint_positions, 1000)

        # Step the simulation and update the camera
        p.stepSimulation()
        update_camera(robot_id)
        time.sleep(1./240.)

# Perform the trot gait motion with proper leg lifting
perform_trot_gait(boxId, num_joints, step_length=0.3, step_height=0.9, speed=0.2)

# Get and print the final position and orientation of the robot
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(f"Final position: {cubePos}, Final orientation: {cubeOrn}")

# Properly disconnect from the physics server
p.disconnect()
