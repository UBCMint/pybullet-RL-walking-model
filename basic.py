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
p.setGravity(0, 0, -9.8)

# Load a plane for the robot to walk on
planeId = p.loadURDF('plane.urdf')

# Define initial position and orientation of the robot
cubeStartPos = [0, 0, 0.4]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])

# Load the custom quadruped URDF
boxId = p.loadURDF(urdf_path, cubeStartPos, cubeStartOrientation)

num_joints = p.getNumJoints(boxId)
print(f"Number of joints: {num_joints}")

# Set up initial dynamics for the robot's legs to ensure good ground contact
for joint in range(num_joints):
    p.changeDynamics(boxId, joint, lateralFriction=5.0)



# Function to update the camera to follow the robot
def update_camera(robot_id, camera_distance=2):
    robot_pos, _ = p.getBasePositionAndOrientation(robot_id)
    p.resetDebugVisualizerCamera(
        cameraDistance=camera_distance,
        cameraYaw=p.getDebugVisualizerCamera()[8],
        cameraPitch= -30,
        cameraTargetPosition=robot_pos
    )


# Function to move joints with velocity control for smoother motion
# Joint indices for rotational and prismatic joints
rotational_joints = [1, 3, 5, 7]  # Indices of the rotational joints for each leg
prismatic_joints = [0, 2, 4, 6]  # Indices of the prismatic joints for each leg

# Function to move both prismatic and rotational joints
def move_joints(robot_id, prismatic_positions, rotational_positions, force=200):
    # Move prismatic joints (vertical movement)
    for joint_index, position in enumerate(prismatic_positions):
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=prismatic_joints[joint_index],  # Prismatic joints
            controlMode=p.POSITION_CONTROL,
            targetPosition=position,  # Target vertical position
            force=50,
        )
    
    # Move rotational joints (leg movement)
    for joint_index, position in enumerate(rotational_positions):
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=rotational_joints[joint_index],  # Rotational joints
            controlMode=p.POSITION_CONTROL,
            targetPosition=position,  # Target angular position
            force=force,
            maxVelocity=4
        )

# Modify perform_trot_gait to control both prismatic and rotational joints
def perform_trot_gait(robot_id, num_joints, duration, step_length=0.3, step_height=0.1, speed=0.1):
    phase_offset = math.pi  # Offset for alternating legs
    prismatic_offset = math.pi / 2
    # Indices for diagonal pairs of legs
    diagonal_1 = [0, 3]  # Front left and back right
    diagonal_2 = [1, 2]  # Front right and back left

    time_step = 0
    
    start_time = time.time()  # Record the start time
    while time.time() - start_time < duration:
        time_step += speed
        prismatic_positions = [0] * len(prismatic_joints)
        rotational_positions = [0] * len(rotational_joints)

        # Move diagonal_1 (front left and back right) first
        for joint in diagonal_1:
            rotational_positions[joint] = step_length * math.sin(time_step)  # Forward/backward motion
            if math.sin(time_step + prismatic_offset) > 0:
                prismatic_positions[joint] = step_height  # Lift the leg
            else:
                prismatic_positions[joint] = 0  # Push the leg down during stance

        # Move diagonal_2 (front right and back left) with a phase offset
        for joint in diagonal_2:
            rotational_positions[joint] = step_length * math.sin(time_step + phase_offset)
            if math.sin(time_step + phase_offset + prismatic_offset) > 0:
                prismatic_positions[joint] = step_height  # Lift the leg
            else:
                prismatic_positions[joint] = 0  # Push the leg down during stance

        # Apply prismatic and rotational joint movements
        move_joints(robot_id, prismatic_positions, rotational_positions, 200)

        # Step the simulation and update the camera
        p.stepSimulation()
        update_camera(robot_id)
        time.sleep(1./240.)

def do_nothing(robot_id, duration):
    start_time = time.time()  # Record the start time
    while time.time() - start_time < duration:
        p.stepSimulation()
        update_camera(robot_id)
        time.sleep(1./240.)

def rotate(robot_id, num_joints, duration, step_height=0.2, rotation_angle=math.pi / 4, speed=0.01):

    # Indices for front-left (prismatic: 0, rotational: 1) and back-left (prismatic: 2, rotational: 3) legs
    prismatic_lift_joints = [0, 2]  # Front-left and back-left
    rotational_lift_joints = [1, 3]  # Corresponding rotational joints for the lifted legs

    # Initialize time tracking
    time_step = 0
    start_time = time.time()

    while time.time() - start_time < duration:
        time_step += speed
        prismatic_positions = [0] * len(prismatic_joints)
        rotational_positions = [0] * len(rotational_joints)

        # Step 1: Lift the left-side legs using prismatic joints (jump)
        for joint in prismatic_lift_joints:
            prismatic_positions[joint] = -step_height  # Lift the leg

        # Step 2: Rotate the lifted legs forward while they are in the air
        for joint in rotational_lift_joints:
            rotational_positions[joint] = rotation_angle * abs(math.sin(time_step))  # Rotate the leg forward

        # Apply the prismatic and rotational joint movements
        move_joints(robot_id, prismatic_positions, rotational_positions, 200)

        # Step the simulation and update the camera
        p.stepSimulation()
        update_camera(robot_id)
        time.sleep(1./240.)

        # Step 3: Bring the lifted legs back to the ground (lower)
        for joint in prismatic_lift_joints:
            prismatic_positions[joint] = 0  # Lower the leg

        # Step 4: Rotate the legs back to their starting positions
        for joint in rotational_lift_joints:
            rotational_positions[joint] = 0  # Reset the leg rotation

        # Apply the prismatic and rotational joint movements to return to the start
        move_joints(robot_id, prismatic_positions, rotational_positions, 200)

        # Step the simulation and update the camera
        p.stepSimulation()
        update_camera(robot_id)
        time.sleep(1./240.)


# Example usage of the rotate function
rotate(boxId, num_joints, duration=7, step_height=0.2, rotation_angle=math.pi / 4, speed=0.2)

perform_trot_gait(boxId, num_joints, duration=20, step_length=0.3, step_height=0.2, speed=0.2)
do_nothing(boxId, duration = 10)


# Get and print the final position and orientation of the robot
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(f"Final position: {cubePos}, Final orientation: {cubeOrn}")

# Properly disconnect from the physics server
p.disconnect()
