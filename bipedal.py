import pybullet as p
import pybullet_data
import time
import os
import math

# Set up PyBullet environment
current_dir = os.path.dirname(os.path.realpath(__file__))
urdf_path = os.path.join(current_dir, 'bipedal.urdf')  # Use the bipedal URDF

# Connect to PyBullet
physicsClient = p.connect(p.GUI)  
p.setRealTimeSimulation(1)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)

# Load a plane for the robot to walk on
planeId = p.loadURDF('plane.urdf')

# Define initial position and orientation of the robot
cubeStartPos = [0, 0, 0.6]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])

# Load the custom bipedal URDF
boxId = p.loadURDF(urdf_path, cubeStartPos, cubeStartOrientation)

num_joints = p.getNumJoints(boxId)
print(f"Number of joints: {num_joints}")

# Set up initial dynamics for the robot's legs to ensure good ground contact
for joint in range(num_joints):
    p.changeDynamics(boxId, joint, lateralFriction=10.0)

# Indices for the joints in the bipedal robot
rotational_joints = [1, 3]  # Indices of the rotational joints for each leg
prismatic_joints = [0, 2]  # Indices of the prismatic joints for each leg

# Function to update the camera to follow the robot
def update_camera(robot_id, camera_distance=2):
    robot_pos, robot_orientation = p.getBasePositionAndOrientation(robot_id)
    robot_euler = p.getEulerFromQuaternion(robot_orientation)
    robot_yaw = robot_euler[2]  # yaw is the rotation around the z-axis
    camera_yaw = robot_yaw * (180.0 / math.pi)

    # Update the camera with the calculated yaw
    p.resetDebugVisualizerCamera(
        cameraDistance=camera_distance,
        cameraYaw=camera_yaw + 90,
        cameraPitch=-30,
        cameraTargetPosition=robot_pos
    )

# Function to move joints with velocity control for smoother motion
def move_joints(robot_id, prismatic_positions, rotational_positions, rotational_force, prismatic_force=50):
    # Move prismatic joints (vertical movement)
    for joint_index, position in enumerate(prismatic_positions):
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=prismatic_joints[joint_index],  # Prismatic joints
            controlMode=p.POSITION_CONTROL,
            targetPosition=position,  # Target vertical position
            force=prismatic_force,
        )
    
    # Move rotational joints (leg movement)
    for joint_index, position in enumerate(rotational_positions):
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=rotational_joints[joint_index],  # Rotational joints
            controlMode=p.POSITION_CONTROL,
            targetPosition=position,  # Target angular position
            force=rotational_force,
            maxVelocity=4
        )

# Function to perform a gait for the bipedal robot
def perform_gait(robot_id, num_joints, step_length=0.3, step_height=0.1, speed=0.1):
    phase_offset = math.pi
    time_step = 0
    
    start_time = time.time()
    while True:
        time_step += speed
        prismatic_positions = [0] * len(prismatic_joints)
        rotational_positions = [0] * len(rotational_joints)

        # Alternate legs for a basic walking gait
        rotational_positions[0] = step_length * math.sin(time_step)  # Left leg
        rotational_positions[1] = step_length * math.sin(time_step + phase_offset)  # Right leg
        prismatic_positions[0] = step_height if math.sin(time_step) > 0 else 0  # Lift or push down left leg
        prismatic_positions[1] = step_height if math.sin(time_step - phase_offset) > 0 else 0  # Lift or push down right leg

        # Apply joint movements
        move_joints(robot_id, prismatic_positions, rotational_positions, rotational_force=600)

        # Step the simulation and update the camera
        p.stepSimulation()
        update_camera(robot_id)
        time.sleep(1./240.)

        keys = p.getKeyboardEvents()
        if ord('e') not in keys and ord('d') not in keys:  # Exit loop when no direction keys are pressed
            break

# Function to control the robot with keys
def control_robot_with_keys(robot_id, num_joints):
    while True:
        keys = p.getKeyboardEvents()

        # E TO MOVE FORWARDS
        if ord('e') in keys and keys[ord('e')] & p.KEY_IS_DOWN:
            perform_gait(robot_id, num_joints, step_length=0.3, step_height=0.2, speed=0.4)
        # D TO MOVE BACKWARDS
        elif ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN:
            perform_gait(robot_id, num_joints, step_length=-0.3, step_height=0.2, speed=0.4)

        p.stepSimulation()
        time.sleep(1./240.)

# Start controlling the robot with keys
control_robot_with_keys(boxId, num_joints)

# Get and print the final position and orientation of the robot
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(f"Final position: {cubePos}, Final orientation: {cubeOrn}")

# Properly disconnect from the physics server
p.disconnect()
