
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
rotational_joints = [1, 4]  # Indices of the rotational joints for each leg
prismatic_joints = [0, 3]  # Indices of the prismatic joints for each leg

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
def move_joints(robot_id, prismatic_positions, rotational_positions, rotational_force, prismatic_force=100):
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
def perform_gait(robot_id, step_length=0.3, step_height=0.1, speed=0.1):
    time_step = 0

    # Use a fixed phase offset of pi (180 degrees) between the two legs for proper alternation
    while True:
        time_step += speed
        prismatic_positions = [0] * len(prismatic_joints)
        rotational_positions = [0] * len(rotational_joints)
        
        # Create a basic phase offset (ensure they alternate every 180 degrees)
        phase_offset = math.pi  # Half cycle offset for alternating legs
        
        prismatic_offset = math.pi / 2
        # Left leg moves according to sin(time_step), right leg moves with an offset
        rotational_positions[0] = step_length * math.sin(time_step)  # Left leg
        rotational_positions[1] = step_length * math.sin(time_step + phase_offset)  # Right leg
        
        # Lift or push down the legs with sine wave for step height
        prismatic_positions[0] = step_height if math.sin(time_step + prismatic_offset) > 0 else 0  # Left leg
        prismatic_positions[1] = step_height if math.sin(time_step + phase_offset + prismatic_offset) > 0 else 0  # Right leg

        # Apply joint movements to the robot
        move_joints(robot_id, prismatic_positions, rotational_positions, rotational_force=350)

        # Step the simulation
        p.stepSimulation()

        # Update the camera for visualization
        update_camera(robot_id)

        # Sleep to maintain the simulation step rate
        time.sleep(1./120.)

        # Listen for key events and break the loop if 'e' or 'd' is pressed
        keys = p.getKeyboardEvents()
        if ord('e') not in keys and ord('d') not in keys:  # Exit loop when no direction keys are pressed
            break

# Function to control the robot with keys
def control_robot_with_keys(robot_id):
    while True:
        keys = p.getKeyboardEvents()
        # E TO MOVE FORWARDS
        if ord('e') in keys and keys[ord('e')] & p.KEY_IS_DOWN:
            perform_gait(robot_id,  step_length=0.4, step_height=0.3, speed=0.3)
        # D TO MOVE BACKWARDS
        elif ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN:
            perform_gait(robot_id,  step_length=0.4, step_height=0.3, speed=0.4)

        p.stepSimulation()
        time.sleep(1/120)

# Start controlling the robot with keys
control_robot_with_keys(boxId)

# Get and print the final position and orientation of the robot
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(f"Final position: {cubePos}, Final orientation: {cubeOrn}")

# Properly disconnect from the physics server
p.disconnect()
