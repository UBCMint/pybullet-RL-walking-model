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
    p.changeDynamics(boxId, joint, lateralFriction=10.0)



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
# Joint indices for rotational and prismatic joints
rotational_joints = [1, 3, 5, 7]  # Indices of the rotational joints for each leg
prismatic_joints = [0, 2, 4, 6]  # Indices of the prismatic joints for each leg

# Function to move both prismatic and rotational joints
def move_joints(robot_id, prismatic_positions, rotational_positions, rotational_force, prismatic_force = 50):
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

# Modify perform_trot_gait to control both prismatic and rotational joints
def perform_trot_gait(robot_id, num_joints, step_length=0.3, step_height=0.1, speed=0.1):
    phase_offset = math.pi  # Offset for alternating legs
    prismatic_offset = math.pi / 2
    # Indices for diagonal pairs of legs
    diagonal_1 = [0, 3]  # Front left and back right
    diagonal_2 = [1, 2]  # Front right and back left

    time_step = 0
    
    start_time = time.time()  # Record the start time
    while True:
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
        move_joints(robot_id, prismatic_positions, rotational_positions, rotational_force = 600)

        # Step the simulation and update the camera
        p.stepSimulation()
        update_camera(robot_id)
        time.sleep(1./240.)
        if (ord('e') not in p.getKeyboardEvents() and step_length == 0.3) or (ord('d') not in p.getKeyboardEvents() and step_length == -0.3):
            break

def do_nothing(robot_id):
    start_time = time.time()  # Record the start time
    while True:
        p.stepSimulation()
        update_camera(robot_id)
        time.sleep(1./240.)
        keys = p.getKeyboardEvents()
        if ord('e') in keys or ord('s') in keys or ord('d') in keys or ord('f') in keys:
            break

def rotate(robot_id, num_joints, direction, step_height=0.2, rotation_angle=math.pi / 4, speed=0.2, steps_per_cycle = 5):
    # direction = True ---> Rotate right, direction = False ---> Rotate left
    if (direction):
        lift_joints = [1, 3]  # Front-left and back-left
        non_lift_joints = [0, 2]
    else:
        lift_joints = [0, 2]  # Front-left and back-left
        non_lift_joints = [1, 3]

    
    # Initialize time tracking
    time_step = 0
    start_time = time.time()

    while True:
        if (ord('f') not in p.getKeyboardEvents() and direction) or (ord('s') not in p.getKeyboardEvents() and not direction):
            break
        time_step += speed
        prismatic_positions = [0] * len(prismatic_joints)
        rotational_positions = [0] * len(rotational_joints)

        # Step 1: Lift the left-side legs using prismatic joints (jump)
        for joint in lift_joints:
            prismatic_positions[joint] = 0  # Lift the leg
        for joint in non_lift_joints:
            prismatic_positions[joint] = 0  
        # Apply the prismatic and rotational joint movements
        move_joints(robot_id, prismatic_positions, rotational_positions, rotational_force = 600, prismatic_force = 1000)

        # Step the simulation and update the camera
        for _ in range(steps_per_cycle):
            p.stepSimulation()
            update_camera(robot_id)
            time.sleep(1./240.)

        for joint in (lift_joints + non_lift_joints):
            prismatic_positions[joint] = step_height
        for joint in lift_joints:
            rotational_positions[joint] = rotation_angle * abs(math.sin(time_step))  # Rotate the leg forward
        for joint in non_lift_joints:
            rotational_positions[joint] = -rotation_angle * abs(math.sin(time_step))  # Rotate the leg forward

        move_joints(robot_id, prismatic_positions, rotational_positions, rotational_force = 600, prismatic_force = 200)

        for _ in range(steps_per_cycle):
            p.stepSimulation()
            update_camera(robot_id)
            time.sleep(1./240.)
        # Step 3: Bring the lifted legs back to the ground (lower)
        for joint in lift_joints + non_lift_joints:
            prismatic_positions[joint] = step_height  # Lower the leg
            rotational_positions[joint] = 0

        # Apply the prismatic and rotational joint movements to return to the start
        move_joints(robot_id, prismatic_positions, rotational_positions, rotational_force = 600, prismatic_force = 200)

        # Step the simulation and update the camera
        for _ in range(steps_per_cycle):
            p.stepSimulation()
            update_camera(robot_id)
            time.sleep(1./240.)
    

def control_robot_with_keys(robot_id, num_joints):
    while True:
        keys = p.getKeyboardEvents()

        # F TO GO RIGHT
        if ord('f') in keys and keys[ord('f')] & p.KEY_IS_DOWN:
            rotate(boxId, num_joints, direction = True, step_height=0.2, rotation_angle=math.pi / 4)
        # S TO GO LEFT
        elif ord('s') in keys and keys[ord('s')] & p.KEY_IS_DOWN:
            rotate(boxId, num_joints, direction = False, step_height=0.2, rotation_angle=math.pi / 4)
        # E TO GO FORWARDS
        elif ord('e') in keys and keys[ord('e')] & p.KEY_IS_DOWN:
            perform_trot_gait(boxId, num_joints, step_length=0.3, step_height=0.2, speed=0.4)
        # D TO GO BACKWARDS
        elif ord('d') in keys and keys[ord('d')] & p.KEY_IS_DOWN:
            perform_trot_gait(boxId, num_joints, step_length= -0.3, step_height=0.2, speed=0.4)
            
        elif not (ord('w') in keys or ord('a') in keys or ord('s') in keys or ord('d') in keys):  # If no keys are pressed at all
            do_nothing(robot_id)

        p.stepSimulation()
        time.sleep(1./240.)


control_robot_with_keys(boxId, num_joints)

# Get and print the final position and orientation of the robot
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(f"Final position: {cubePos}, Final orientation: {cubeOrn}")

# Properly disconnect from the physics server
p.disconnect()
