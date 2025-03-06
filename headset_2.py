import pybullet as p
import pybullet_data
import time
import os
import math
import numpy as np
import random
import requests
import urllib3
urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

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

# Load the custom quadruped robot URDF
# THIS IS NOT THE GOAL BOX, this refers to the body of the robot
boxId = p.loadURDF(urdf_path, cubeStartPos, cubeStartOrientation)

num_joints = p.getNumJoints(boxId)
print(f"Number of joints: {num_joints}")

# Set up initial dynamics for the robot's legs to ensure good ground contact
for joint in range(num_joints):
    p.changeDynamics(boxId, joint, lateralFriction=50.0)



# Function to update the camera to follow the robot
def update_camera(robot_id, camera_distance=2):
    robot_pos, robot_orientation = p.getBasePositionAndOrientation(robot_id)
    robot_euler = p.getEulerFromQuaternion(robot_orientation)
    robot_yaw = robot_euler[2]  # yaw is the rotation around the z-axis
    camera_yaw = robot_yaw * (180.0 / math.pi)

    # Update the camera with the calculated yaw
    p.resetDebugVisualizerCamera(
        cameraDistance=camera_distance,
        cameraYaw=p.getDebugVisualizerCamera()[8],  # p.getDebugVisualizerCamera()[8] # camera_yaw + 90
        cameraPitch=-30,
        cameraTargetPosition=robot_pos
    )


# Function to move joints with velocity control for smoother motion
# Joint indices for rotational and prismatic joints
rotational_joints = [1, 3, 5, 7]  # Indices of the rotational joints for each leg
prismatic_joints = [0, 2, 4, 6]  # Indices of the prismatic joints for each leg

# Function to move both prismatic and rotational joints
def move_joints(robot_id, prismatic_positions, rotational_positions, rotational_force = 600, prismatic_force = 50):
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

calculate = False
last_box_id = None
# Modify perform_trot_gait to control both prismatic and rotational joints
def perform_trot_gait(robot_id, step_length, step_height=0.1, speed=0.1):
    global calculate
    global headset_data
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
        move_joints(robot_id, prismatic_positions, rotational_positions)

        detect_collision(robot_id)
        # Step the simulation and update the camera
        p.stepSimulation()
        update_camera(robot_id)
        time.sleep(1./240.)
        with open("shared_data.txt", "r") as f:
            current_data = f.read()
        if current_data != headset_data:
            break
        if (calculate):
            calculate_angle_and_distance(robot_id=boxId, box_id=last_box_id)
       

def do_nothing(robot_id):
    global calculate
    global headset_data
    start_time = time.time()  # Record the start time
    while True:
        keys = p.getKeyboardEvents()
        with open("shared_data.txt", "r") as f:
            current_data = f.read()
        if current_data != headset_data:
            break
        if (calculate):
            calculate_angle_and_distance(robot_id=boxId, box_id=last_box_id)
        p.stepSimulation()
        update_camera(robot_id)
        time.sleep(1./240.)

def rotate(robot_id, direction, step_height=0.2, rotation_angle=math.pi / 4, speed=0.2, steps_per_cycle = 5):
    global calculate
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
        move_joints(robot_id, prismatic_positions, rotational_positions, prismatic_force = 500)

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

        move_joints(robot_id, prismatic_positions, rotational_positions, prismatic_force = 500)

        for _ in range(steps_per_cycle):
            p.stepSimulation()
            update_camera(robot_id)
            time.sleep(1./240.)
        # Step 3: Bring the lifted legs back to the ground (lower)
        for joint in lift_joints + non_lift_joints:
            prismatic_positions[joint] = step_height  # Lower the leg
            rotational_positions[joint] = 0

        # Apply the prismatic and rotational joint movements to return to the start
        move_joints(robot_id, prismatic_positions, rotational_positions, prismatic_force = 500)

        # Step the simulation and update the camera
        for _ in range(steps_per_cycle):
            p.stepSimulation()
            update_camera(robot_id)
            time.sleep(1./240.)
        if (calculate):
            calculate_angle_and_distance(robot_id=boxId, box_id=last_box_id)
        detect_collision(robot_id)
    



def spawn_box(robot_id, distance_ahead=1.0, side_offset = 0):    # if offset is positive the box is to the right
    distance_ahead = -distance_ahead

    global last_box_id

    if last_box_id is not None:
        p.removeBody(last_box_id)
        
    # Get the robot's current position and orientation
    robot_pos, robot_orientation = p.getBasePositionAndOrientation(robot_id)

    # Convert the orientation quaternion to Euler angles to get the yaw
    robot_euler = p.getEulerFromQuaternion(robot_orientation)
    robot_yaw = robot_euler[2]  # yaw is the third element
    forward_direction = [math.cos(robot_yaw), math.sin(robot_yaw), 0]
    side_direction = [-math.sin(robot_yaw), math.cos(robot_yaw), 0]

    box_position = [
        robot_pos[0] + forward_direction[0] * distance_ahead + side_direction[0] * side_offset,
        robot_pos[1] + forward_direction[1] * distance_ahead + side_direction[1] * side_offset,
        robot_pos[2] + 0.3
    ]

    current_dir = os.path.dirname(os.path.realpath(__file__))
    cube_urdf_path = os.path.join(current_dir, "goal_box.urdf")
    box_orientation = p.getQuaternionFromEuler([0, 0, 0])

    # Spawn the new box and store its ID
    last_box_id = p.loadURDF(cube_urdf_path, box_position, box_orientation)


def calculate_angle_and_distance(robot_id, box_id):
    
    robot_pos, robot_orientation = p.getBasePositionAndOrientation(robot_id)
    robot_euler = p.getEulerFromQuaternion(robot_orientation)
    robot_yaw = robot_euler[2]

    # Get the box's position
    box_pos, _ = p.getBasePositionAndOrientation(box_id)

    # Calculate the direction vector from robot to box
    direction_to_box = np.array([box_pos[0] - robot_pos[0], box_pos[1] - robot_pos[1]])

    # Calculate the forward direction of the robot
    robot_forward = np.array([np.cos(robot_yaw), np.sin(robot_yaw)])

    # Calculate the angle between the forward direction and the direction to the box
    dot_product = np.dot(robot_forward, direction_to_box)
    magnitude_product = np.linalg.norm(robot_forward) * np.linalg.norm(direction_to_box)

    if magnitude_product == 0:
        angle_degrees = 0
    else:
        cos_theta = dot_product / magnitude_product
        angle_radians = np.arccos(np.clip(cos_theta, -1.0, 1.0))
        angle_degrees = np.degrees(angle_radians)


    distance = np.linalg.norm(direction_to_box)
    angle_degrees -= 180
    print(f"Angle between the robot and the box: {angle_degrees} degrees")
    print(f"Distance between the robot and the box: {distance}")
    # return angle_degrees, distance


last_box_position = None

def detect_collision(robot_id):
    global last_box_position

    # Get the current position of the box
    box_position, _ = p.getBasePositionAndOrientation(last_box_id)

    # Check if this is the first time the function is called
    if last_box_position is None:
        last_box_position = box_position  # Initialize the position
        return False  # No movement detected yet

    # Check if the box moved in the x or y direction
    movement_detected = (
        abs(box_position[0] - last_box_position[0]) > 1e-3 or  # Movement in x
        abs(box_position[1] - last_box_position[1]) > 1e-3     # Movement in y
    )

    # radius is the variable determining how far away the box will spawn around the robot, the box is spawned in a random position around a circle area of the robot
    radius = random.uniform(2, 5)
    # Generate a random angle in radians
    angle = random.uniform(0, 2 * math.pi)

    # Calculate x and y coordinates on the circle
    x_offset = radius * math.cos(angle)
    y_offset = radius * math.sin(angle)
    
    if movement_detected:
        print(f"Box moved! New position: {box_position}")
        spawn_box(robot_id, distance_ahead=y_offset, side_offset=x_offset)
        # Update the last known position of the box
        last_box_position, _ = p.getBasePositionAndOrientation(last_box_id)


headset_data = 1
def control_robot_with_keys(robot_id):
    global calculate
    global headset_data
    spawn_box(robot_id, distance_ahead=2, side_offset=0) 
    keys = [3.6]
    calculate = True
    while True:
        with open("shared_data.txt", "r") as f:
            headset_data = f.read()

        # url = "http://50b4-206-87-120-166.ngrok-free.app/shared_data.txt"

        # try:
        #     response = requests.get(url, verify=False)

        #     response.raise_for_status()  # Raise an error if the request failed
        #     headset_data = response.text
        # except requests.exceptions.RequestException as e:
        #     print(f"Error fetching data: {e}")
    
        # print("read " + headset_data + " from shared_data.txt")
        
        if headset_data == "1":
            perform_trot_gait(boxId, step_length=0.3, step_height=0.2, speed=0.4)
        elif headset_data == "0":
            perform_trot_gait(boxId, step_length=-0.3, step_height=0.2, speed=0.4)
        else:
            do_nothing(robot_id)
        print(headset_data)
        
        p.stepSimulation()
        time.sleep(1./50.)
        keys = p.getKeyboardEvents()

        

control_robot_with_keys(boxId)

# Get and print the final position and orientation of the robot
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(f"Final position: {cubePos}, Final orientation: {cubeOrn}")

# Properly disconnect from the physics server
p.disconnect()
