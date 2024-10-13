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

p.setRealTimeSimulation(1)
# Set the search path to find built-in URDF files and load the plane
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)

# Load a plane for the robot to walk on
planeId = p.loadURDF('plane.urdf')

# Set a larger time step to increase the speed of joint movements
p.setTimeStep(1./500.)  # Use 1/500 for a faster simulation speed

# Define initial position and orientation of the robot
cubeStartPos = [0, 0, 0.3]
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

# Set initial angle for legs to start at 45 degrees
initial_angle = math.pi / 4  # 45 degrees in radians

prismatic_joints = [0, 2, 4, 6]  # Prismatic joints for front_left, front_right, back_left, back_right
prismatic_initial_height = -0.1  # Initial height for the prismatic joints (lowered by 0.2 meters)

# Adjust the prismatic joints to lower the legs
for joint in prismatic_joints:
    p.resetJointState(boxId, joint, prismatic_initial_height)
    
# Joint indexes for rotational joints only (ignoring prismatic joints)
rotational_joints = [1, 3, 5, 7]  # Continuous joints for front_left, front_right, back_left, back_right

# Set the initial angle for all the rotational joints
p.resetJointState(boxId, rotational_joints[0], -initial_angle)  # front_left_leg
p.resetJointState(boxId, rotational_joints[2], -initial_angle)  # back_left_leg
p.resetJointState(boxId, rotational_joints[1], initial_angle)   # front_right_leg
p.resetJointState(boxId, rotational_joints[3], initial_angle)   # back_right_leg

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

# Function to move joints with velocity control for smoother motion
def move_joints_to_target_position(robot_id, target_positions, force=200):
    for joint_index, position in enumerate(target_positions):
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=rotational_joints[joint_index],  # Only controlling rotational joints
            controlMode=p.POSITION_CONTROL,
            maxVelocity=2,
            targetPosition=position,  # Target position in radians
            force=force,
            positionGain=0.1  # Optional: Adjust gain for smoother control
        )

# Function to move prismatic joints to lift or lower legs
def move_prismatic_joints(robot_id, target_positions, force=200):
    for joint_index, position in enumerate(target_positions):
        p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=prismatic_joints[joint_index],  # Only controlling prismatic joints
            controlMode=p.POSITION_CONTROL,
            targetPosition=position,  # Set target height for the legs
            force=force
        )

def get_link_index_by_name(robot_id, link_name):
    num_joints = p.getNumJoints(robot_id)
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        current_link_name = joint_info[12].decode('utf-8')  # Link name is stored at index 12
        if current_link_name == link_name:
            print(f"Index is  {i}")
            return i
    return None  # Return None if the link name is not found

# Function to change the friction for a link by its name
def change_leg_friction_by_name(robot_id, link_name, friction_value):
    link_index = get_link_index_by_name(robot_id, link_name)
    if link_index is not None:
        p.changeDynamics(robot_id, link_index, lateralFriction=friction_value)
        print(f"Changed friction for {link_name} (index {link_index}) to {friction_value}")
    else:
        print(f"Link name {link_name} not found.")

# Function to create a consistent walking motion between -45 degrees and +45 degrees
# Function to check if all joints have reached the target positions
def joints_reached_target(robot_id, target_positions):
    for joint_index, target_position in enumerate(target_positions):
        joint_state = p.getJointState(robot_id, rotational_joints[joint_index])
        current_position = joint_state[0]  # Joint position is the first element in the tuple
        # Check if the current position is close to the target position (tolerance of 0.05 radians)
        if abs(current_position - target_position) > 0.05:
            return False
    return True

# Function to create a consistent walking motion between -45 degrees and +45 degrees with only angle checks
def perform_fast_walking_motion(robot_id, num_joints, step_angle=math.pi/4, force=100, steps_per_cycle=60):
    # Split the legs into two groups for alternating movements
    left_legs = [0, 2]  # Left legs (rotational joints 0 and 2)
    right_legs = [1, 3]  # Right legs (rotational joints 1 and 3)

    while True:
        # Step 1: Move left legs to +45 degrees (forward), right legs to -45 degrees (backward)
        joint_positions = [0] * len(rotational_joints)
        for leg in left_legs:
            joint_positions[leg] = step_angle  # Left legs forward to +45 degrees
        for leg in right_legs:
            joint_positions[leg] = -step_angle  # Right legs backward to -45 degrees

        # Adjust friction for the legs
        change_leg_friction_by_name(boxId, "front_right_leg", 5)
        change_leg_friction_by_name(boxId, "back_right_leg", 5)
        change_leg_friction_by_name(boxId, "front_left_leg", 0.2)
        change_leg_friction_by_name(boxId, "back_left_leg", 0.2)
        
        # Move the joints to the target positions
        move_joints_to_target_position(robot_id, joint_positions, force)

        # Simulate for a portion of the cycle to observe movement
        for _ in range(steps_per_cycle):
            p.stepSimulation()
            update_camera(robot_id)
            time.sleep(1./500.)

        # Wait until all joints have reached the target positions (angle check only)
        while not joints_reached_target(robot_id, joint_positions):
            p.stepSimulation()
            update_camera(robot_id)
            time.sleep(1./500.)

        # Step 2: Move left legs to -45 degrees (backward), right legs to +45 degrees (forward)
        for leg in left_legs:
            joint_positions[leg] = -step_angle  # Left legs backward to -45 degrees
        for leg in right_legs:
            joint_positions[leg] = step_angle  # Right legs forward to +45 degrees

        # Adjust friction for the legs
        change_leg_friction_by_name(boxId, "front_right_leg", 0.2)
        change_leg_friction_by_name(boxId, "back_right_leg", 0.2)
        change_leg_friction_by_name(boxId, "front_left_leg", 5)
        change_leg_friction_by_name(boxId, "back_left_leg", 5)
        
        # Move the joints to the target positions
        move_joints_to_target_position(robot_id, joint_positions, force)

        # Simulate for a portion of the cycle to observe movement
        for _ in range(steps_per_cycle):
            p.stepSimulation()
            update_camera(robot_id)
            time.sleep(1./500.)

        # Wait until all joints have reached the target positions (angle check only)
        while not joints_reached_target(robot_id, joint_positions):
            p.stepSimulation()
            update_camera(robot_id)
            time.sleep(1./500.)


# Run the simulation for a specified number of steps to m-ove forward with smoother motion

perform_fast_walking_motion(boxId, num_joints, step_angle=math.pi/4, force=16)

# Get and print the final position and orientation of the robot
cubePos, cubeOrn = p.getBasePositionAndOrientation(boxId)
print(f"Final position: {cubePos}, Final orientation: {cubeOrn}")

# Properly disconnect from the physics server
p.disconnect()
