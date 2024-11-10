# pybullet-RL-walking-model

URDF creator website: https://marksoulier.github.io/URDF_creator/  
F TO GO RIGHT           
S TO GO LEFT    
E TO GO FORWARDS  
D TO GO BACKWARDS  

# Code Documentation (For the basic.py file)
The global variables and functions are listed alphabetically.
## Global Variables
### calculate
Determines whether real time distance and angle calculation between the robot and the box should be done. Set to True after a box is spawned.
### last_box_id 
Keeps track of the current goal box that is present.
### prismatic_joints
Indices of the prismatic joints for each leg
### rotational_joints
Indices of the rotational joints for each leg

## Functions
### move_joints(robot_id, prismatic_positions, rotational_positions, rotational_force, prismatic_force = 50)
**Description:** Controls the movement of both prismatic and rotational joints on the robot. It sets target positions and forces for each joint type.  
**prismatic_positions:** List of target positions (elevation relative to the z-axis) for the prismatic joints of the robot’s legs  
**rotational_positions:** List of target positions (angel of the legs relative to the z-axis) for the rotational joints of the robot’s legs  
**rotational_force:** Force that will be applied to the rotational joints  
**prismatic_force:** Force that will be applied to the prismatic joints  

### perform_trot_gait(robot_id, step_length=0.3, step_height=0.1, speed=0.1)
**Description:** Performs the forwards and backwards motion of the robot by making use of the **move_joints** function.  
**robot_id:** urdf of the robot  
**step_length:** How much the legs will reach forwards and backwards rotationally  
**step_height:** How much the legs will raise themselves to not collide with the ground while rotating themselves forwards to get ready for the next push backwards.  
**speed:** Speed of the leg movements

### update_camera(robot_id, camera_distance=2)
**Description:** Updates the camera position and orientation to follow the robot. The camera aligns with the robot's yaw (rotation around the z-axis) and maintains a specified distance.  
**robot_id:** urdf of the robot  
**camera_distance:** distance of the camera from the robot  
**Note:**  
**cameraYaw** inside the body of the function can be given the value **p.getDebugVisualizerCamera()[8]** to make the camera freely rotatable using control + mouse1, or it can be given the value **camera_yaw + 90** so that the camera always follows the back of the robot.
