# pybullet-RL-walking-model

URDF creator website: https://marksoulier.github.io/URDF_creator/  
F TO GO RIGHT           
S TO GO LEFT    
E TO GO FORWARDS  
D TO GO BACKWARDS  

K TO SPAWN BOX ON THE RIGHT           
H TO SPAWN BOX ON THE LEFT    
U TO SPAWN BOX IN THE FRONT  
J TO SPAWN BOX IN THE BACK  

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
### calculate_angle_and_distance(robot_id, box_id)
**Description:** Calculates the angle and distance between the robot and the goal box and prints these values to the console.  
**robot_id:** urdf of the robot  
**box_id:** urdf of the box  
**Note:**  
The line at the end of the function can be un-commented to also return the angle and distance values so that these values can be used for point calculation or anything else.

### control_robot_with_keys(robot_id)
**Description:** Controls the robot's movements and the direction of the goal box's spawn according to the keybinds given at the beginning of this README file.  
**robot_id:** urdf of the robot  

### do_nothing(robot_id)
**Description:** The robot does not perform any movement with its legs. The purpose of this function is to keep the simulation running when no keys are pressed, called by **control_robot_with_keys**.  
**robot_id:** urdf of the robot  

### move_joints(robot_id, prismatic_positions, rotational_positions, rotational_force, prismatic_force = 50)
**Description:** Controls the movement of both prismatic and rotational joints on the robot. It sets target positions and forces for each joint type.  
**robot_id:** urdf of the robot  
**prismatic_positions:** List of target positions (elevation relative to the z-axis) for the prismatic joints of the robot’s legs  
**rotational_positions:** List of target positions (angel of the legs relative to the z-axis) for the rotational joints of the robot’s legs  
**rotational_force:** Force that will be applied to the rotational joints  
**prismatic_force:** Force that will be applied to the prismatic joints  

### perform_trot_gait(robot_id, step_length=0.3, step_height=0.1, speed=0.1)
**Description:** Performs the forwards and backwards motion of the robot by making use of the **move_joints** function.  
**robot_id:** urdf of the robot  
**step_length:** How much the legs will reach forwards and backwards rotationally. The robot will move forwards if this is given a positive value and backwards if it is given a negative value.  
**step_height:** How much the legs will raise themselves to not collide with the ground while rotating themselves forwards to get ready for the next push backwards.  
**speed:** Speed of the leg movements

### rotate(robot_id, direction, step_height=0.2, rotation_angle=math.pi / 4, speed=0.2, steps_per_cycle = 5)
**Description:** Performs the rotational movement of the robot by making use of the **move_joints** function.  
**robot_id:** urdf of the robot  
**direction:** True ---> Rotate right, False ---> Rotate left
**step_height:** How much the legs will push the ground.  
**rotation_angle:** How much the legs will rotate.  
**speed:** Speed of the leg movements  
**steps_per_cycle:** How long the legs will be stationary before performing the next rotational movement cycle. This is an alternate way to control the speed of the rotation than the **speed** parameter. This value prevents the legs from trying to rotate prematurely before they have made contact with the ground after the previous rotational cycle.

### spawn_box_in_direction_of_motion(robot_id, distance_ahead=1.0, side_offset = 0)
**Description:** Spawns the goal box in the desired location relative the the robot.  
**robot_id:** urdf of the robot  
**distance_ahead:** The desired forward distance of the box relative to the robot, in the direction the robot is facing.  
**side_offset:** The desired horizontal distance of the box from the robot, aligned with the direction the robot is facing.

### update_camera(robot_id, camera_distance=2)
**Description:** Updates the camera position and orientation to follow the robot. The camera aligns with the robot's yaw (rotation around the z-axis) and maintains a specified distance.  
**robot_id:** urdf of the robot  
**camera_distance:** distance of the camera from the robot  
**Note:**  
**cameraYaw** inside the body of the function can be given the value **p.getDebugVisualizerCamera()[8]** to make the camera freely rotatable using control + mouse1, or it can be given the value **camera_yaw + 90** so that the camera always follows the back of the robot.
