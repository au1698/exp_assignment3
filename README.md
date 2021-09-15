# exp_assignment3

## Behavioral Architecture 
The robot moves in an environment divided into 6 rooms and in each of them there is a ball of a different colour. The robot simulates a dog that moves on two wheels with differential drive control. It perceives has the perception of the environment through a hokuyo laser sensor and a RGBD camera. 
The robot implements four behaviors: sleep, normal, play and find. The human can interact by sending a Play command to the robot, followed by a GoTocommand + target location (Entrance,Closet, Living room, Kitchen, Bathroom, Bedroom). 

In this project the ROS Melodic is used and scripts are written in Python 2.7. 


<p align="center"> 
<img src=https://github.com/au1698/exp_assignment3/blob/main/exp_assignment3/Images/environment.png raw=true">
</p> 
                                                                                                             
 Each colour is associated to a different room:

- Blue: Entrance 
- Red: Closet 
- Green: Living room 
- Yellow: Kitchen 
- Orange: Bathroom 
- Black: Bedroom
                                                                                                            
                                                                                                                   
## Ros Architecture of the System 

## human_command.py 
This node simulates user's commands like "PLAY"  and "GO TO TARGET", where target are the rooms of the house such as 'kitchen', 'bathroom', 'closet', 'bedroom', 'living_room', 'entrance'. The command PLAY is published every 2 minutes (it's possible to set this value) on the topic /human_command. 
This node also subscribes on the topic /human_command and when the message is "arrived" means that the robot has reached the person, and is ready to 
listen to the user's command GO TO "LOCATION".
At this point, in the PLAY state of the state machine, if the location is known, the robot moves to the target avoiding obstacles otherwise
it enters in the FIND state. 

## state_machine.py
This node is a finite state machine made of three states: PLAY, SLEEP, NORMAL,PLAY,FIND.

SLEEP: The robot reaches a predefined location in the house, it stays there for some times, and when it switches in the Normal behavior. To implement this behavior, I created a function MoveNormal() that takes as input x,y coordinates and creates an action client to let the robot move to the target. In this case, the target is x=0 and y=0. Moreover, if the human gives the command PLAY, the request to the server is canceled. 

NORMAL: the robot moves randomly in different locations of the house. To implement this behavior, as for the sleep state I used the the MoveNormal() function, with random x and y coordinates. If the human gives the command PLAY it switches to the PLAY state. Moreover, the robot subscribes to the topic /camera1/image_raw with "callback_camera" as callback function. 
In this function, image processing is applied (Gaussian filter, mask, finding the contours). When the ball is detected, a global variable "see_ball" is set to TRUE while the variable "reach_ball" is FALSE, this means that the robot has never reached the ball while exploring . Under this condition the robot enters in the substate TRACK. Obviously, this logic is applied to every colored ball.    
 
SUBSTATE-TRACK: This substate is represented by a function Track_ball(). When the robot enters in this substate, you can see the ball on the window that contains the camera output with red centroid. Then, the robot adjust its linear and angular velocity to reach the ball, publishing on the topic /cmd. When the robot is too close to the ball and is considered ARRIVED TO DESTINATION, the global variable "reach_ball" is set to TRUE. Under this condition, the robot position is saved in a python list that contains the name of each location and the x,y position. In this way, the robot stores informations about rooms. Once done, the robot goes back to the Normal behavior and if it sees a colored ball that already has the position of, it no longer tries to reach it. 

PLAY: The robot goes to person's location x=-5, y=8, when it's arrived to the person, it publishes on the topic /human_command the message "arrived". In this way, the person knows it has arrived and can tell it to reach a specific location. Therefore, if the received command is a room and the the global variable "reach_ball" is TRUE, means that the robot has already stored position informations. So, it takes from the list the x and y coordinates and calls the function MovePlay() to let the robot move towards the target. 
If the location is unknown, the robot switches to the FIND state. 

FIND: The robot explores the environment in order to find the coloured ball using the explore_lite package. This part is to be implemented. 

## Ros Messages 

## How to run the code 

The first thing to do, after having cloned the repository in the Ros workspace, is to build the package in your workspace with
    ```
    catkin_make
    ```
To run the system:
    
    ```
    roslaunch exp_assignment3 simulation.launch 

    ```

## Working hypoteses 
The ROS packages used in this project are the following:
- Smach state machine to implement the switch between states
- Vision_opencv and cv_bridge to implement the camera vision 
- gmapping used to create a map of the environment 
- move_base as planner that let the robot reach the goal position with obstacle avoidance 

## Possible Improvements 
- Code optimization 
- Implement FIND state

## Author 
* Aurora Bertino: bertino.aurora16@gmail.com


