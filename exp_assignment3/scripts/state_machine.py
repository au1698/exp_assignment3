
# Python libs
import sys
import time
import math
import random

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

import imutils

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy
import smach
import smach_ros
import actionlib

import exp_assignment2.msg

# Ros Messages
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int64MultiArray
from std_msgs.msg import String
from gazebo_msgs.msg import LinkState
from tf import transformations
from std_msgs.msg import String, Float64 

from exp_assignment2.msg import PlanningAction, PlanningGoal

# Imports file .action ed and messages used by move base action 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# FUNCTIONS 

#  Move-Base client that lets the robot go home
def Go_home():

     # Crea un action client chiamato "move_base" con action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)


   # Aspetta che l'action si sia avviato ed abbia iniziato ad essere ricettivo per i goal
    client.wait_for_server()

   # Crea un nuovo goal con il costruttore MoveBaseGoal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
   # Muovere di 0.5 metri avanti lungo l'asse x del sistema di riferimento della mappa
    goal.target_pose.pose.position.x = 0
    goal.target_pose.pose.position.y = 0
    goal.target_pose.pose.orientation.w = 1.0

   # Invia il goal all'action server.
    client.send_goal(goal)
   # Aspetta che il server finisca di eseguire la richiesta
    wait = client.wait_for_result()
   # Se il risultato non arriva, assumiamo che il Server non sia disponibile
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Restituisce il risultato dell'esecuzione dell'action
        return client.get_result()   

#  Move-Base client that lets the robot move normal 
def Move_normal():
     
    # Crea un action client chiamato "move_base" con action definition file "MoveBaseAction"
     client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

     client.wait_for_server()

     # Crea un nuovo goal con il costruttore MoveBaseGoal
     goal = MoveBaseGoal()
     goal.target_pose.header.frame_id = "map"
     goal.target_pose.header.stamp = rospy.Time.now()

     goal.target_pose.pose.position.x = np.random.randint(1, 8)
     goal.target_pose.pose.position.y = np.random.randint(1, 8)
     goal.target_pose.pose.orientation.w = 1.0

     # Sends the goal to the action server.
     client.send_goal(goal)

     # Waits for the server to finish performing the action.
     wait = client.wait_for_result() 
     
     # Se il risultato non arriva, assumiamo che il Server non sia disponibile
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Restituisce il risultato dell'esecuzione dell'action
        return client.get_result()   


# SLEEP STATE -STATE MACHINE 

class Sleep(smach.State):
    ## Constructor of the class Sleep
    def __init__(self):       
    ## Initialization function 
        smach.State.__init__(self, 
                        outcomes=['wake_up'],
                        input_keys=['sleep_counter_in'],
                        output_keys=['sleep_counter_out'])


    def execute(self,userdata):
        
        rospy.loginfo('Executing state SLEEP')
        # userdata.sleep_counter_out = userdata.sleep_counter_in + 1  
        
        result = Go_home()
        if result:
             rospy.loginfo('robot arrived at home')
             time.sleep(10)    # it stays for some time  
        
              

    ## Change state: from 'SLEEP' to 'NORMAL'  
             return 'wake_up'  

## Define state Normal
class Normal(smach.State):
    ## Constructor of the class Normal
    def __init__(self):     
    ## Initialization function 
        smach.State.__init__(self, 
                        outcomes=['go_to_play'],
                        input_keys=['normal_counter_in'],
                        output_keys=['normal_counter_out'])

    def execute(self,userdata):
        time.sleep(5)
        rospy.loginfo('Executing state NORMAL')
        # userdata.normal_counter_out = userdata.normal_counter_in + 1  
        Move_normal()
        rospy.loginfo('The robot moves randomly')


        # Track-sub state 


        return 'go_to_play'  
                                
## Define state Play 
class Play(smach.State):
    time.sleep(5)
    global move_head
    ## Constructor of the class Play
    def __init__(self):       
    ## Initialization function   
        smach.State.__init__(self, 
                         outcomes=['go_to_normal','go_to_sleep'],
                         input_keys=['play_counter_in'],
                         output_keys=['play_counter_out'])

    def execute(self,userdata):
        
        rospy.loginfo('Executing state PLAY')
        # userdata.play_counter_out = userdata.play_counter_in + 1
        # To track the data  
        time.sleep(5)
        
    ## Change state randomly: from 'PLAY' to 'NORMAL' 
        return random.choice([ 'go_to_normal','go_to_sleep'])

    
              
def main():    
    ## Inizialize ros node ''pet_state_machine'
    rospy.init_node('state_machine')   
    
    ## Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])   
    sm.userdata.sm_counter = 0
    
    ## Open state machine container
    with sm:
         ## Add states to the container
         smach.StateMachine.add('SLEEP', Sleep(),
                                  
                               transitions={'wake_up':'NORMAL'},
                                                                
                                            
                               remapping={'sleep_counter_in':'sm_counter', 
                                         'sleep_counter_out':'sm_counter'})
         smach.StateMachine.add('NORMAL', Normal(), 
                               transitions={'go_to_play' :'PLAY'},
                                            
                                            
                               remapping={'normal_counter_in':'sm_counter',
                                          'normal_counter_out':'sm_counter'})

         smach.StateMachine.add('PLAY', Play(), 
                               transitions={'go_to_normal':'NORMAL',
                                             'go_to_sleep' : 'SLEEP'}, 
                                            
                                            
                               remapping={'play_counter_in':'sm_counter',
                                          'play_counter_out':'sm_counter'})
                                



    ## Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    ## Execute the state machine
    outcome = sm.execute()

    ## Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
     main()

