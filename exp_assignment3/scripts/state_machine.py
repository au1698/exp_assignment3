#!/usr/bin/env python
# -*- coding: iso-8859-15 -*-

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

import exp_assignment3.msg

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

from exp_assignment3.msg import PlanningAction, PlanningGoal

# Imports file .action ed and messages used by move base action 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

global see_ball_green, see_ball_red, see_ball_black, see_ball_yellow, see_ball_blue, see_ball_magenta

global reach_green_ball, reach_red_ball, reach_black_ball, reach_yellow_ball, reach_blue_ball, reach_magenta_ball

reach_green_ball = False
reach_black_ball = False
reach_red_ball = False
reach_yellow_ball = False
reach_blue_ball = False
reach_magenta_ball = False

VERBOSE = False 
# FUNCTIONS 

#  Move-Base client that lets the robot go home avoiding obstacles
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
    goal.target_pose.pose.position.x = 0.0
    goal.target_pose.pose.position.y = 0.0
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

     # guarda in giro se ci sono palle
     # subscribe_camera = rospy.Subscriber("robot/camera1/image_raw/compressed", CompressedImage, callback_camera,  queue_size=1)

     # Waits for the server to finish performing the action.
     wait = client.wait_for_result() 
     
     # Se il risultato non arriva, assumiamo che il Server non sia disponibile
     if not wait:
         rospy.logerr("Action server not available!")
         rospy.signal_shutdown("Action server not available!")
     else:
    # Restituisce il risultato dell'esecuzione dell'action
         return client.get_result()   


# PUBLISHERS
vel_pub = rospy.Publisher("/cmd_vel",Twist, queue_size=1)

# Track the ball function
# inputs: centroid radius,x,y,center,image_np - IT'S CALLED BY THE CALLBACK_CAMERA
def Track_ball(radius,x,y,image_np,center):
     if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
         cv2.circle(image_np, (int(x), int(y)), int(radius),
                           (0, 255, 255), 2)
         cv2.circle(image_np, center, 5, (0, 0, 255), -1)  
         vel = Twist()   # create object velocity of type "TWIST"
         vel.angular.z = 0.002*(center[0]-400)  # set the angular velocity
         vel.linear.x = 0.01*(radius-100)      # set the linear velocity
         # publish the velocity on the topic /cmd
         vel_pub.publish(vel) 
     return               

# CALLBACK- FUNCTIONS 

# THIS FUNCTION DETECTS ALL THE BALLS
def callback_camera(ros_data):
     #rospy.logerr("CALLBACK CAMERA IS EXECUTED")
     #global see_ball           # GLOBAL VARIABLE, IS SET TO TRUE WHEN A BALL IS DETECTED 
    
     '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
     if VERBOSE:
         print ('received image of type: "%s"' % ros_data.format)

     #### direct conversion to CV2 ####
     np_arr = np.fromstring(ros_data.data, np.uint8)
     image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:

     

     if len(image_np) == 0:
         rospy.loginfo('sono qui')
         print('the array is empty')
     
     # mettere colori delle altre palle 

     # Colours of the balls 
     GreenLower = (50, 50, 20)   # range to detect green ball 
     GreenUpper = (70, 255, 255) 
     BlackLower = (0, 0, 0)      # black ball 
     BlackUpper = (5, 50, 50)
     RedLower = (0, 50, 50)      # red ball 
     RedUpper = (5, 255, 255)
     YellowLower = (25, 50, 50)  # yellow ball 
     YellowUpper = (35, 255, 255)
     BlueLower = (100, 50, 50)   # blue ball 
     BlueUpper = (130, 255, 255)
     MagentaLower = (125, 50, 50) # magenta ball 
     MagentaUpper = (150, 255, 255)

     
     blurred = cv2.GaussianBlur(image_np, (11, 11), 0)  # apply a Gaussian filter
     hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

     # Create a mask for each colour 

     # Mask for green ball 
     mask_green = cv2.inRange(hsv, GreenLower, GreenUpper)
     mask_green = cv2.erode(mask_green, None, iterations=2)
     mask_green = cv2.dilate(mask_green, None, iterations=2)

      # Mask for Black ball 
     mask_black = cv2.inRange(hsv, BlackLower, BlackUpper)
     mask_black = cv2.erode(mask_black, None, iterations=2)
     mask_black = cv2.dilate(mask_black, None, iterations=2)

     # Mask for Red ball 
     mask_red = cv2.inRange(hsv, RedLower, RedUpper)
     mask_red = cv2.erode(mask_red, None, iterations=2)
     mask_red = cv2.dilate(mask_red, None, iterations=2)

     # Mask for Yellow ball 
     mask_yellow = cv2.inRange(hsv, YellowLower, YellowUpper)
     mask_yellow = cv2.erode(mask_yellow, None, iterations=2)
     mask_yellow = cv2.dilate(mask_yellow, None, iterations=2)

     # Mask for Blue ball 
     mask_blue = cv2.inRange(hsv, BlueLower, BlueUpper)
     mask_blue = cv2.erode(mask_blue, None, iterations=2)
     mask_blue = cv2.dilate(mask_blue, None, iterations=2)

     # Mask for Magenta ball 
     mask_magenta = cv2.inRange(hsv, MagentaLower, MagentaUpper)
     mask_magenta = cv2.erode(mask_magenta, None, iterations=2)
     mask_magenta = cv2.dilate(mask_magenta, None, iterations=2)
    
     # Find the contours of each ball 

     #cv2.imshow('mask', mask)
     cnts_green = cv2.findContours(mask_green.copy(), cv2.RETR_EXTERNAL, 
                                cv2.CHAIN_APPROX_SIMPLE)                    # Contour green ball 
     cnts_green = imutils.grab_contours(cnts_green) 

     cnts_black = cv2.findContours(mask_black.copy(), cv2.RETR_EXTERNAL, 
                                cv2.CHAIN_APPROX_SIMPLE)                    # Contour black ball 
     cnts_black = imutils.grab_contours(cnts_black)   

     cnts_red = cv2.findContours(mask_red.copy(), cv2.RETR_EXTERNAL, 
                                cv2.CHAIN_APPROX_SIMPLE)                    # Contour red ball 
     cnts_red = imutils.grab_contours(cnts_red) 

     cnts_yellow = cv2.findContours(mask_yellow.copy(), cv2.RETR_EXTERNAL, 
                                cv2.CHAIN_APPROX_SIMPLE)                    # Contour yellow ball 
     cnts_yellow = imutils.grab_contours(cnts_yellow) 

     cnts_blue = cv2.findContours(mask_blue.copy(), cv2.RETR_EXTERNAL, 
                                cv2.CHAIN_APPROX_SIMPLE)                    # Contour blue ball 
     cnts_blue = imutils.grab_contours(cnts_blue) 

     cnts_magenta = cv2.findContours(mask_magenta.copy(), cv2.RETR_EXTERNAL, 
                                cv2.CHAIN_APPROX_SIMPLE)                    # Contour magenta ball 
     cnts_magenta = imutils.grab_contours(cnts_magenta) 

     radius = 0
     #c = 0
     center = None

     global reach_green_ball, reach_red_ball, reach_black_ball, reach_yellow_ball, reach_blue_ball, reach_magenta_ball


     # only proceed if at least one contour was found 
     #  
     if len(cnts_green) > 0:  # if the green ball is detected 

         see_ball_green = True  # the robot can see the GREEN ball 

         # find the largest contour in the mask, then use
         # it to compute the minimum enclosing circle and
         # centroid 
         rospy.logerr("the image is acquired")
         c = max(cnts_green, key=cv2.contourArea)  
         ((x, y), radius) = cv2.minEnclosingCircle(c)
         M = cv2.moments(c)
         center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

         # only proceed if the radius meets a minimum size
         # if radius > 10:

             # draw the circle and centroid on the frame,
             # then update the list of tracked points
         #cv2.circle(image_np, (int(x), int(y)), int(radius),
                           #(0, 255, 255), 2)
         #cv2.circle(image_np, center, 5, (0, 0, 255), -1) 
         rospy.loginfo('GREEN BALL IS DETECTED') 
         # se non ha ancora raggiunto la ball
         if (see_ball_green == True and reach_green_ball == False ):   
                 Track_ball(radius,x,y,image_np,center)  # fa il track della palla
                 rospy.loginfo('ENTER IN THE SUB-STATE TRACK')
                 reach_green_ball = True  #  QUESTA VARIABILE è TRUE SOLO QUANDO HO GIà RAGGIUNTO LA PALLA
         else: 
             rospy.loginfo('GREEN BALL ALREADY REACHED')

         # Track_ball(radius,x,y,image_np,center)      # TRACK FUNCTION (la prima volta fa il track)   

     elif len (cnts_black) > 0:   

         see_ball_black = True  # the robot can see the BLACK ball 
         c = max(cnts_black, key=cv2.contourArea)  
         ((x, y), radius) = cv2.minEnclosingCircle(c)
         M = cv2.moments(c)
         center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

         #cv2.circle(image_np, (int(x), int(y)), int(radius),
                           #(0, 255, 255), 2)
         #cv2.circle(image_np, center, 5, (0, 0, 255), -1) 
         rospy.loginfo('BLACK BALL IS DETECTED')

         if (see_ball_black == True and reach_black_ball == False ):   # se non ha ancora raggiunto la ball
                 Track_ball(radius,x,y,image_np,center)  # fa il track della palla
                 rospy.loginfo('ENTER IN THE SUB-STATE TRACK')
                 reach_black_ball = True  #  QUESTA VARIABILE è TRUE SOLO QUANDO HO GIà RAGGIUNTO LA PALLA
         else: 
             rospy.loginfo('BLACK BALL ALREADY REACHED')

     elif len(cnts_red) > 0:   

         see_ball_red = True  # the robot can see the BLACK ball 
         c = max(cnts_red, key=cv2.contourArea)  
         ((x, y), radius) = cv2.minEnclosingCircle(c)
         M = cv2.moments(c)
         center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

         #cv2.circle(image_np, (int(x), int(y)), int(radius),
                          # (0, 255, 255), 2)
         #cv2.circle(image_np, center, 5, (0, 0, 255), -1) 
         rospy.loginfo('RED BALL IS DETECTED')

         if (see_ball_red == True and reach_red_ball == False ):   # se non ha ancora raggiunto la ball
                 Track_ball(radius,x,y,image_np,center)  # fa il track della palla
                 rospy.loginfo('ENTER IN THE SUB-STATE TRACK')
                 reach_red_ball = True  #  QUESTA VARIABILE è TRUE SOLO QUANDO HO GIà RAGGIUNTO LA PALLA
         else: 
             rospy.loginfo('RED BALL ALREADY REACHED')

     elif len(cnts_yellow) > 0:   

         see_ball_yellow = True  # the robot can see the BLACK ball 
         c = max(cnts_yellow, key=cv2.contourArea)  
         ((x, y), radius) = cv2.minEnclosingCircle(c)
         M = cv2.moments(c)
         center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

         #cv2.circle(image_np, (int(x), int(y)), int(radius),
                           #(0, 255, 255), 2)
         #cv2.circle(image_np, center, 5, (0, 0, 255), -1) 
         rospy.loginfo('YELLOW BALL IS DETECTED')

         if (see_ball_yellow== True and reach_yellow_ball == False ):   # se non ha ancora raggiunto la ball
                 Track_ball(radius,x,y,image_np,center)  # fa il track della palla
                 rospy.loginfo('ENTER IN THE SUB-STATE TRACK')
                 reach_yellow_ball = True  #  QUESTA VARIABILE è TRUE SOLO QUANDO HO GIà RAGGIUNTO LA PALLA
         else: 
             rospy.loginfo('YELLOW BALL ALREADY REACHED')

     elif len(cnts_blue) > 0:   

         see_ball_blue = True  # the robot can see the BLACK ball 
         c = max(cnts_blue, key=cv2.contourArea)  
         ((x, y), radius) = cv2.minEnclosingCircle(c)
         M = cv2.moments(c)
         center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

         #cv2.circle(image_np, (int(x), int(y)), int(radius),
                           #(0, 255, 255), 2)
         #cv2.circle(image_np, center, 5, (0, 0, 255), -1) 
         rospy.loginfo('BLUE BALL IS DETECTED')   # MI HA VISTO LA BLUE BALL OK!!! 

         if (see_ball_blue == True and reach_blue_ball == False ):   # se non ha ancora raggiunto la ball
                 Track_ball(radius,x,y,image_np,center)  # fa il track della palla
                 rospy.loginfo('ENTER IN THE SUB-STATE TRACK')
                 reach_blue_ball = True  #  QUESTA VARIABILE è TRUE SOLO QUANDO HO GIà RAGGIUNTO LA PALLA
         else: 
             rospy.loginfo('BLUE BALL ALREADY REACHED')

     elif len(cnts_magenta) > 0:   

         see_ball_magenta = True  # the robot can see the BLACK ball 
         c = max(cnts_magenta, key=cv2.contourArea)  
         ((x, y), radius) = cv2.minEnclosingCircle(c)
         M = cv2.moments(c)
         center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

         #cv2.circle(image_np, (int(x), int(y)), int(radius),
                           #(0, 255, 255), 2)
         #cv2.circle(image_np, center, 5, (0, 0, 255), -1) 
         rospy.loginfo('MAGENTA BALL IS DETECTED')

         if (see_ball_magenta == True and reach_magenta_ball == False):   # se non ha ancora raggiunto la ball
                 Track_ball(radius,x,y,image_np,center)  # fa il track della palla
                 rospy.loginfo('ENTER IN THE SUB-STATE TRACK')
                 reach_magenta_ball = True  #  QUESTA VARIABILE è TRUE SOLO QUANDO HO GIà RAGGIUNTO LA PALLA
         else: 
             rospy.loginfo('MAGENTA BALL ALREADY REACHED')
       

     cv2.imshow('window', image_np)
     cv2.waitKey(2)

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
        
        #rospy.loginfo('Executing state SLEEP')
        # userdata.sleep_counter_out = userdata.sleep_counter_in + 1  
        
        #result = Go_home()
        #if result:
             #rospy.loginfo('robot arrived at home')
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

        global see_ball_green, see_ball_black, see_ball_red, see_ball_yellow, see_ball_blue, see_ball_magenta 

        # FARE CICLO WHILE 
        while True: 
            subscribe_camera = rospy.Subscriber("/camera1/image_raw/compressed",CompressedImage, callback_camera,  queue_size=1)
            Move_normal()  #  the robot moves randomly
            rospy.loginfo('The robot moves randomly')
             # Subscribe to camera topic and execute the callbacl_camera function

              

                 # creo variabile
                 # ball_colour = "green" - LABEL 
                 # faccio un publish ad al nodo "track_ball" 
            time.sleep(50)
            break

                 # Track-sub state, chiamo una funzione di TRACKING a cui passo come argomento la variabile TRUE
                 # poi vedo se questa è TRUE, faccio il tracking della palla del colore selezionato. 
                  
        # fare il CHECK SEMPRE, se la variabile  stata già detectata OK, altrimenti -> TRACK   
        # in cui ti avvicinii all'oggetto, fai lo STORE dell'Odom. 

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
    ## Inizialize ros node ''state_machine'
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

