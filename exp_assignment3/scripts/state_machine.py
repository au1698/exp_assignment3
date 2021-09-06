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


# Imports file .action ed and messages used by move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Global variables

global see_ball_green, see_ball_red, see_ball_black, see_ball_yellow, see_ball_blue, see_ball_magenta

global reach_green_ball, reach_red_ball, reach_black_ball, reach_yellow_ball, reach_blue_ball, reach_magenta_ball

global FLAG_BLU, FLAG_MAGENTA, FLAG_BLACK, FLAG_YELLOW, FLAG_GREEN, FLAG_RED

FLAG_BLU = 1 
FLAG_BLACK =1 
FLAG_GREEN =1
FLAG_MAGENTA =1
FLAG_RED = 1
FLAG_YELLOW = 1


# Variables inizialization 

reach_green_ball = False
reach_black_ball = False
reach_red_ball = False
reach_yellow_ball = False
reach_blue_ball = False
reach_magenta_ball = False

see_ball_black = False
see_ball_green = False
see_ball_red = False
see_ball_yellow = False
see_ball_blue = False
see_ball_magenta = False


location = [['LivingRoom', 'black', 0, 0], ['Kitchen', 'magenta', 0, 0], ['Closet', 'yellow', 0, 0], [
    'Entrance', 'blue', 0, 0], ['Bathroom', 'red', 0, 0], ['Bedroom', 'green', 0, 0]]

VERBOSE = False

# FUNCTIONS

#  Move-Base client that lets the robot go home avoiding obstacles

#  Move-Base client that lets the robot move normal

global result
global ret 
ret = String()
global arrived 

# Functions 


# It takes as input x,y position and performs and action client to let the robot move towards the target with obstacle avoidance.
# It also subscribes to the topic /human_command. If the message is = "play" it cancels the goal and switches to the PLAY state. 

# This function is used in the SLEEP state (x=0, y=0) and NORMAL state with random x,y. 

def Move_normal(target_x, target_y):
    global see_ball_black, see_ball_blue, see_ball_green, see_ball_magenta, see_ball_red, see_ball_yellow
    global robot_position 
    global user_command 
    global ret 

    user_command = rospy.Subscriber("human_command", String, callback_user)
     
    rospy.loginfo("MOVE NORMAL")

    robot_position_x = robot_position.x
    robot_position_y = robot_position.y
    
    # Create an action client  called "move_base" with action definition file "MoveBaseAction"
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
    client.wait_for_server()

    # Create a new goal through the costructor MoveBaseGoal
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = target_x
    goal.target_pose.pose.position.y = target_y
    goal.target_pose.pose.orientation.w = 1.0

    # Sends the goal to the action server.
    client.send_goal(goal)
    print(goal)
    
    if(user_command == "play"):
        rospy.loginfo("GOAL CANCELED")
        ret = "play"
        return client.cancel_goal()     

    # If I see the ball, cancel the goal 
    if (see_ball_blue == True):          
        rospy.loginfo("GOAL CANCELED")
        return client.cancel_goal()
    if (see_ball_green == True): 
        rospy.loginfo("GOAL CANCELED")
    if (see_ball_black == True): 
        rospy.loginfo("GOAL CANCELED")
        return client.cancel_goal()
    if (see_ball_red == True): 
        rospy.loginfo("GOAL CANCELED")
        return client.cancel_goal()
    if (see_ball_yellow == True): 
        rospy.loginfo("GOAL CANCELED")
        return client.cancel_goal()
    if (see_ball_magenta == True): 
        rospy.loginfo("GOAL CANCELED")
        return client.cancel_goal() 

    # Waits for the server to finish performing the action.
    wait = client.wait_for_result()

    # If no result, the server is not available
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   


# It takes as input x,y position and performs and action client to let the robot move towards the location of the person with obstacle avoidance.
# This function is used in the PLAY state (x=-5, y=8)

def Move_Play(targ_x, targ_y):

    global robot_position 
    global user_command 

    rospy.loginfo("MOVE PLAY")

    robot_position_x = robot_position.x
    robot_position_y = robot_position.y
    
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        
    client.wait_for_server()

    # Create a new goal 
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = targ_x
    goal.target_pose.pose.position.y = targ_y
    goal.target_pose.pose.orientation.w = 1.0

    # Sends the goal to the action server.
    client.send_goal(goal)
    print(goal)
  
    # Waits for the server to finish performing the action.
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
    # Result of executing the action
        return client.get_result()   

# This function Tracks the ball and it's called in the NORAMAL state. When the robot reaches the ball, it saves in a python list its location and the 
# colour of the ball. 
# It has as inputs: centroid radius, x, y, center, image_np which are given by the callback of the camera and are used to track the ball.  

def Track_ball(radius, x, y, image_np, center, colour):
    global reach_blue_ball,reach_black_ball,reach_green_ball,reach_magenta_ball,reach_red_ball,reach_yellow_ball
    global see_ball_black, see_ball_blue, see_ball_green, see_ball_magenta, see_ball_red, see_ball_yellow
    
    if radius > 10 and radius < 80:
        # draw the circle and centroid on the frame,
        # then update the list of tracked points
        cv2.circle(image_np, (int(x), int(y)), int(radius),
                   (0, 255, 255), 2)
        cv2.circle(image_np, center, 5, (0, 0, 255), -1)

        # Create an object of type Twist
        vel = Twist()   
        vel.angular.z = -0.002*(center[0]-400)  # set the angular velocity
        vel.linear.x = -0.01*(radius-100)       # set the linear velocity
        print(radius, center[0])

        # publish the velocity on the topic /cmd
        vel_pub.publish(vel)
        rospy.loginfo('ENTER IN TRACK MODE ')

    elif radius > 80: 
        rospy.loginfo('radius > 80 ')
        cv2.circle(image_np, (int(x), int(y)), int(radius),
                   (0, 255, 255), 2)
        cv2.circle(image_np, center, 5, (0, 0, 255), -1)
        vel = Twist() 
        vel.angular.z = 0 
        vel.linear.x = 0 
        vel_pub.publish(vel)

        rospy.loginfo('ARRIVED TO DESTINATION')
        
        if (colour == "green"):
            reach_green_ball = True
            # save robot position and the color
            location[5][2] = robot_position.x
            location[5][3] = robot_position.y
            see_ball_green == False 
        elif (colour == "blue"):
            reach_blue_ball = True
            rospy.loginfo("COLORE BLU")
            location[3][2] = robot_position.x
            location[3][3] = robot_position.y
            see_ball_blue = False
        elif (colour == "red"):
            reach_red_ball = True
            location[4][2] = robot_position.x
            location[4][3] = robot_position.y
            see_ball_red == False
        elif (colour == "black"):
            reach_black_ball = True
            location[0][2] = robot_position.x
            location[0][3] = robot_position.y
            see_ball_black == False
        elif (colour == "yellow"):
            reach_yellow_ball = True
            location[2][2] = robot_position.x
            location[2][3] = robot_position.y
            see_ball_yellow == False
        elif (colour == "magenta"):
            reach_magenta_ball = True
            location[1][2] = robot_position.x
            location[1][3] = robot_position.y
            see_ball_magenta == False

        rospy.loginfo('stored position')
        return 

# CALLBACK- FUNCTIONS



 radius = 0
 center = None


# PUBLISHERS
vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)   


def callback_camera(ros_data):
    global see_ball_black,see_ball_blue,see_ball_green,see_ball_magenta,see_ball_red,see_ball_yellow
    global reach_black_ball, reach_blue_ball, reach_green_ball, reach_magenta_ball, reach_red_ball, reach_yellow_ball
    '''Callback function of subscribed topic.
       Here images get converted and features detected'''
    if VERBOSE:
        print('received image of type: "%s"' % ros_data.format)

    #### direct conversion to CV2 ####
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # OpenCV >= 3.0:

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
    MagentaLower = (125, 50, 50)  # magenta ball
    MagentaUpper = (150, 255, 255)

    # apply a Gaussian filter
    blurred = cv2.GaussianBlur(image_np, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

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

    #global reach_green_ball, reach_red_ball, reach_black_ball, reach_yellow_ball, reach_blue_ball, reach_magenta_ball
    # only proceed if at least one contour was found

    if len(cnts_green) > 0:  # if the green ball is detected

        see_ball_green = True  # the robot can see the GREEN ball

        # find the largest contour in the mask, then use
        # it to compute the minimum enclosing circle and
        # centroid
        c = max(cnts_green, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # se non ha ancora raggiunto la ball
        if (see_ball_green == True and reach_green_ball == False):
            # fa il track della palla
            Track_ball(radius, x, y, image_np, center, "green")

    elif len(cnts_black) > 0:

        see_ball_black = True  # the robot can see the BLACK ball
        c = max(cnts_black, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        if (see_ball_black == True and reach_black_ball == False):   # se non ha ancora raggiunto la ball
            # fa il track della palla
            Track_ball(radius, x, y, image_np, center, "black")

    elif len(cnts_red) > 0:

        see_ball_red = True  # the robot can see the BLACK ball
        c = max(cnts_red, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        if (see_ball_red == True and reach_red_ball == False):   # se non ha ancora raggiunto la ball
            # fa il track della palla
            Track_ball(radius, x, y, image_np, center, "red")

    elif len(cnts_yellow) > 0:

        see_ball_yellow = True  # the robot can see the BLACK ball
        c = max(cnts_yellow, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # se non ha ancora raggiunto la ball
        if (see_ball_yellow == True and reach_yellow_ball == False):
            # fa il track della palla
            Track_ball(radius, x, y, image_np, center, "yellow")

    elif len(cnts_blue) > 0:

        see_ball_blue = True  
        c = max(cnts_blue, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])) 
        #print(reach_blue_ball)

        if (see_ball_blue == True and reach_blue_ball == False):   # se non ha ancora raggiunto la ball
            # fa il track della palla
            Track_ball(radius, x, y, image_np, center, "blue")      
    

    elif len(cnts_magenta) > 0:

        see_ball_magenta = True  # the robot can see the BLACK ball
        c = max(cnts_magenta, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        M = cv2.moments(c)
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

        # se non ha ancora raggiunto la ball
        if (see_ball_magenta == True and reach_magenta_ball == False):
            # fa il track della palla
            Track_ball(radius, x, y, image_np, center, "magenta")

    cv2.imshow('window', image_np)
    cv2.waitKey(2)
    #return

user_command = String()

# Callback for the topic /human_command used to interacts with the user (script: 'human_command.py')

def callback_user(msg):
    global user_command
    user_command = msg.data
    #rospy.loginfo(" %s",msg.data)

    return user_command

robot_position = Point()

# Callback for the topic /odom used to retrieve robot's position over time

def callback_odom(data):

    global robot_position
    robot_position.x = data.pose.pose.position.x
    robot_position.y = data.pose.pose.position.y 
    # print(robot_position_x)
       

# STATE MACHINE


# SLEEP state

class Sleep(smach.State):
    # Constructor of the class Sleep
    def __init__(self):
        # Initialization function
        smach.State.__init__(self,
                             outcomes=['wake_up'],
                             input_keys=['sleep_counter_in'],
                             output_keys=['sleep_counter_out'])

    def execute(self, userdata):

        rospy.loginfo('Executing state SLEEP')
        x = 0.0 
        y = 0.0   
        result = Move_normal(x,y)
        if result:
             rospy.loginfo('robot arrived at home')
             time.sleep(10)    # it stays for some time
 
        # Change state: from 'SLEEP' to 'NORMAL'
        return 'wake_up'

# NORMAL state

class Normal(smach.State):
    global user_command,ret 
    user_command = String()
    # Constructor of the class Normal
    def __init__(self):
        # Initialization function
        smach.State.__init__(self,
                             outcomes=['go_to_play'],
                             input_keys=['normal_counter_in'],
                             output_keys=['normal_counter_out'])

    def execute(self, userdata):
        time.sleep(5)
        rospy.loginfo('Executing state NORMAL')

        while True:
            subscribe_camera = rospy.Subscriber(
                "/camera1/image_raw/compressed", CompressedImage, callback_camera,  queue_size=1)
            
            x = random.randrange(-5,6)
            y = random.randrange(-5,3)
            print(x,y)
            Move_normal(x, y)  # the robot moves randomly
            # Check if user said 'PLAY' subscribing to the topic /human_command
          
            user_command = user_command.data = rospy.Subscriber("human_command", String, callback_user)
            rospy.loginfo(" %s",user_command)

             # ret is a global variable from MoveNormal(), in this way if the user says PLAY while the robot is moving towards a random location 
             # it immediately switches to PLAY state. 

            if (user_command == "play" or ret == "play"): 
                rospy.loginfo(" USER COMMAND PLAY")
                return 'go_to_play'    

# PLAY state 

class Play(smach.State):
    # Constructor of the class Play
    def __init__(self):
        # Initialization function
        smach.State.__init__(self,
                             outcomes=['go_to_normal', 'find'],
                             input_keys=['play_counter_in'],
                             output_keys=['play_counter_out'])

    def execute(self, userdata):
        pos_x = 0
        pos_y = 0
        global arrived 
        global user_command
        robot_say = String()
        rospy.loginfo('Executing state PLAY')
        #user_command = rospy.Subscriber("human_command", String, callback_user)

        # move the robot towards the user location
        arrived = Move_Play(-5, 8) 
        rospy.loginfo("Arrived to the HUMAN")
        for i in location:
            print(i)
        if (arrived):
                
            robot_say = "arrived"
            pub_arrived = rospy.Publisher('human_command', String, queue_size=10) 

            # publish to the topic /human_command "arrived"
            pub_arrived.publish(robot_say)

            # It subscribes to the topic /human_command to listen to the location 
            go_to_command = rospy.Subscriber("/human_command", String, callback_user,  queue_size=1)

            if (go_to_command == "living_room"):
                if (reach_black_ball == True):
                    pos_x = location[0][2]
                    pos_y = location[0][3]
                    print(pos_x,pos_y)

            if (go_to_command == "kitchen"):
                if (reach_magenta_ball == True):
                    pos_x = location[1][2]
                    pos_y = location[1][3] 
                    print(pos_x,pos_y)
                else:
                    return 'find'
            if (go_to_command == "closet"):
                if (reach_yellow_ball == True):
                    pos_x = location[2][2]
                    pos_y = location[2][3]
                    print(pos_x,pos_y)
                else:
                    return 'find'
            if (go_to_command == "entrance"):
                if (reach_blue_ball == True):
                    pos_x = location[3][2]
                    pos_y = location[3][3]
                    print(pos_x,pos_y)
                else:
                    return 'find'
            if (go_to_command == "bathroom"):
                if (reach_red_ball == True):
                    pos_x = location[4][2]
                    pos_y = location[4][3]
                    print(pos_x,pos_y)
                else:
                    return 'find'
            if (go_to_command == "bedroom"):
                if (reach_green_ball == True):
                    pos_x = location[5][2]
                    pos_y = location[5][3]
                    print(pos_x,pos_y)
                else:
                    return 'find' 
    
        # Call MovePlay to let the robot reach the given location                
        Move_Play(pos_x, pos_y)
        time.sleep(90)
    

    # Change state randomly: from 'PLAY' to 'NORMAL'
        return random.choice(['go_to_normal', 'find'])


# FIND state: not implemented :( 
class Find (smach.State):
    # Constructor of the class Sleep
    def __init__(self):
        # Initialization function
        smach.State.__init__(self,
                             outcomes=['go_to_play'],
                             input_keys=['sleep_counter_in'],
                             output_keys=['sleep_counter_out'])

    def execute(self, userdata):

        rospy.loginfo('Executing state FIND')
        

        # Change state: from 'SLEEP' to 'NORMAL'
        return 'go_to_play'


def main():
    # Inizialize ros node ''state_machine'
    rospy.init_node('state_machine')
    rospy.Subscriber("odom", Odometry, callback_odom)
    user_command = user_command.data = rospy.Subscriber("human_command", String, callback_user)

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['container_interface'])
    sm.userdata.sm_counter = 0

    # Open state machine container
    with sm:
        # Add states to the container
        smach.StateMachine.add('SLEEP', Sleep(), transitions={
                               'wake_up': 'NORMAL'})
        smach.StateMachine.add('NORMAL', Normal(), transitions={
                               'go_to_play': 'PLAY'})

        smach.StateMachine.add('PLAY', Play(), transitions={
                               'go_to_normal': 'NORMAL', 'find': 'FIND'})

        smach.StateMachine.add('FIND', Find(), transitions={
                               'go_to_play': 'PLAY'})

    # Create and start the introspection server for visualization
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute the state machine
    outcome = sm.execute()

    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()
