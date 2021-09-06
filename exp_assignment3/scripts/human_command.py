#! /usr/bin/python
# -*- coding: utf-8 -*-

import numpy as np
import roslib
import rospy
import actionlib
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped
from std_msgs.msg import Int64MultiArray
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkState
from std_msgs.msg import String
from tf import transformations
import math
import exp_assignment3.msg
import time
import random

from exp_assignment3.msg import PlanningAction, PlanningGoal

# Imports file .action ed and messages used by move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Human_command node

"""This node simulates human commands: PLAY or GO TO "LOCATION" 
   where location could be: kitchen, bathroom, closet, bedroom, living_room, entrance. 

   The command PLAY is published every 2 minutes (you can set this value) on the topic /human_command 

   This node subscribes on the same topic /human_command and when the message is "arrived" means that the robot reached the person, and is ready to 
   listent to the user's command GO TO "LOCATION".

   At this point, in the PLAY state of the state machine, if the lcoation is known, the robot moves to the target avoiding obstacles otherwise
   it enters in the FIND state. 

"""



def callback_robot(msg):
    data = msg.data
    if (msg.data == "arrived"):
         rospy.loginfo("The robot is arrived to the human")

def Human():
    pub = rospy.Publisher('human_command', String, queue_size=10)
    rospy.init_node('human_command', anonymous=True)
    human_command = String()
    loc = String()
    rate = rospy.Rate(10) 
    while not rospy.is_shutdown(): 

        # Set this time parameter to decide the frequency of the command PLAY 
        time.sleep(120)   
        human_command = "play"
        rospy.loginfo(human_command)
        time.sleep(60)
        pub.publish(human_command)
        rate.sleep()

        # Subscribe to the/human_command topic
        time.sleep(2)
        robot_say = rospy.Subscriber("/human_command", String, callback_robot,  queue_size=1)
        if (robot_say == "arrived"):
            loc = random.choice(['kitchen', 'bathroom', 'closet', 'bedroom', 'living_room', 'entrance'])
            pub.publish(loc)
            time.sleep(30)
            

if __name__ == '__main__':
    try:
        Human()
    except rospy.ROSInterruptException:
        pass


