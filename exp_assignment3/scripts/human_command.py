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


def callback_robot(msg):
    # TO GO TO COMMAND RANDOM CHOICE
    if (msg.data == "arrived"):

        location_choice = random.choice(
        ['kitchen', 'bathroom', 'closet', 'bedroom', 'living_room', 'entrance'])
    # PUBLISH LOCATION TO THE ROBOT
        pub_command(location_choice)


def Human_command():

    while True:

        # User random choice
        time.sleep(400)  # ASPETTA 1000 SEC???  ADESSO L'HO ABBASSATO A 100
        user_choice = "play"
        print("play")

        # A STO PUNTO FACCIO QUESTO PUBLISH
        pub_command = rospy.Publisher('/user_command', String, queue_size=10)
        # publish on the topic user's choice
        pub_command(user_choice)
        #rospy.set_param('/user_command', 'no play ')
        time.sleep(50)  # aspetta 30 sec

        sub_robot = rospy.Subscriber(
            "/arrived_human", String, callback_robot,  queue_size=1)

        # if (say_location == True): HO CANCELLATO ANCHE QUESTO
        #location_choice = random.choice(['kitchen','bathroom','closet','bedroom','living_room','entrance'])

        # NELLA STATE MACHINE, OGNI VOLTA CHE SALVO LA POSIZIONE DELLA PALLA, METTO UNA BOOLEANA A TRUE
        # -> MI DICE CHE LA POSIZIONE è STATA SALVATA (QUESTO DA FARE NALLA STATE MACHINE) E UN'ALTRO ROSPARAM
        # CHE MI SALVA LA POSIZIONE DELLA PALLA (NUMERO)

        # GROUNDING HO CANCELLATO TUTTO IL GROUNDING
        # if (location_choice == 'kitchen'): # SE L'USER HA DETTO KITCHEN
        # PRENDI IL VALORE DAL ROSPARAM
        # robot_position = rospy.get_param('/position_kitchen')   # ricorda di settare tutti i rosparam a 0 nel launch file
        # if (robot_position != 0) # CHEK SE LA LOCATION è CONOSCIUTA
        # pub_location(robot_position)  # PUBBLICALO SU UN TOPIC /COORDINATE_LOCATION
        # COSì LO STATO PLAY CHIAMA LA FUNZIONE CHE GLI FA RAGGIUNGERE IL TARGET

        # else:


def main():
    # Inizialize the node
    rospy.init_node('human_command', anonymous=True)

    while True:

        Human_command()
        time.sleep(20)  # wait some time


if __name__ == '__main__':
    main()

 # IN PRATICA QUESTO NODO è UN PUBLISHER
 # IF USER SAY "PLAY"
 # FACCIO IL PUBBLISH DI QUESTA STRINGA E LO LEGGO IN OGNI STATO (DELLA STATE MACHINE -> COSì VADO A PLAY)
 # OVUNQUE MI TROVO ENTRO IN PLAY E MANDO IL MOVENORMAL COL TARGET (DA INSERIRE) DELLA POS INIZIALE DEL ROBOT
 # WAIT FOR A GO TO + LOCATION COMMAND (PUBBLIO UN'ALTRA VOLTA SULLO STESSO TOPIC).
 # NELLO STATO PLAY: IF THE LOCATION IS KNOWN -> GO TO THE LOCATION -> MANDO IL MOVENORMAL COL TARGET DI
 # QUESTA LOCATION ALTRIMENTI PASSO NELLO STATO FIND.


# This node simulates human commands (like: GET TO + LOCATION)
# qui faccio il ground ->  bedroom -> yelllow ball
# QUI FACCIO:
# if "see_ball_yellow" == TRUE (cioè, se il robot conosce già la location -> estrai le coordinate di yellow e pubblicale)
# in modo tale che il PLAY possa fare da subscriber (ricevendo la location di destinazione) e possa andare al target
# chiamando la funzione MOVE TO TARGET.
