#!/usr/bin/env python
# -*- encoding: utf-8 -*-

import pkgutil 
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Point32
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, Float64, Float32, Float64MultiArray
import math
import time
from PyKDL import Frame, Vector, Rotation
import PyKDL
import tf
from wires_robotic_platform.srv import PolyDetectionService
import wires_robotic_platform.utils.transformations as transformations
from wires_robotic_platform.utils.logger import Logger
from wires_robotic_platform.vision.markers import MarkerDetector
from wires_robotic_platform.vision.cameras import CameraRGB
from wires_robotic_platform.vision.terminals import TerminalTray, Terminal, TerminalDetector
import wires_robotic_platform.utils.visualization as visualization
from wires_robotic_platform.utils.sci import ClusterBuilder2D
from scipy.ndimage.measurements import center_of_mass
import wires_robotic_platform.vision.cv as cv
from wires_robotic_platform.storage.tf_storage import TfStorage #
from wires_robotic_platform.utils.ros import RosNode
from wires_robotic_platform.proxy.proxy_message import SimpleMessage, SimpleMessageProxy

import message_filters
from sensor_msgs.msg import Image, CameraInfo
import cv2
# import aruco # Need to be install opencv
import rospkg
import numpy as np
import math
import sys
import random



import time



# Remember to catkin_make and source the working directory.
# Then run roscore 
# Once done above go for run the code
# rosrun package_name collect_data_grasp.py 

node = RosNode("ml_gripper") # Node of the gripper that holding tactile sensor.

node.setupParameter("hz", 250) # Frequency
node.setHz(node.getParameter("hz"))
node_rate = node.getParameter("hz")

gripper_pub = node.createPublisher("/schunk_pg70/joint_setpoint", JointState)

tactile_reset_pub = node.createPublisher("tactile_reset", String)

def gripper(pos, t = 1):
    gr_msg = JointState()
    gr_msg.position = [pos]
    gr_msg.velocity = [70] # Gripper moving velocity.
    gr_msg.effort = [50]
    gripper_pub.publish(gr_msg)
    time.sleep(t) # t is sleep time of the gripper

testNum = 50 # Number of test we are going to carry
# We will carry out 50 different wire positions and collect data 
results = []


gripper(20, t = 3) # Gripper starting point # play with these for different outcomes.
tactile_reset_pub.publish("")
time.sleep(2)
while node.isActive(): 
    raw_input("Press Enter to continue.....") # Once we pressed the enter gripper will
    # grab the wire
    gripper(2, t = 4) # Grapping point of the wire 
    

    #"Press Enter to continue"
    gripper(10, t = 2)

    tactile_reset_pub.publish("")
    print(testNum)
    testNum -= 1
    if testNum == 0:
        raw_input("End.....")
    
    time.sleep(2)
    node.tick


