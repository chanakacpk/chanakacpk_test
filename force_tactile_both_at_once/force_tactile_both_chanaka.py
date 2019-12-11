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


import pkgutil
import rospy
import numpy as np
import time
import tf
import math
import time
import PyKDL
import random
from PyKDL import Frame, Vector, Rotation

from sensor_msgs.msg import JointState 
from std_msgs.msg import String
from geometry_msgs.msg import Twist 
from std_msgs.msg import Header, Float64, Float32, Float64MultiArray 

import wires_robotic_platform.utils.transformations as transformations
from wires_robotic_platform.utils.logger import Logger
from wires_robotic_platform.utils.ros import RosNode 
from wires_robotic_platform.sensors.sensor_manager import SensorManager


#################### tactile sensor ###############
# node_tactile = RosNode("ml_gripper") # Node of the gripper that holding tactile sensor.
# node_tactile.setupParameter("hz", 250) # Frequency
# node_tactile.setHz(node_tactile.getParameter("hz"))
# node_tactile_rate = node_tactile.getParameter("hz")

# gripper_pub = node_tactile.createPublisher("/schunk_pg70/joint_setpoint", JointState)
# tactile_reset_pub = node_tactile.createPublisher("tactile_reset", String)



####################### force sensor ###############

node_force = RosNode("atift_manager_node") # Node name of force sensor

node_force.setupParameter("hz", 250) # Frequency
node_force.setHz(node_force.getParameter("hz"))
node_tactile_rate = node_force.getParameter("hz")

sensor_name = "atift"
sens = SensorManager(sensor_name)
node_force.createSubscriber("/atift", Twist, sens.sensor_callback)

