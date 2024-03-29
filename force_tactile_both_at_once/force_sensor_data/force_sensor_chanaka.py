#!/usr/bin/env python
# -*- encoding: utf-8 -*-

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


node = RosNode("atift_manager_node")

node.setupParameter("hz", 250)
node.setHz(node.getParameter("hz"))
node_rate = node.getParameter("hz")

sensor_name = "atift"
sens = SensorManager(sensor_name)
node.createSubscriber("/atift", Twist, sens.sensor_callback)

try:
    while node.isActive():
        node.tick()
except rospy.ROSInterruptException:
    pass










