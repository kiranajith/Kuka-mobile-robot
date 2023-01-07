from operator import inv
from re import T
from sympy import *
from sympy.abc import Y, Z
from sympy.core import symbol
from sympy.plotting.textplot import linspace
from sympy.tensor.array import Array
import numpy as np
import matplotlib.pyplot as plt
import rospy

from std_msgs.msg import Float64

import sys
import select
import termios
import tty

rospy.init_node('reset')

# Add your topic here between ''. Eg '/my_robot/steering_controller/command'
pub_joint1 = rospy.Publisher('/joint1/command', Float64, queue_size=10)
pub_joint2 = rospy.Publisher('/joint2/command', Float64, queue_size=10)
pub_joint3 = rospy.Publisher('/joint3/command', Float64, queue_size=10)
pub_joint4 = rospy.Publisher('/joint4/command', Float64, queue_size=10)
pub_joint5 = rospy.Publisher('/joint5/command', Float64, queue_size=10)
pub_joint6 = rospy.Publisher('/joint6/command', Float64, queue_size=10)
pub_joint7 = rospy.Publisher('/joint7/command', Float64, queue_size=10)


pub_joint1.publish(0)
pub_joint2.publish(0)
pub_joint4.publish(0)
pub_joint5.publish(0)
pub_joint6.publish(0)
pub_joint7.publish(0)

