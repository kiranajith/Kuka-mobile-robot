import time
import signal
import sys
import getch
import rospy
from std_msgs.msg import Float64, Float64MultiArray
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

from kuka_mobile.srv import Attach, AttachRequest, AttachResponse

from std_msgs.msg import Float64

import sys
import select
import termios
import tty

msg = """
Control Kuka Manipulator!
---------------------------
Moving around:
        w          i
   a    s    d      
        x          k
w : move +z
x : move -z
a : move -x
d : move +x
i : move +y
k : move -y
s : stop
"""

rospy.init_node('transform')
rospy.loginfo(msg)

# Add your topic here between ''. Eg '/my_robot/steering_controller/command'
pub_joint1 = rospy.Publisher('/joint1/command', Float64, queue_size=10)
pub_joint2 = rospy.Publisher('/joint2/command', Float64, queue_size=10)
pub_joint3 = rospy.Publisher('/joint3/command', Float64, queue_size=10)
pub_joint4 = rospy.Publisher('/joint4/command', Float64, queue_size=10)
pub_joint5 = rospy.Publisher('/joint5/command', Float64, queue_size=10)
pub_joint6 = rospy.Publisher('/joint6/command', Float64, queue_size=10)
pub_joint7 = rospy.Publisher('/joint7/command', Float64, queue_size=10)
pub_g1 = rospy.Publisher('/gripp1/command', Float64, queue_size=10)
pub_g2 = rospy.Publisher('/gripp2/command', Float64, queue_size=10)


# transformation matrix
def transformationMatrix(theta_i, alpha_i, d_i, a_i):
    R_theta = Matrix([[cos(theta_i), -sin(theta_i), 0, a_i],
                      [sin(theta_i), cos(theta_i), 0, 0],
                      [0, 0, 1, d_i],
                      [0, 0, 0, 1]])
    R_alpha = Matrix([[1, 0, 0, 0],
                      [0, cos(alpha_i), -sin(alpha_i), 0],
                      [0, sin(alpha_i), cos(alpha_i), 0],
                      [0, 0, 0, 1]])
    T = R_theta * R_alpha

    return T


def jacobian(theta1, theta2, theta4, theta5, theta6, thata7):  # Defination of jacobian function
    T12 = transformationMatrix(theta1, -pi / 2, 360, 0)
    T23 = transformationMatrix(theta2, pi / 2, 0, 0)
    T34 = transformationMatrix(0, pi / 2, 4200, 0)
    T45 = transformationMatrix(theta4, -pi / 2, 0, 0)
    T56 = transformationMatrix(theta5, -pi / 2, 399, 0)
    T67 = transformationMatrix(theta6, pi / 2, 0, 0)
    T7eff = transformationMatrix(thata7, 0, 205, 0)
    T1eff = T12 * T23 * T34 * T45 * T56 * T67 * T7eff
    T13 = T12 * T23
    T14 = T13 * T34
    T15 = T14 * T45
    T16 = T15 * T56
    T17 = T12 * T23 * T34 * T45 * T56 * T67

    z0 = Matrix([[0], [0], [1]])  # z Axis
    z1 = T12[0:3, 2]
    z2 = T13[0:3, 2]
    z3 = T14[0:3, 2]
    z4 = T15[0:3, 2]
    z5 = T16[0:3, 2]
    z6 = T17[0:3, 2]
    z7 = T1eff[0:3, 2]

    o0 = Matrix([[0],  # Origins
                 [0],
                 [0]])

    o1 = T12[0:3, 3]

    o2 = T13[0:3, 3]

    o3 = T14[0:3, 3]
    o4 = T15[0:3, 3]

    o5 = T16[0:3, 3]

    o6 = T17[0:3, 3]

    o7 = T1eff[0:3, 3]

    jv1 = z0.cross(o7 - o0)

    j1 = Matrix([[(jv1)],
                 [(z0)]])

    jv2 = z1.cross(o7 - o1)
    j2 = Matrix([[(jv2)],
                 [(z1)]])

    jv3 = z2.cross(o7 - o2)
    j3 = Matrix([[(jv3)],
                 [(z2)]])

    jv4 = z3.cross(o7 - o3)
    j4 = Matrix([[(jv4)],
                 [(z3)]])

    jv5 = z4.cross(o7 - o4)
    j5 = Matrix([[(jv5)],
                 [(z4)]])

    jv6 = z5.cross(o7 - o5)
    j6 = Matrix([[(jv6)],
                 [(z5)]])

    jv7 = z6.cross(o7 - o6)
    j7 = Matrix([[(jv7)],
                 [(z6)]])

    j = Matrix([[j1, j2, j4, j5, j6, j7]])
    return j


def signal_handler(sig, frame):
    print("")
    print("Terminated")
    sys.exit(0)


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
   
    N = 2000  # Number of Iteration

    x = 80
    y = 80
    z = 80

    q = np.zeros((6, N))  # Initial State

    q[0, 0] = 2 * pi / 3
    q[1, 0] = 0.01
    q[2, 0] = pi / 2
    q[3, 0] = pi / 6 - 0.3

    q[4, 0] = -pi / 2
    q[5, 0] = -pi / 6

    pub_joint1.publish(q[0, 0])
    pub_joint2.publish(q[1, 0])
    pub_joint4.publish(q[2, 0])
    pub_joint5.publish(q[3, 0])
    pub_joint6.publish(q[4, 0])
    pub_joint7.publish(q[5, 0])
    pub_g1.publish(-0.025)
    pub_g2.publish(0.025)

    for i in range(0, N - 1):
        
        val = getch.getch()
        
        if (val == 'W' or val == 'w'):
            z += 80
            x = 0.0
            y = 0.0
            rospy.loginfo("w: moving up(+z)")

        # Backward
        elif (val == 'X' or val == 'x'):
            z -= 80
            x = 0.0
            y = 0.0
            rospy.loginfo("x: moving down(-z)")

        elif (val == 'I' or val == 'i'):
            y += 80
            z = 0
            x = 0
            rospy.loginfo("i: moving +y")
        
        elif (val == 'K' or val == 'k'):
            y -= 80
            z = 0
            x = 0
            rospy.loginfo("k: moving -y")

        elif (val == 'A' or val == 'a'):
            x -= 80
            y = 0
            z = 0
            rospy.loginfo("k: moving -x")

        elif (val == 'D' or val == 'd'):
            x += 80
            y = 0
            z = 0
            rospy.loginfo("k: moving +x")
        
        # Stop
        elif (val == 'S' or val == 's'):
            x = 0.0
            y = 0.0
            z = 0.0
            rospy.loginfo("s: stop")
        
        y = y  # Velocity
        z = z
        x = x

        v = Matrix([x, y, z, 0, 0, 0])

        q_dot = Inverse(jacobian(q[0, i], q[1, i],
                                 q[2, i], q[3, i], q[4, i], q[5, i])) * v

        q[0, i + 1] = q[0, i] + q_dot[0, 0] * (5 / N)
        q[1, i + 1] = q[1, i] + q_dot[1, 0] * (5 / N)
        q[2, i + 1] = q[2, i] + q_dot[2, 0] * (5 / N)
        q[3, i + 1] = q[3, i] + q_dot[3, 0] * (5 / N)
        q[4, i + 1] = q[4, i] + q_dot[4, 0] * (5 / N)
        q[5, i + 1] = q[5, i] + q_dot[5, 0] * (5 / N)
        pub_joint1.publish(q[0, i])
        pub_joint2.publish(q[1, i])
        pub_joint4.publish(q[2, i])
        pub_joint5.publish(q[3, i])
        pub_joint6.publish(q[4, i])
        pub_joint7.publish(q[5, i])

    rospy.loginfo("Reached PickUp Position")
