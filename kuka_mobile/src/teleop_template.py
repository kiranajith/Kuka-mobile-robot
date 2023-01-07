# !/usr/bin/env python
import rospy

from std_msgs.msg import Float64

import sys
import select
import termios
import tty
from kuka_mobile.srv import Attach, AttachRequest, AttachResponse


msg = """
Control Kuka_Mobile Base!
---------------------------
Moving around:
        w     
   a    s    d
        x     
w : move forward
a : turn left
d : turn right
s : stop
x : reverse
"""

moveBindings = {
    'i': (1, 0),
    'o': (1, -1),
    'j': (0, 1),
    'l': (0, -1),
    'u': (1, 1),
    ',': (-1, 0),
    '.': (-1, 1),
    'm': (-1, -1),
}
moveBindings1 = {
    'w': (10, 10),
    'a': (-10, 10),
    'd': (10, -10),
    'x': (-10, -10),
    's': (0, 0),
    'o': (0, 1),
    'c': (0, 1),
    'd': (0, 1),
    # ',': (-1, 0),
    # '.': (-1, 1),
    # 'm': (-1, -1),
}

speedBindings = {
    'q': (1.1, 1.1),
    'z': (.9, .9),
    '[': (1.1, 1),
    ']': (.9, 1),
    'e': (1, 1.1),
    'n': (1, .9),
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


speed = 8
turn = 0.5


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('alt_teleop1')

    # Add your topic here between ''. Eg '/my_robot/steering_controller/command'
    pub_right = rospy.Publisher('/wheel1/command', Float64, queue_size=10)
    pub_left = rospy.Publisher('/wheel2/command', Float64, queue_size=10)
    pub_g1 = rospy.Publisher('/gripp1/command', Float64, queue_size=10)
    pub_g2 = rospy.Publisher('/gripp2/command', Float64, queue_size=10)

    speed1 = 0
    speed2 = 0

    x = 0
    th = 0
    status = 0
    count = 0
    acc = 0.1
    target_speed = 0
    target_turn = 0
    control_speed = 0
    control_turn = 0
    try:
        print(msg)
        print(vels(speed, turn))
        while(1):
            key = getKey()
            if key in moveBindings1.keys():
                if (key == 'c'):
                    gripp1 = 0
                    gripp2 = 0
                    pub_g1.publish(gripp1)
                    pub_g2.publish(gripp2)

                elif (key == 'o'):
                    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                                    Attach)
                    attach_srv.wait_for_service()
                    rospy.loginfo(
                        "Created ServiceProxy to /link_attacher_node/attach")

                    # Link them
                    rospy.loginfo("droping the object")
                    req = AttachRequest()
                    req.model_name_1 = "unit_box_clone"
                    req.link_name_1 = "link"
                    req.model_name_2 = "unit_box_0"
                    req.link_name_2 = "link"

                    attach_srv.call(req)
                    attach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
                                                    Attach)
                    attach_srv.wait_for_service()
                    req = AttachRequest()
                    req.model_name_1 = "my_robot"
                    req.link_name_1 = "gripp1"
                    req.model_name_2 = "unit_box_0"
                    req.link_name_2 = "link"
                    attach_srv.call(req)

                    gripp1 = -0.025
                    gripp2 = 0.025
                    pub_g1.publish(gripp1)
                    pub_g2.publish(gripp2)

                else:

                    speed1 = 0
                    speed2 = 0
                    speed1 = moveBindings1[key][0]
                    speed2 = moveBindings1[key][1]
                    x = moveBindings1[key][0]
                    th = moveBindings1[key][1]
                    count = 0
                x = 0
                th = 0
                control_speed = 0
                control_turn = 0
            else:
                count = count + 1
                if count > 4:
                    x = 0
                    th = 0
                if (key == '\x03'):
                    break

            target_speed = speed * x
            target_turn = turn * th

            if target_speed > control_speed:
                control_speed = min(target_speed, control_speed + 0.1)
            elif target_speed < control_speed:
                control_speed = max(target_speed, control_speed - 0.1)
            else:
                control_speed = target_speed

            if target_turn > control_turn:
                control_turn = min(target_turn, control_turn + 0.1)
            elif target_turn < control_turn:
                control_turn = max(target_turn, control_turn - 0.1)
            else:
                control_turn = target_turn

            pub_right.publish(speed1)  # publish the turn command.
            pub_left.publish(speed2)  # publish the turn command.
            

    except:
        print('e')

    finally:
        pub_right.publish(speed1)
        pub_left.publish(speed2)
        

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
