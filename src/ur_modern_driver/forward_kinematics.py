#!/usr/bin/python
#
# Send joint values to UR5 using messages
#

import math
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory

from trajectory_msgs.msg import JointTrajectoryPoint
import rospy

from trac_ik_python.trac_ik import IK

from tf.transformations import quaternion_from_euler

import numpy as np
import sys
import roslib
#roslib.load_manifest("ur_kinematics")
#from ur_kin_py import forward
from sensor_msgs.msg import JointState

#UR5_PARAMS
d1 =  0.089159;
a2 = -0.42500;
a3 = -0.39225;
d4 =  0.10915;
d5 =  0.09465;
d6 =  0.0823;

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I heard %s", data.position)
    T = forwardk(data.position)
    x = 0
    y = 0
    z = 0.1
    xee = x*T[0] + y*T[1] + z*T[2] + T[3]
    yee = x*T[4] + y*T[5] + z*T[6] + T[7]
    zee = x*T[8] + y*T[9] + z*T[10] + T[11]
    print("T: ", T[0], T[1], T[2], T[3])
    print(T[4], T[5], T[6], T[7])
    print(T[8], T[9], T[10], T[11])
    print(T[12], T[13], T[14], T[15])
    print("x: ", xee)
    print("y: ", yee)
    print("z: ", zee)
    
def joint_listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('joint_listener', anonymous=True)

    rospy.Subscriber("joint_states", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def forwardk(q):
    T = []

    s1 = math.sin(q[0])
    c1 = math.cos(q[0])

    q234 = q[1]
    s2 = math.sin(q[1])
    c2 = math.cos(q[1])

    s3 = math.sin(q[2])
    c3 = math.cos(q[2])
    q234 += q[2]

    q234 += q[3]

    s5 = math.sin(q[4])
    c5 = math.cos(q[4])

    s6 = math.sin(q[5])
    c6 = math.cos(q[5])

    s234 = math.sin(q234)
    c234 = math.cos(q234)

    #T.append(((c1*c234-s1*s234)*s5)/2.0 - c5*s1 + ((c1*c234+s1*s234)*s5)/2.0)
    #T.append((c6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0) - (s6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0))
    #T.append((-(c6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0 - s6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0)))
    #T.append(((d5*(s1*c234-c1*s234))/2.0 - (d5*(s1*c234+c1*s234))/2.0 - d4*s1 + (d6*(c1*c234-s1*s234)*s5)/2.0 + (d6*(c1*c234+s1*s234)*s5)/2.0 - a2*c1*c2 - d6*c5*s1 - a3*c1*c2*c3 + a3*c1*s2*s3))
    #T.append(c1*c5 + ((s1*c234+c1*s234)*s5)/2.0 + ((s1*c234-c1*s234)*s5)/2.0)
    #T.append((c6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0) + s6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0)))
    #T.append((c6*((c1*c234-s1*s234)/2.0 - (c1*c234+s1*s234)/2.0) - s6*(((s1*c234+c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234-c1*s234)*c5)/2.0)))
    #T.append(((d5*(c1*c234-s1*s234))/2.0 - (d5*(c1*c234+s1*s234))/2.0 + d4*c1 + (d6*(s1*c234+c1*s234)*s5)/2.0 + (d6*(s1*c234-c1*s234)*s5)/2.0 + d6*c1*c5 - a2*c2*s1 - a3*c2*c3*s1 + a3*s1*s2*s3))
    #T.append(((c234*c5-s234*s5)/2.0 - (c234*c5+s234*s5)/2.0))
    #T.append(((s234*c6-c234*s6)/2.0 - (s234*c6+c234*s6)/2.0 - s234*c5*c6))
    #T.append((s234*c5*s6 - (c234*c6+s234*s6)/2.0 - (c234*c6-s234*s6)/2.0))
    #T.append((d1 + (d6*(c234*c5-s234*s5))/2.0 + a3*(s2*c3+c2*s3) + a2*s2 - (d6*(c234*c5+s234*s5))/2.0 - d5*c234))
    #T.append(0.0)
    #T.append(0.0)
    #T.append(0.0)
    #T.append(1.0)
    
#nx
    T.append(c6*(s1*s5 + ((c1*c234-s1*s234)*c5)/2.0 + ((c1*c234+s1*s234)*c5)/2.0) - (s6*((s1*c234+c1*s234) - (s1*c234-c1*s234)))/2.0)
#ox
    T.append(-(c6*((s1*c234 + c1*s234) - (s1*c234 - c1*s234)))/2.0 - s6*(s1*s5 + ((c1*c234 - s1*s234)*c5)/2.0 + ((c1*c234 + s1*s234)*c5)/2.0))
#ax
    T.append(c5*s1 - ((c1*c234 - s1*s234)*s5)/2.0 - ((c1*c234 + s1*s234)*s5)/2.0)
#px
    T.append(-((d5*(s1*c234 - c1*s234))/2.0 + (d5*(s1*c234 + c1*s234))/2.0 + d4*s1 - (d6*(c1*c234 - s1*s234)*s5)/2.0 - (d6*(c1*c234 +s1*s234)*s5)/2.0 + a2*c1*c2 + d6*c5*s1 + a3*c1*c2*c3 - a3*c1*s2*s3))
#ny
    T.append(c6*(((s1*c234 + c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234 - c1*s234)*c5)/2.0) + s6*((c1*c234 - s1*s234)/2.0 - (c1*c234 + s1*s234)/2.0))
#oy
    T.append(c6*((c1*c234 - s1*s234)/2.0 - (c1*c234 + s1*s234)/2.0) - s6*(((s1*c234 + c1*s234)*c5)/2.0 - c1*s5 + ((s1*c234 - c1*s234)*c5)/2.0))
#ay
    T.append(-c1*c5 - ((s1*c234 + c1*s234)*s5)/2.0 + ((c1*s234 - s1*c234)*s5 )/2.0)
#py
    T.append(-((d5*(c1*c234 - s1*s234))/2.0 + (d5*(c1*c234 + s1*s234))/2.0 - d4*c1 - (d6*(s1*c234 +c1*s234)*s5)/2.0 - (d6*(s1*c234 - c1*s234)*s5)/2.0 - d6*c1*c5 + a2*c2*s1 + a3*c2*c3*s1 - a3*s1*s2*s3))
#nz
    T.append((s234*c6 + c234*s6)/2.0 + s234*c5*c6 - (s234*c6 - c234*s6)/2.0)
#oz
    T.append((c234*c6 + s234*s6)/2.0 + (c234*c6 - s234*s6)/2.0 - s234*c5*s6)
#az
    T.append((c234*c5 - s234*s5)/2.0 - (c234*c5 + s234*s5)/2.0)
#pz
    T.append(d1 + (d6*(c234*c5 - s234*s5))/2.0 + a3*(s2*c3 + c2*s3) + a2*s2 - (d6*(c234*c5 + s234*s5))/2.0 - d5*c234)
    
    T.append(0.0)
    T.append(0.0)
    T.append(0.0)
    T.append(1.0)

    return T

if __name__ == '__main__':
    joint_listener()
