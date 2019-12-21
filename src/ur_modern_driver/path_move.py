#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from std_msgs.msg import Header
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi
from std_msgs.msg import Float32MultiArray
import numpy as np

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
    
client = None

class Path_listener(object):
    def __init__(self):
        # Params
        self.layout = None
        self.path = None
        self.waypoints = None
        self.ready = False

        # Node cycle rate (in Hz).
        self.rate = rospy.Rate(1)

        # Subscribers
        rospy.Subscriber("path", Float32MultiArray, self.callback)


    def getCommandPath(self):
        return self.waypoints

    def callback(self, data):
        self.ready = True
        rospy.loginfo(rospy.get_caller_id() + "\nI heard:\n%s", data)
        self.path = data.data
        self.layout = data.layout
        self.makePath()

    def start(self):
        rospy.loginfo("Starting listener")

        while not rospy.is_shutdown():
                if self.ready:
                    break
                self.rate.sleep()

    def makePath(self):
	#pathLength = self.layout.dim[0].size/3
        #stride = self.layout.dim[1].stride
        #size = self.layout.dim[1].size
	pathLength = len(self.path)/4
        rospy.loginfo("Path Length: %d", pathLength)
        stride = 4
        size = 3

        self.waypoints = np.zeros((pathLength, size + 3))
	
        for i in range(0, pathLength):
	    #rospy.loginfo("i: %d", i)
            for j in range(0, size):
		#rospy.loginfo("j: %d", j)
                if (j == 0):
                    self.waypoints[i, j] = 0
                if (j == 2):
                    self.waypoints[i, j+2] = 0
                    self.waypoints[i, j+3] = self.path[j + stride*i + 1] # colocar +1
                self.waypoints[i, j+1] = self.path[j + stride*i]

        print "The command is:\n", self.waypoints

def move(Path_listener):
    pl = Path_listener
    global joints_pos
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        g.trajectory.points = []
        g.trajectory.points.append(JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)))
        g.trajectory.points.append(JointTrajectoryPoint(positions=[0,-1.57,0,0,0,0], velocities=[0]*6, time_from_start=rospy.Duration(2.0)))
        g.trajectory.points.append(JointTrajectoryPoint(positions=pl.waypoints[0], velocities=[0]*6, time_from_start=rospy.Duration(4.0)))
        for i in range(1, len(pl.waypoints)+1):
            g.trajectory.points.append(JointTrajectoryPoint(positions=pl.waypoints[i-1], velocities=[0]*6, time_from_start=rospy.Duration(0.03*i+4.0)))
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise
   
def main():
    global client
    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        path_listener = Path_listener()
        path_listener.start()
        client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
        # client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction) # REAL UR5 
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(JOINT_NAMES):
                JOINT_NAMES[i] = prefix + name
        #print "Please make sure that your robot can move before proceeding!"
        #inp = raw_input("Continue? y/n: ")[0]
        #if (inp == 'y'):
        #    move()
        #else:
        #   print "Halting program"
        move(path_listener)
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()
