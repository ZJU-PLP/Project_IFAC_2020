#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension

def talker():
    pub = rospy.Publisher('path', Float32MultiArray, queue_size=10)
    rospy.init_node('path_planner', anonymous=True)
    rate = rospy.Rate(1) # 1hz

    waypoints = Float32MultiArray()
    waypoints.layout.dim.append(MultiArrayDimension())
    waypoints.layout.dim.append(MultiArrayDimension())

    waypoints.layout.dim[0].label = "Length"
    #waypoints.layout.dim[0].size = 18
    length = waypoints.layout.dim[0].size
    waypoints.layout.dim[0].stride = length*3
    waypoints.layout.dim[1].label = "Joints"
    waypoints.layout.dim[1].size = 3
    waypoints.layout.dim[1].stride = 3
    waypoints.layout.data_offset = 0

    while not rospy.is_shutdown():
        waypoints.data = [0]*(length)
        #for i in range(0, length):
        #    waypoints.data[i] = i + 1
        #waypoints.data = [-1.44, 1.4, 0.6, -1.57, 0, -1.57, -1.0, 0.7, 0, -0.5, 0.7, -1.0, -0.3, 0.5, -0.5, 0, 0, 0, -0.3, 0.5, -0.5, 0, 0, 0]
	waypoints.data = [-1.57, 0, 0]
        waypoints.layout.dim[0].size = len(waypoints.data)
        rospy.loginfo(waypoints)
        pub.publish(waypoints)
        rate.sleep()
        print("\nTask completed\n")

        rospy.signal_shutdown("Task completed")

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass

