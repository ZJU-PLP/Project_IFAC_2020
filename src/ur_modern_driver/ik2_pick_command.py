#!/usr/bin/python
#
# Send joint values to UR5 using messages
#

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory

from trajectory_msgs.msg import JointTrajectoryPoint
import rospy

from trac_ik_python.trac_ik import IK

from tf.transformations import quaternion_from_euler

ik_solver = IK("base_link", "wrist_2_link")

seed_state = [0.0] * ik_solver.number_of_joints

# Convert RPY to Quaternions
q = quaternion_from_euler(0, 0, 0)

joint_space = ik_solver.get_ik(seed_state,
                0.0, 0.50, 0.3,  # X, Y, Z
       	         q[0], q[1], q[2], q[3])  # QX, QY, QZ, QW

print(joint_space)
waypoints = [[joint_space[0], joint_space[1], joint_space[2], joint_space[3], joint_space[4], 0]]

def main():

    rospy.init_node('send_joints')
    pub = rospy.Publisher('/arm_controller/command',
                          JointTrajectory,
                          queue_size=10)

    # Create the topic message
    traj = JointTrajectory()
    traj.header = Header()
    # Joint names for UR5
    traj.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint',
                        'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                        'wrist_3_joint']

    rate = rospy.Rate(1)
    pts = JointTrajectoryPoint()
    traj.header.stamp = rospy.Time.now()

    while not rospy.is_shutdown():
        for i in range(0,1):
            pts.positions = waypoints[i]

            pts.time_from_start = rospy.Duration(1.0)

            # Set the points to the trajectory
            traj.points = []
            traj.points.append(pts)
            # Publish the message
            pub.publish(traj)
            rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
	print ("Program interrupted before completion")
