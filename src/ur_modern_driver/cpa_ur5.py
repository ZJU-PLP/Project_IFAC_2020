#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import geometry_msgs.msg

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from trac_ik_python.trac_ik import IK
from tf.transformations import quaternion_from_euler

ik_solver = IK("base_link", "wrist_3_link")
seed_state = [0.0] * ik_solver.number_of_joints

# Initial, final and obstacle points
ptFinal = [0.45, 0.60, 0.30]
ptObst = [0.45, 0.20, 0.35]

# Attractive and repulsive fields constants
rho = 100
eta = 0.1

# Size of each iteration step
#alpha = 0.0022
alpha = 0.005

# Reach of repulsive field
di = 0.20

def distancia_pontos(ponto1, ponto2):
	'''calculate the euclidean distance, no numpy
	input: numpy.arrays or lists
	return: euclidean distance
	'''
	dist = [(a - b)**2 for a, b in zip(ponto1, ponto2)]
	dist = math.sqrt(sum(dist))
	return dist

def planejar_caminho(ptAtual):
	ptX = ptAtual[0]
	ptY = ptAtual[1]
	ptZ = ptAtual[2]

# Attractive force
	F = [0, 0, 0]
	F[0] = ptX - ptFinal[0]
	F[1] = ptY - ptFinal[1]
	F[2] = ptZ - ptFinal[2]

	distF = distancia_pontos(ptAtual, ptFinal)

	F[0] = F[0]/distF
	F[1] = F[1]/distF
	F[2] = F[2]/distF

	modF = rho*distF

# Repulsive force
	R = [0, 0, 0]
	R[0] = ptX - ptObst[0]
	R[1] = ptY - ptObst[1]
	R[2] = ptZ - ptObst[2]

	distO = distancia_pontos(ptAtual, ptObst)

	if distO > di:
		modR = 0
	else:
		R[0] = R[0]/distO
		R[1] = R[1]/distO
		R[2] = R[2]/distO

		modR = eta*(1/distO - 1/di)/math.pow(distO,2)
	print("modF: ", modF)
	print("modR: ", modR)
	#modR = 0

	newPt = [0, 0, 0]
	newPt[0] = ptX - alpha*modF*F[0] + alpha*modR*R[0]
	newPt[1] = ptY - alpha*modF*F[1] + alpha*modR*R[1]
	newPt[2] = ptZ - alpha*modF*F[2] + alpha*modR*R[2]

	return newPt

def tf_listener():
	rospy.init_node('ur5_tf_listener', 'send_joints')

	listener = tf.TransformListener()

	rate = rospy.Rate(10.0)

	return listener, rate


if __name__ == '__main__':
	lstnr, rate = tf_listener()

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

	pts = JointTrajectoryPoint()
	traj.header.stamp = rospy.Time.now()
    
	while not rospy.is_shutdown():
		try:
			(eeTrans,eeRot) = lstnr.lookupTransform('base_link', 'wrist_3_link', rospy.Time(0))
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

		if distancia_pontos(eeTrans, ptFinal) > 0.01:
			print("Current point:")
			print("x: %.4f y: %.4f z: %.4f"% (eeTrans[0], eeTrans[1], eeTrans[2]))

			ptProx = planejar_caminho(eeTrans)
			print("Next point:")
			print("x: %.4f y: %.4f z: %.4f"% (ptProx[0], ptProx[1], ptProx[2]))

			# Convert RPY to Quaternions
			q = quaternion_from_euler(0, 0, 0)

			joint_space = ik_solver.get_ik(seed_state,
		            				ptProx[0], ptProx[1], ptProx[2],  # X, Y, Z
		   	        				q[0], q[1], q[2],  q[3])  # QX, QY, QZ, QW

			#print(joint_space)

			pts.positions = joint_space
			pts.time_from_start = rospy.Duration(1.0)
			# Set the points to the trajectory
			traj.points = []
			traj.points.append(pts)
			# Publish the message
			pub.publish(traj)

			rate.sleep()
		else:
			print("Objective reached")
