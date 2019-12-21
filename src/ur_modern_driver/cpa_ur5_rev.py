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
from sensor_msgs.msg import JointState

ik_solver = IK("base_link", "wrist_3_link")
seed_state = [0.0] * ik_solver.number_of_joints

# Pontos final e obstaculo
ptFinal = [0.45, 0.60, 0.30]
ptObst = [0.45, 0.20, 0.35]

# Constantes dos campos atrativo e repulsivo
rho = 100
eta = 0.1

# Tamanho do passo de cada iteracao
#alpha = 0.0022
alpha = 0.005

# Distancia de influencia do campo repulsivo
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

# Forca atrativas
	F = [0, 0, 0]
	F[0] = ptX - ptFinal[0]
	F[1] = ptY - ptFinal[1]
	F[2] = ptZ - ptFinal[2]

	distF = distancia_pontos(ptAtual, ptFinal)

	F[0] = F[0]/distF
	F[1] = F[1]/distF
	F[2] = F[2]/distF

	modF = rho*distF

# Forca repulsiva
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

def joint_listener():
	


if __name__ == '__main__':
