from numpy import array#, sym.cos, sym.sin
import sympy as sym
# import tf
# from tf import TransformListener
# import rospy

"""
Show Homogeneous Transformation Matrices of each frame for comparison
"""
# def show_HTM(ur5_param, joint_values):

# matrix = tf.TransformListener(True, rospy.Duration(10.0))
# rospy.sleep(0.5)

# th1, th2, th3, th4, th5, th6 = [0.5, -1.30, 0, -1.5707, 1.5707, 0]
# d1, SO, EO, a2, a3, d4, d45, d5, d6 = (0.089159, 0.13585, -0.1197, 0.425, 0.39225, 0.10915, 0.093, 0.09465, 0.0823 + 0.15)
th1 = sym.Symbol('th1')
th2 = sym.Symbol('th2')
th3 = sym.Symbol('th3')
th4 = sym.Symbol('th4')
th5 = sym.Symbol('th5')
th6 = sym.Symbol('th6')

d1 = sym.Symbol('d1')
SO = sym.Symbol('SO')
EO = sym.Symbol('EO')
a2 = sym.Symbol('a2')
a3 = sym.Symbol('a3')
d4 = sym.Symbol('d4')
d45 = sym.Symbol('d45')
d5 = sym.Symbol('d5')
d6 = sym.Symbol('d6')



print "T0_1:  -------------------------------------- SHOULDER_LINK "
t0_1 = sym.Matrix([[sym.cos(th1), -sym.sin(th1), 0, 0],
              [sym.sin(th1), sym.cos(th1), 0, 0],
              [0, 0, 1, d1],
              [0, 0, 0, 1]])
print(t0_1)

print "T0_1:  -------------------------------------- SHOULDER_LINK_ROS"
# pos1, quat1 = matrix.lookupTransform("base_link", "shoulder_link", rospy.Time())
# t0_1_ros = matrix.fromTranslationRotation(pos1, quat1)
# print(t0_1_ros)

print '\n'

print "T0_2:  -------------------------------------- UPPER_ARM_LINK "
t1_2 = sym.Matrix([[-sym.sin(th2), 0, sym.cos(th2), 0],
              [0, 1, 0, SO],
              [-sym.cos(th2), 0, -sym.sin(th2), 0],
              [0, 0, 0, 1]])
t0_2 = t0_1*t1_2
print(t0_2)

# Maneira correta de obter somente a orientacao
print(t0_2[:3,:3])

# print "T0_2:  -------------------------------------- UPPER_ARM_LINK_ROS"
# pos2, quat2 = matrix.lookupTransform("base_link", "upper_arm_link", rospy.Time())
# t0_2_ros = matrix.fromTranslationRotation(pos2, quat2)
# print(t0_2_ros)

print '\n'

print "T0_3:  -------------------------------------- FOREARM_LINK "
t2_3 = sym.Matrix([[sym.cos(th3), 0, sym.sin(th3), 0],
              [0, 1, 0, EO],
              [-sym.sin(th3), 0, sym.cos(th3), a2],
              [0, 0, 0, 1]])
t0_3 = t0_2*(t2_3)
print(t0_3)

# print "T0_3:  -------------------------------------- FOREARM_LINK_ROS"
# pos3, quat3 = matrix.lookupTransform("base_link", "forearm_link", rospy.Time())
# t0_3_ros = matrix.fromTranslationRotation(pos3, quat3)
# print(t0_3_ros)

print '\n'

print "T0_4:  -------------------------------------- WRIST_1_LINK "
t3_4 = sym.Matrix([[-sym.sin(th4), 0, sym.cos(th4), 0],
              [0, 1, 0, 0],
              [-sym.cos(th4), 0, -sym.sin(th4), a3],
              [0, 0, 0, 1]])
t0_4 = t0_3*(t3_4)
print(t0_4)

# print "T0_4:  -------------------------------------- WRIST_1_LINK_ROS"
# # pos4, quat4 = matrix.lookupTransform("base_link", "wrist_1_link", rospy.Time())
# # t0_4_ros = matrix.fromTranslationRotation(pos4, quat4)
# # print(t0_4_ros)

print '\n'

print "T0_5:  -------------------------------------- WRIST_2_LINK "
t4_5 = sym.Matrix([[sym.cos(th5), -sym.sin(th5), 0, 0],
              [sym.sin(th5), sym.cos(th5), 0, d45],
              [0, 0, 1, 0],
              [0, 0, 0, 1]])
t0_5 = t0_4*(t4_5)
print(t0_5)

# print "T0_5:  -------------------------------------- WRIST_2_LINK_ROS"
# # pos5, quat5 = matrix.lookupTransform("base_link", "wrist_2_link", rospy.Time())
# # t0_5_ros = matrix.fromTranslationRotation(pos5, quat5)
# # print(t0_5_ros)

print '\n'

print "T0_6:  -------------------------------------- WRIST_3_LINK "
t5_6 = sym.Matrix([[sym.cos(th6), 0, sym.sin(th6), 0],
              [0, 1, 0, 0],
              [-sym.sin(th6), 0, sym.cos(th6), d5],
              [0, 0, 0, 1]])
t0_6 = t0_5*(t5_6)
print(t0_6)

# print "T0_6:  -------------------------------------- WRIST_3_LINK_ROS"
# # pos6, quat6 = matrix.lookupTransform("base_link", "wrist_3_link", rospy.Time())
# # t0_6_ros = matrix.fromTranslationRotation(pos6, quat6)
# # print(t0_6_ros)

print '\n'

print "T0_7:  -------------------------------------- TOOL0 "
t6_7 = sym.Matrix([[1, 0, 0, 0],
              [0, 0, 1, d6],
              [0, -1, 0, 0],
              [0, 0, 0, 1]])
t0_7 = t0_6*(t6_7)
print(t0_7)
print '\n'
print(sym.simplify(t0_7))
print '\n'
print(sym.simplify(sym.simplify(t0_7)))

# print "T0_7:  -------------------------------------- TOOL0_LINK_ROS"
# # pos7, quat7 = matrix.lookupTransform("base_link", "tool0", rospy.Time())
# # t0_7_ros = matrix.fromTranslationRotation(pos7, quat7)
# # print(t0_7_ros)
