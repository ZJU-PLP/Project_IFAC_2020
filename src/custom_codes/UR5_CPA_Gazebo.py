#!/usr/bin/env python

# ROS import
import sys
import rospy
import tf
from tf.transformations import quaternion_from_euler, euler_from_matrix, euler_from_quaternion #, quaternion_matrix
from tf import TransformListener, TransformerROS
import actionlib

# Moveit Import
import moveit_commander
from moveit_python import *
# from moveit_commander.conversions import pose_to_list

# Msg Import
from moveit_msgs.msg import *
# from geometry_msgs.msg import *
from geometry_msgs.msg import PoseStamped, Point, Vector3, Pose
from std_msgs.msg import String, Header, ColorRGBA, Float64
from visualization_msgs.msg import Marker
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from control_msgs.msg import *
from trajectory_msgs.msg import *

# Inverse Kinematics Import
from ur_inverse_kinematics import *

# Python Import
import numpy as np
from numpy import array, dot, pi
from numpy.linalg import det, norm
import csv
import argparse
import time
from itertools import izip_longest

# Customized code
from get_geometric_jacobian import *
from get_ur5_position import *
from get_dist3D import *
import CPA
from CPA import *
from smooth_path import *

def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value

"""
    PARSE ARGS
"""

def parse_args():
    parser = argparse.ArgumentParser(description='AAPF_Orientation')
    # store_false assumes that variable is already true and is only set to false if is given in command terminal
    parser.add_argument('--APF', action='store_false', help='Choose AAPF instead of APF')
    parser.add_argument('--COMP', action='store_true', help='Simulate APF and AAPF for comparison and generate CSV files for each of them')
    parser.add_argument('--OC_Off', action='store_false', help='Deactivate Orientation Control')
    parser.add_argument('--CSV', action='store_true', help='Write topics into a CSV file')
    parser.add_argument('--plot', action='store_true', help='Plot path to RVIZ through publish_trajectory.py (run this node first)')
    parser.add_argument('--realUR5', action='store_true', help='Enable real UR5 controlling')
    args = parser.parse_args()
    return args

def print_joint_angles(args, vec, file, type):
    with open('/home/caio/3_Projetos_UR5/Project_IFAC_2019/src/custom_codes/csv_files/' + file + '.csv', mode='w') as employee_file:
        employee_writer = csv.writer(employee_file, delimiter=' ', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        if type == 'joint':
            employee_writer.writerow(['Index', 'Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'])
            n = 0
            for position in vec:
                employee_writer.writerow([n, position[0], position[1], position[2], position[3], position[4], position[5]])
                n+=1
        if type == 'dist_to_goal' and args.COMP:
            employee_writer.writerow(['Index', 'dist_to_goal_AAPF', 'dist_to_goal_APF'])
            new_vec1 = [x[0] for x in vec[0]]
            new_vec2 = [x[0] for x in vec[1]]
            new_index = [x for x in range(max(len(vec[0]),len(vec[1])))]
            rows = izip_longest(new_index, new_vec1, new_vec2, fillvalue='nan')
            employee_writer.writerows(rows)
        elif type == 'dist_to_goal':
            employee_writer.writerow(['Index', 'dist_to_goal_AAPF'])
            n = 0
            for dist in vec:
                employee_writer.writerow([n, dist[0]])
                n+=1

def print_output(n, way_points, wayPointsSmoothed, dist_EOF_to_Goal):
    print("Dados dos CPAAs")
    print("Iterations: ", n)
    print("Way points: ", len(way_points))
    print("Way points smoothed: ", len(wayPointsSmoothed))
    print("Distance to goal: ", dist_EOF_to_Goal)

class MoveGroupPythonIntefaceTutorial(object):
    """MoveGroupPythonIntefaceTutorial"""

    def __init__(self, args, comp = False):
        super(MoveGroupPythonIntefaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('move_group_python_interface_ur5_robot', anonymous=True)

        # self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
        #                                                     moveit_msgs.msg.DisplayTrajectory,
        #                                                     queue_size=20)

        self.pose = PoseStamped()
        # publish path or trajectory to publish_trajectory.py
        self.pose_publisher = rospy.Publisher('pose_publisher_tp', PoseStamped, queue_size=10)

        # Topico para publicar marcadores para o Rviz
        self.marker_publisher = rospy.Publisher('visualization_marker2', Marker, queue_size=100)

        self.tf = tf.TransformListener()
        self.scene = PlanningSceneInterface("base_link")
        self.marker = Marker()
        self.joint_states = JointState()
        self.joint_states.name = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                                  'wrist_3_joint']

        # d1, SO, EO, a2, a3, d4, d45, d5, d6
        self.ur5_param = (0.089159, 0.13585, -0.1197, 0.425,
                          0.39225, 0.10915, 0.093, 0.09465, 0.0823 + 0.15)

        # Plot path inside while loop
        self.plot_path = True

        # Choose APF instead of AAPF if set True
        self.AAPF = True

        self.n = 1
        self.id = 100
        self.id2 = 130
        self.id_path = 200
        self.path_color = ColorRGBA(0.0, 0.0, 0.1, 0.8)
        self.diam_goal = [0.05]
        self.ptFinal = [[0.55, 0.0, 0.55]]

        if not comp:
            self.client = actionlib.SimpleActionClient('arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            print "Waiting for server (gazebo)..."
            self.client.wait_for_server()
            print "Connected to server (gazebo)"

            # Obstacle positions
            oc = [0.86, 0, 0.375 + 0.1]  # Obstacle reference point - 3D printer
            d1 = -0.080
            s = -1
            self.obs_pos = [oc, np.add(oc, [s * 0.14, 0.0925, 0.255 + d1]), np.add(oc, [s * 0.14, 0.185, 0.255 + d1]), np.add(oc, [s * 0.14, 0, 0.255 + d1]), np.add(oc, [s * 0.14, -0.0925, 0.255 + d1]), np.add(oc, [s * 0.14, -0.185, 0.255 + d1]),
                       np.add(oc, [s * 0.14, -0.185, 0.16 + d1]), np.add(oc, [s * 0.14, 0.185, 0.16 + d1]), np.add(oc, [s * 0.14, 0.0925, 0.05 + d1]), np.add(oc, [s * 0.14, 0.185, 0.05 + d1]), np.add(oc, [s * 0.14, 0, 0.05 + d1]),
                       np.add(oc, [s * 0.14, -0.0925, 0.05 + d1]), np.add(oc, [s * 0.14, -0.185, 0.05 + d1])]
            self.diam_obs = [0.18] * len(self.obs_pos)  # Main obstacle repulsive field
            self.diam_obs[0] = 0.3
            self.add_sphere(self.obs_pos, self.diam_obs, ColorRGBA(1.0, 0.0, 0.0, 0.5))


        if args.realUR5:
            self.clientreal = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
            print "Waiting for server (real)..."
            self.clientreal.wait_for_server()
            print "Connected to server (real)"

    """
    TH Matrix from joint angles
    """

    def matrix_from_joint_angles(self):
            th1, th2, th3, th4, th5, th6 = self.joint_states.position
            d1, SO, EO, a2, a3, d4, d45, d5, d6 = self.ur5_param

            matrix = [[-(sin(th1)*sin(th5) + cos(th1)*cos(th5)*cos(th2 + th3 + th4))*cos(th6) + sin(th6)*sin(th2 + th3 + th4)*cos(th1), (sin(th1)*sin(th5) + cos(th1)*cos(th5)*cos(th2 + th3 + th4))*sin(th6) + sin(th2 + th3 + th4)*cos(th1)*cos(th6), -sin(th1)*cos(th5) + sin(th5)*cos(th1)*cos(th2 + th3 + th4), -EO*sin(th1) - SO*sin(th1) + a2*cos(th1)*cos(th2) + a3*cos(th1)*cos(th2 + th3) - d45*sin(th1) - d5*sin(th2 + th3 + th4)*cos(th1) - d6*(sin(th1)*cos(th5) - sin(th5)*cos(th1)*cos(th2 + th3 + th4))], [-(sin(th1)*cos(th5)*cos(th2 + th3 + th4) - sin(th5)*cos(th1))*cos(th6) + sin(th1)*sin(th6)*sin(th2 + th3 + th4), (sin(th1)*cos(th5)*cos(th2 + th3 + th4) - sin(th5)*cos(th1))*sin(th6) + sin(th1)*sin(th2 + th3 + th4)*cos(th6), sin(th1)*sin(th5)*cos(th2 + th3 + th4) + cos(th1)*cos(th5), EO*cos(th1) + SO*cos(th1) + a2*sin(th1)*cos(th2) + a3*sin(th1)*cos(th2 + th3) + d45*cos(th1) - d5*sin(th1)*sin(th2 + th3 + th4) + d6*(sin(th1)*sin(th5)*cos(th2 + th3 + th4) + cos(th1)*cos(th5))], [sin(th6)*cos(th2 + th3 + th4) + sin(th2 + th3 + th4)*cos(th5)*cos(th6), -sin(th6)*sin(th2 + th3 + th4)*cos(th5) + cos(th6)*cos(th2 + th3 + th4), -sin(th5)*sin(th2 + th3 + th4), -a2*sin(th2) - a3*sin(th2 + th3) + d1 - d5*cos(th2 + th3 + th4) - d6*sin(th5)*sin(th2 + th3 + th4)], [0, 0, 0, 1]]
            return matrix

    """
    Get UR5 Inverse Kinematics
    """

    def get_ik(self, pose):
        matrix = tf.TransformerROS()
        q = quaternion_from_euler(1.5707, 1.5707, 0)

        matrix2 = matrix.fromTranslationRotation(
            (pose[0] * (-1), pose[1] * (-1), pose[2]), (q[0], q[1], q[2], q[3]))

        rospy.loginfo(matrix2)
        th = invKine(matrix2)
        sol1 = th[:, 0].transpose()
        joint_values_from_ik = np.array(sol1)

        joint_values = joint_values_from_ik[0, :]

        return joint_values.tolist()

    """
    Also gets each frame position through lookupTransform
    """

    def get_repulsive_cp(self, obs_pos, joint_values, CP_ur5_rep):

        marker_lines = Marker()

        ur5_links = [
            "upper_arm_link",
            "forearm_link",
            "wrist_1_link",
            "wrist_2_link",
            "wrist_3_link",
            "grasping_link"
        ]

        CP_pos, CP_dist = [], []

        for i in range(len(ur5_links)):
            link_pose = get_ur5_position(
                self.ur5_param, joint_values, ur5_links[i])
            CP_pos.append(link_pose)
            CP_inter = []
            # self.add_sphere2(link_pose, CP_ur5_rep, ColorRGBA(1.0, 0.0, 0.0, 0.5)) # Plot UR5 repulsive fields
            for y in range(len(obs_pos)):
                CP_inter.append(np.linalg.norm(link_pose - obs_pos[y]))
            CP_dist.append(CP_inter)
        return CP_pos, CP_dist

    """
    Adds the obstacles and repulsive control points on the robot
    """

    def add_sphere(self, pose, diam, color):
        marker = Marker()
        marker.header.frame_id = "base_link"
        for i in range(len(pose)):
            marker.id = self.id
            marker.pose.position = Point(pose[i][0], pose[i][1], pose[i][2])
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale = Vector3(diam[i], diam[i], diam[i])
            marker.color = color
            self.marker_publisher.publish(marker)
            self.id += 1

    """
    This function plot UR5 Repulsive Fields
    """

    def add_sphere2(self, pose, diam, color):
        marker = Marker()
        if self.id2 == 137:
            self.id2 = 130
        marker.header.frame_id = "base_link"
        marker.id = self.id2
        marker.pose.position = Point(pose[0], pose[1], pose[2])
        marker.type = marker.SPHERE
        marker.action = marker.MODIFY
        marker.scale = Vector3(diam, diam, diam)
        marker.color = color
        self.marker_publisher.publish(marker)
        self.id2 += 1

    """
    Add Cylinder pose
    """
    def add_obstacles(self, name, height, radius, pose, orientation, r, g, b):
        scene = self.scene
        # addCylinder (self, name, height, radius, x, y, z, use_service=True)
        s = SolidPrimitive()
        s.dimensions = [height, radius] # [height, radius]
        s.type = s.CYLINDER

        ps = Pose()
        # ps.header.frame_id = "cylinder1"
        ps.position.x = pose[0]
        ps.position.y = pose[1]
        ps.position.z = pose[2]
        x, y, z, w = quaternion_from_euler(orientation[0], orientation[1], orientation[2])
        ps.orientation.x = x
        ps.orientation.y = y
        ps.orientation.z = z
        ps.orientation.w = w
        scene.addSolidPrimitive(name, s, ps)
        scene.setColor(name, r, g, b)

    """
    Plot robot's path to the RViz environment
    """

    def visualize_path_planned(self, path, G = 1.0, B = 0.0):
        self.marker.points.append(Point(path[0], path[1], path[2]))
        self.marker.header.frame_id = "base_link"
        self.marker.id = self.id_path
        self.marker.type = self.marker.LINE_STRIP
        self.marker.action = self.marker.ADD
        self.marker.scale = Vector3(0.008, 0.009, 0.1)
        self.marker.color = self.path_color
        self.marker_publisher.publish(self.marker)

    """
    Send final trajectory to gazebo or real UR5
    """

    def move(self, way_points, target):
        g = FollowJointTrajectoryGoal()
        g.trajectory = JointTrajectory()
        g.trajectory.joint_names = self.joint_states.name

        # for joint in range(len(way_points)):
        hz = get_param("rate", 30)  # 10hz
        r = rospy.Rate(hz)

        try:
            i = 0
            while not rospy.is_shutdown() and i < len(way_points):
                g.trajectory.points.append(JointTrajectoryPoint(positions=way_points[i],
                                                                velocities=[0] * 6,
                                                                time_from_start=rospy.Duration(0.4 * i + 1))) #default 0.1
                i += 1

            if target == "gazebo":
                self.client.send_goal(g)
                self.client.wait_for_result()
            elif target == "real":
                self.clientreal.send_goal(g)
                self.clientreal.wait_for_result()

        except KeyboardInterrupt:
            self.client.cancel_goal()
            self.clientreal.cancel_goal()
            raise
        except:
            raise

    def CPA_loop(self, args, ur5_robot, way_points, R, P, Y):


        '# add_obstacles(name, height, radius, pose, orientation, r, g, b):'
        # ur5_robot.add_obstacles("up", 0.54, 0.09, [-0.76, 0, 0.345], [1.5707, 1.5707, 0], 1, 0, 0)
        # ur5_robot.add_obstacles("bottom", 0.54, 0.09, [-0.76, 0, 0.55], [1.5707, 1.5707, 0], 1, 0, 0)
        # ur5_robot.add_obstacles("right", 0.35, 0.09, [-0.76, 0.185, 0.455], [0, 0, 0], 1, 0, 0)
        # ur5_robot.add_obstacles("left", 0.35, 0.09, [-0.76, -0.185, 0.455], [0, 0, 0], 1, 0, 0)

        self.add_sphere(ur5_robot.ptFinal, ur5_robot.diam_goal, ColorRGBA(0.0, 1.0, 0.0, 0.8))

        # apply obstacle colors to moveit
        ur5_robot.scene.sendColors()

        # Final position
        Displacement = [0.01, 0.01, 0.01]

        # CPA Parameters
        err = ur5_robot.diam_goal[0] / 2  # Max error allowed
        max_iter = 1500  # Max iterations
        zeta = [0.5 for i in range(7)]  # Attractive force gain of each obstacle
        eta = [0.00006 for i in range(6)]  # Repulsive gain of each obstacle
        rho_0 = [i / 2 for i in self.diam_obs]  # Influence distance of each obstacle
        dist_att = 0.05  # Influence distance in workspace
        dist_att_config = 0.2  # Influence distance in configuration space
        alfa = 0.5  # Grad step of positioning - Default: 0.5
        alfa_rot = 0.05  # Grad step of orientation - Default: 0.4
        CP_ur5_rep = 0.15  # Repulsive fields on UR5

        ptAtual = get_ur5_position(ur5_robot.ur5_param, ur5_robot.joint_states.position, "grasping_link")

        hz = get_param("rate", 120)
        r = rospy.Rate(hz)

        dist_EOF_to_Goal = np.linalg.norm(ptAtual - np.asarray(ur5_robot.ptFinal[0]))
        dist_EOF_to_Goal_vec = dist_EOF_to_Goal

        n = 0
        err_ori = 1
        corr = [R, P, Y]

        # Get current orientation and position of grasping_link
        ur5_joint_positions_vec = ur5_robot.joint_states.position

        # ur5_joint_positions_vec = []


        while (dist_EOF_to_Goal > err  or abs(err_ori) > 0.02) and not rospy.is_shutdown() and n < max_iter:


            # Get UR5 Jacobian of each link
            Jacobian = get_geometric_jacobian(
                ur5_robot.ur5_param, ur5_robot.joint_states.position)

            # Get position and distance from each link to each obstacle
            CP_pos, CP_dist = ur5_robot.get_repulsive_cp(self.obs_pos, ur5_robot.joint_states.position, CP_ur5_rep)

            oriAtual = euler_from_matrix(ur5_robot.matrix_from_joint_angles())
            oriAtual = [oriAtual[i] + corr[i] for i in range(len(corr))]

            # Get attractive linear and angular forces and repulsive forces
            joint_att_force_p, joint_att_force_w, joint_rep_force = \
                CPA.get_joint_forces(ur5_robot.AAPF, ptAtual, ur5_robot.ptFinal, oriAtual, Displacement,
                                     dist_EOF_to_Goal, Jacobian, ur5_robot.joint_states.position, ur5_robot.ur5_param, zeta,
                                     eta, rho_0, dist_att, dist_att_config, CP_dist, CP_pos, self.obs_pos, CP_ur5_rep)

            # Joint angles UPDATE - Attractive force
            ur5_robot.joint_states.position = ur5_robot.joint_states.position + \
                alfa * joint_att_force_p[0]
            if arg.OC_Off:
                ur5_robot.joint_states.position = ur5_robot.joint_states.position + \
                    alfa_rot * joint_att_force_w[0]

            # Joint angles UPDATE - Repulsive force
            list = np.transpose(joint_rep_force[0]).tolist()
            for j in range(6):
                for i in range(6):
                    ur5_robot.joint_states.position[i] = ur5_robot.joint_states.position[i] + \
                        alfa * list[j][i]

            matrix = ur5_robot.matrix_from_joint_angles()

            # Get current position of grasping_link
            ptAtual = get_ur5_position(ur5_robot.ur5_param, ur5_robot.joint_states.position, "grasping_link")

            if ur5_robot.plot_path:
                # Visualize path planned in RVIZ
                ur5_robot.visualize_path_planned(ptAtual)

            # Get absolute orientation error
            err_ori = np.sum(oriAtual)

            # Get distance from EOF to goal
            dist_EOF_to_Goal = np.linalg.norm(ptAtual - np.asarray(ur5_robot.ptFinal))

            # If true, publish topics to transform into csv later on
            if arg.CSV:
                ur5_joint_positions_vec = np.vstack((ur5_joint_positions_vec, ur5_robot.joint_states.position))
                dist_EOF_to_Goal_vec = np.vstack((dist_EOF_to_Goal_vec, dist_EOF_to_Goal))

            # If true, publish topics to publish_trajectory.py in order to see the path in RVIZ
            # if n % 10 is used to reduced the total number of waypoints generated by APF or AAPF
            if arg.plot:
                if n % 1 == 0:
                    ur5_robot.pose.pose.position.x = ptAtual[0]
                    ur5_robot.pose.pose.position.y = ptAtual[1]
                    ur5_robot.pose.pose.position.z = ptAtual[2]
                    ur5_robot.pose_publisher.publish(ur5_robot.pose)
                    # Save way points in order to send it to gazebo
                    way_points.append(ur5_robot.joint_states.position)

            try:
                r.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                pass

            n += 1

        return way_points, n, dist_EOF_to_Goal, ur5_joint_positions_vec, dist_EOF_to_Goal_vec

    def trajectory_execution(self, args, point, home_pos, way_points, traj, ur5_robot_APF = None):
        raw_input("' =========== Aperte enter para iniciar o algoritmo dos CPAAs")

        if ur5_robot_APF:
            ur5_robot_APF.scene.clear

        self.scene.clear()

        # Final positions
        self.ptFinal = point

        # Angle correction relative to base_link (from grasping_link)
        R, P, Y = 1.5707, 0, 1.5707

        way_points, n, dist_EOF_to_Goal, ur5_joint_positions_vec, dist_EOF_to_Goal_vec = \
            self.CPA_loop(self.AAPF, self, way_points, R, P, Y)

        # Smooth path generated by AAPF
        wayPointsSmoothed = smooth_path(way_points)

        # print_output(n, way_points, wayPointsSmoothed, dist_EOF_to_Goal)

        if arg.COMP and arg.CSV:
            raw_input("' =========== Aperte enter para iniciar o algoritmo dos CPAs (classicos)")
            way_points2 = []
            ur5_robot_APF.ptFinal = point
            ur5_robot_APF.plot_path = True
            ur5_robot_APF.AAPF = False # Use False as default in order to use APF instead of AAPF for comparison
            way_points2.append(ur5_robot_APF.joint_states.position)

            _, _, _, ur5_joint_positions_vec_APF, dist_EOF_to_Goal_vec_APF = \
                self.CPA_loop(args, ur5_robot_APF, way_points2, R, P, Y)

            # len1 = len(dist_EOF_to_Goal_vec)
            # len2 = len(dist_EOF_to_Goal_vec_APF)
            # raw_input(len1)
            # raw_input(len2)
            # list = [len1, len2]
            # max_len = min(list)
            dist_EOF_to_GOAL_traj = []
            # dist_EOF_to_GOAL_traj = np.hstack((dist_EOF_to_Goal_vec[:max_len], dist_EOF_to_Goal_vec_APF[:max_len]))
            dist_EOF_to_GOAL_traj.append(dist_EOF_to_Goal_vec)
            dist_EOF_to_GOAL_traj.append(dist_EOF_to_Goal_vec_APF)

            print_joint_angles(args, ur5_joint_positions_vec, 'joint_' + str(traj) + '_AAPF', 'joint')
            print_joint_angles(args, wayPointsSmoothed, 'joint_' + str(traj) + '_smoothed_AAPF', 'joint')
            print_joint_angles(args, ur5_joint_positions_vec_APF, 'joint_' + str(traj) + '_APF', 'joint')
            print_joint_angles(args, dist_EOF_to_GOAL_traj, 'dist_to_goal_' + str(traj) + '_AAPF_e_APF', 'dist_to_goal')
        else:
            print_joint_angles(args, ur5_joint_positions_vec, 'joint_' + str(traj) + '_AAPF', 'joint')
            print_joint_angles(args, wayPointsSmoothed, 'joint_' + str(traj) + '_smoothed_AAPF', 'joint')
            print_joint_angles(args, dist_EOF_to_Goal_vec, 'dist_to_goal_' + str(traj) + '_AAPF', 'dist_to_goal')

        if arg.plot:
            # choose the trajectory to display in RVIZ
            self.pose.header.frame_id = "trajectory"
            self.pose_publisher.publish(self.pose)

        raw_input("' =========== Press enter to send the trajectory to Gazebo \n")
        self.move(wayPointsSmoothed, "gazebo")

        if args.realUR5:
            raw_input("' =========== Aperte enter para enviar a trajetoria para o UR5 !!!REAL!!! \n")
            self.move(wayPointsSmoothed, "real")


def main(args):
    ur5_robot = MoveGroupPythonIntefaceTutorial(args)
    way_points = []
    ur5_robot.scene.clear()

    # UR5 Initial position
    raw_input("' =========== Aperte enter para posicionar o UR5 \n")
    home_pos = [0, -1.5707, 0, -1.5707, -1.5707, -1.5707]
    # home_pos = [-0.39491863, -1.72555185,  1.92111876, -3.44760581, -1.10707033, -0.09005711]
    ur5_robot.joint_states.position = home_pos
    ur5_robot.path_color = ColorRGBA(0.0, 1.0, 0.0, 0.8)
    way_points.append(ur5_robot.joint_states.position)
    ur5_robot.move(way_points, "gazebo")

    if args.realUR5:
        raw_input("' =========== Aperte enter para posicionar o UR5 real na posicao UP\n")
        self.move((home_pos,), "real")

    ur5_robot_APF = None
    if args.COMP:
        ur5_robot_APF = MoveGroupPythonIntefaceTutorial(args, comp = True)
        ur5_robot_APF.joint_states.position = home_pos
        ur5_robot_APF.id_path = 201
        ur5_robot_APF.path_color = ColorRGBA(0.0, 0.0, 1.0, 0.8)

    '''
    Trajectory 1

    '''
    point = [[0.55, 0, 0.55]]
    way_points = []

    ur5_robot.trajectory_execution(args, point, home_pos, way_points, 'traj_1', ur5_robot_APF)

    '''
    Trajectory 2

    '''
    point = [[0.80, 0, 0.55]]
    way_points = []
    ur5_robot.trajectory_execution(args, point, home_pos, way_points, 'traj_2', ur5_robot_APF)

if __name__ == '__main__':
    try:
        arg = parse_args()
        main(arg)
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        pass
