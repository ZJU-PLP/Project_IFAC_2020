#!/usr/bin/env python
import rospy
import tf
from tf import TransformListener
from geometry_msgs.msg import Pose, Point, Vector3, PoseStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
path, trajectory = Marker(), Marker()
path_id, traj_id = 1000, 2000

def visualize_path_planned(point, marker, id, color):
    marker.points.append(point)
    marker.header.frame_id = "base_link"
    marker.id = id
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD
    marker.scale = Vector3(0.008, 0.009, 0.1)
    marker.color = color
    marker_publisher.publish(marker)

def delete_all(path, path_id, trajectory, traj_id):
    path.header.frame_id = "base_link"
    path.id = path_id
    path.action = path.DELETE
    marker_publisher.publish(path)

    trajectory.header.frame_id = "base_link"
    trajectory.id = traj_id
    trajectory.action = trajectory.DELETE
    marker_publisher.publish(trajectory)


# It was required to create this function because we have to plot the trajectory when UR5
# is executing the motion in gazebo

def pose_publisher_callback(PoseStampedMsg):
    global trajectory, path, path_id, traj_id

    if(PoseStampedMsg.header.frame_id == "path"):
        visualize_path_planned(PoseStampedMsg.pose.position, path, path_id, ColorRGBA(0.0, 1.0, 0.0, 0.8))
    elif(PoseStampedMsg.header.frame_id == "trajectory"):
        while PoseStampedMsg.header.frame_id == "trajectory":
            try:
                eof_position, _ = tf_listener.lookupTransform("base_link", "grasping_link", rospy.Time())
                visualize_path_planned(Point(eof_position[0],eof_position[1], eof_position[2]), trajectory, traj_id, ColorRGBA(0.0, 0.0, 1.0, 0.8))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
    elif(PoseStampedMsg.header.frame_id == "clear"):
        delete_all(path, path_id, trajectory, traj_id)
    else:
        pass


def listener():
    #
    # rospy.sleep(0.5)
    rospy.Subscriber("pose_publisher_tp", PoseStamped, pose_publisher_callback)
    rospy.spin()

if __name__ == '__main__':
    # global path_id, traj_id
    rospy.init_node('publish_trajectory', anonymous=True)

    rospy.loginfo("Initiating publish_trajectory...")
    marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=100)

    tf_listener = tf.TransformListener()


    listener()
