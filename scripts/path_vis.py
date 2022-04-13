#!/usr/bin/env python3
# print working directory
from geometry_msgs.msg import TransformStamped, Quaternion
import math
import rospy
from tf import TransformListener, transformations
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from math import pi
import tf
import os
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs

from ompl import base as ob
from ompl import geometric as og

print("Current working directory:", os.getcwd())
DRONES_NUMBER = 5


class MeshMarker(Marker):
    def __init__(self, id, mesh_path, pos=[0, 0, 0], rot=[0, 0, 0, 1]):
        super().__init__()
        self.header.frame_id = "world"
        self.header.stamp = rospy.get_rostime()
        self.ns = "collada_mesh"
        self.id = id
        self.type = Marker.MESH_RESOURCE
        self.mesh_resource = mesh_path
        self.action = 0

        self.updatePose(pos, rot)

        scale_fac = 1
        self.scale.x = scale_fac
        self.scale.y = scale_fac
        self.scale.z = scale_fac

        self.color.r = 0.0
        self.color.g = 1.0
        self.color.b = 0.0
        self.color.a = 1.0

        self.lifetime = rospy.Duration(0)

    def updatePose(self, pos, quatern, frame="world"):
        try:
            self.pose.position.x = pos[0]
            self.pose.position.y = pos[1]
            self.pose.position.z = pos[2]

            self.pose.orientation.x = quatern[0]
            self.pose.orientation.y = quatern[1]
            self.pose.orientation.z = quatern[2]
            self.pose.orientation.w = quatern[3]
        except:
            self.pose.position = pos
            self.pose.orientation = quatern


def getPath(data):
    path = Path()
    path.header.frame_id = "world"
    path.header.stamp = rospy.get_rostime()
    path.poses = []
    print("Path size:", len(data))
    for i in range(data.shape[0]):
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.header.stamp = rospy.get_rostime()

        pose.pose.position.x = data[i, 0]
        pose.pose.position.y = data[i, 1]
        pose.pose.position.z = data[i, 2]

        q = tf.quaternion_from_euler(0, 0, data[i, 3])
        pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

        # transformed_pose = transform(pose,  inverse=True)
#
        # pose.pose.orientation.x = data[i, 3]
        # pose.pose.orientation.y = data[i, 4]
        # pose.pose.orientation.z = data[i, 5]
        # pose.pose.orientation.w = data[i, 6]

        path.poses.append(pose)

    return path


if __name__ == "__main__":

    rospy.init_node("rb_path_planning")
    # transform()

    # robot marker initialization
    mesh = "package://drones_rope_planning/resources/collada/custom_triangle_robot.dae"
    rb = MeshMarker(id=0, mesh_path=mesh)

    robPub = rospy.Publisher('rb_robot',  Marker, queue_size=10)

    # Environment marker initialization
    mesh = "package://drones_rope_planning/resources/collada/env-scene-ltu-experiment.dae"
    env = MeshMarker(id=1, mesh_path=mesh)
    env.color.r = 1
    env.color.g = 0
    env.color.b = 0
    env.updatePose([0, 0, 0], [0, 0, 0, 1])
    envPub = rospy.Publisher('rb_environment',  Marker, queue_size=10)

    # path
    try:
        path_file = "/home/marios/thesis_ws/src/drones_rope_planning/resources/paths/path.txt"
        data = np.loadtxt(path_file)
    except Exception as e:
        print("Error:", e)
        print("Trying crazyswarm/path.txt...")
        try:
            data = np.loadtxt('crazyswarm/path.txt')
        except Exception as e:
            print("No path file found")
            exit(0)

    path = getPath(data)
    trajPub = rospy.Publisher('rigiBodyPath',  Path, queue_size=10)
    trajPub.publish(path)

    # transform
    br = tf.TransformBroadcaster()

    i = 0
    rate = rospy.Rate(10.0)  # hz
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(-math.pi/2, 0, 0), rospy.Time.now(),
                         "world", "ompl")

        if i == data.shape[0]-1:
            i = 0
        else:
            i += 1

        rb.updatePose(path.poses[i].pose.position,
                      path.poses[i].pose.orientation, frame="world")

        # rb_transformed = transform(rb)

        # print("robot pos:", rb_transformed.pose.position.x,
        #   rb_transformed.pose.position.y, rb_transformed.pose.position.z, " orient:", rb_transformed.pose.orientation.x, rb_transformed.pose.orientation.y, rb_transformed.pose.orientation.z, rb_transformed.pose.orientation.w)

        robPub.publish(rb)

        pos = (rb.pose.position.x, rb.pose.position.y, rb.pose.position.z)
        orientation = (rb.pose.orientation.x, rb.pose.orientation.y,
                       rb.pose.orientation.z, rb.pose.orientation.w)

        br.sendTransform(pos, orientation, rospy.Time.now(),
                         "rigid_body", "world")

        envPub.publish(env)
        trajPub.publish(path)

        rate.sleep()
