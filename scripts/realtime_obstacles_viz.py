#!/usr/bin/env python3
# print working directory
import sys
from geometry_msgs.msg import TransformStamped, Quaternion
import rospy
from tf import TransformListener, transformations
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from math import pi
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
import rospkg


# get command line arguments
# CALCULATE_PATH = len(sys.argv) == 1 or sys.argv[1] == '1' or sys.argv[1].lower() == 'true'

class EnvMarkersArray(MarkerArray):
    def __init__(self):
        super().__init__()
        self.markers = []
        self.id_index_dict = {}

    def update(self, id, radius, height,  trans=[0, 0, 0], rot=[0, 0, 0, 1]):

        if id not in self.id_index_dict.keys():
            self.markers.append(CylinderMarker(id, radius, height,   trans, rot))

            # if no key in dictionary
            if (len(self.id_index_dict.values()) == 0):
                new_ind = 0
            else:
                new_ind = max(self.id_index_dict.values())+1

            self.id_index_dict[id] = new_ind
        else:
            index = self.id_index_dict[id]
            self.markers[index].updatePose(trans, rot)


class CylinderMarker(Marker):
    def __init__(self, id, radius, height, pos=[0, 0, 0], rot=[0, 0, 0, 1]):
        super().__init__()
        self.header.frame_id = "world"
        self.header.stamp = rospy.get_rostime()
        self.ns = "cylinders"
        self.id = id
        self.type = Marker.CYLINDER
        self.action = 0

        self.updatePose(pos, rot)

        scale_fac = 1
        self.scale.x = radius
        self.scale.y = radius
        self.scale.z = height

        self.color.r = 1.0
        self.color.g = 0.0
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


def generate_obstacles_markers(obstacles_config):
    # load ros param
    N = rospy.get_param("/obstacles/cylinders_number")
    print("obstacles_config: ", obstacles_config)

    for index, cyl in enumerate(obstacles_config):
        r = cyl['radius']-0.4  # radius  smaller for safety
        h = cyl['height']

        # pos
        try:
            pos = [cyl['x'], cyl['y'], cyl['z']]
            # orientation
            roll = cyl['roll']
            pitch = cyl['pitch']
            yaw = 0
            q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
            quatern = [q[0], q[1], q[2], q[3]]

        except:
            pos = [0, 4, 0]
            quatern = [0, 0, 0, 1]

        cyls_marker_array.update(index, r, h, pos, quatern)


def callback(odom: Odometry, id: int):

    pos = [odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z]

    if (odom.child_frame_id == "stik/stik"):
        q = [0, 0, 0, 1]
    else:
        q = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]

    print("Received id {} with pos:{}".format(id, pos))
    cyls_marker_array.update(id, 0.1, 0.1, pos, q)


if __name__ == "__main__":
    rospy.init_node("rb_path_planning")

    env_markers_pub = rospy.Publisher('env_markers_array',  MarkerArray, queue_size=10)

    cyls_marker_array = EnvMarkersArray()

    obstacles_config = rospy.get_param("/obstacles/cylinders")

    conf = generate_obstacles_markers(obstacles_config)
    odom_names = [cyl["odom_name"] for cyl in obstacles_config]

    for index, odom_name in enumerate(odom_names):
        topic_name = "/pixy/vicon/{}/{}/odom".format(odom_name, odom_name)
        print("Subscribing to topic: ", topic_name)
        rospy.Subscriber(topic_name, Odometry, callback, callback_args=index)

    f = 20
    rate = rospy.Rate(f)
    while not rospy.is_shutdown():
        env_markers_pub.publish(cyls_marker_array)
        # print("Published markers array")
        # print(cyls_marker_array)
        rate.sleep()

    rospy.spin()
