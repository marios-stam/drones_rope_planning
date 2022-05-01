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
from nav_msgs.msg import Path
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
        self.scale.x = radius-0.2  # radius axis smaller for safety
        self.scale.y = radius-0.2  # radius axis smaller for safety
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


def generate_obstacles_markers():
    # load ros param
    N = rospy.get_param("/obstacles/cylinders_number")
    obstacles_config = rospy.get_param("/obstacles/cylinders")
    print("obstacles_config: ", obstacles_config)

    for index, cyl in enumerate(obstacles_config):
        r = cyl['radius']
        h = cyl['height']
        pos = [cyl['x'], cyl['y'], cyl['z']]
        quatern = [0, 0, 0, 1]

        cyls_marker_array.update(index, r, h, pos, quatern)


def callback(path: Path):

    for i, pose in enumerate(path.poses):
        pose: PoseStamped

        pos = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
        q = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]

        cyls_marker_array.update(i, 0.1, 0.1, pos, q)

    print("publishing markers:" + str(cyls_marker_array.markers[0].pose))

    env_markers_pub.publish(cyls_marker_array)


if __name__ == "__main__":
    rospy.init_node("rb_path_planning")

    env_markers_pub = rospy.Publisher('env_markers_array',  MarkerArray, queue_size=10)

    cyls_marker_array = EnvMarkersArray()

    conf = generate_obstacles_markers()

    sub = rospy.Subscriber("/obstacles_transforms", Path, callback)

    rospy.spin()
