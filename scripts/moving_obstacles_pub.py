#!/usr/bin/env python3
# print working directory
from cmath import cos, sin

from sympy import Q
from geometry_msgs.msg import TransformStamped, Quaternion
import rospy
from tf import TransformListener, transformations
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from math import pi
import tf
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import rospkg

from drones_rope_planning.srv import PlanningRequest
from drones_rope_planning.msg import CylinderObstacleData

from nav_msgs.msg import Path


def config_to_odoms() -> list:
    """
    Converts the obstacles configuration to the odom names.
    Returns:
        list of Odoms: [Odoms]
    """
    obstacles_config = rospy.get_param("/obstacles/cylinders")
    print("obstacles_config: ", obstacles_config)

    odom_names = []

    odoms = []
    for i in obstacles_config:
        odom = Odometry()
        odom.header.frame_id = "world"
        odom.child_frame_id = i['odom_name']

        try:
            pos = [i["x"], i["y"], i["z"]]
            roll = i["roll"] * pi / 180
            pitch = i["pitch"] * pi / 180
            yaw = 0
            q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        except:
            pos = [0, 4, 0]
            q = [0, 0, 0, 1]

        odom.pose.pose.position.x = pos[0]
        odom.pose.pose.position.y = pos[1]
        odom.pose.pose.position.z = pos[2]

        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odoms.append(odom)
        odom_names.append(i['odom_name'])

    return odoms, odom_names


# Global Variables
pos = [0, 0, 0]
new_odom_received = False

if __name__ == "__main__":
    """
        A path is used to hold and publish the obstacles configuration
        in the form of a list of poses.
        e.g.
        The pose of the first obstacle is stored in the first pose of the path(path.poses[0])
    """
    rospy.init_node('moving_obstacles_pub')

    odoms, odoms_names = config_to_odoms()

    pubs = []
    for i, name in enumerate(odoms_names):
        topic_name = "pixy/vicon/{}/{}/odom".format(name, name)
        pubs.append(rospy.Publisher(topic_name, Odometry, queue_size=10))

    last_reset = 0
    center = [.5, 4]
    t = 0
    rate = rospy.Rate(28)  # hz
    while not rospy.is_shutdown():
        odoms[0].header.stamp = rospy.Time.now()
        odoms[1].header.stamp = rospy.Time.now()

        period = 8

        odoms[0].pose.pose.position.x = -np.cos(2*np.pi*t/period)

        if len(odoms) > 1:
            pass

        # print("t: ", t, " x: ", path.poses[0].pose.position.x, " y: ", path.poses[0].pose.position.y)

        t += rate.sleep_dur.to_sec()

        for i in range(len(odoms)):
            pubs[i].publish(odoms[i])

        rate.sleep()
