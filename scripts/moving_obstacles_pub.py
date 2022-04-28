#!/usr/bin/env python3
# print working directory
from cmath import cos, sin
from geometry_msgs.msg import TransformStamped, Quaternion
import rospy
from tf import TransformListener, transformations
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from math import pi
import tf
from geometry_msgs.msg import PoseStamped

import rospkg

from drones_rope_planning.srv import PlanningRequest
from drones_rope_planning.msg import CylinderObstacleData

from nav_msgs.msg import Path


def load_obstacles_config() -> list:
    """
    Loads the obstacles configuration from the rosparams
    and prepares it to be sent throyght the service.

    Returns:
        list of CylinderObstacleData: The obstacles configuration.
    """
    # load ros param
    obstacles_config = rospy.get_param("/obstacles/cylinders")
    print("obstacles_config: ", obstacles_config)

    cyl_config = []
    for index, cyl in enumerate(obstacles_config):
        cylinder = CylinderObstacleData()

        cylinder.radius = cyl['radius']
        cylinder.height = cyl['height']
        cylinder.pos = [cyl['x'], cyl['y'], cyl['z']]

        roll = cyl['roll'] * pi/180
        pitch = cyl['pitch']*pi/180
        yaw = 0  # there is no point of a cylinder having yaw rotation

        cylinder.quat = transformations.quaternion_from_euler(roll, pitch, yaw)

        cyl_config.append(cylinder)

    return cyl_config


def config_to_Path(conf: list) -> Path:
    path = Path()
    path.header.frame_id = "world"
    path.poses = []
    for i in conf:
        pose = PoseStamped()
        pose.pose.position.x = i.pos[0]
        pose.pose.position.y = i.pos[1]
        pose.pose.position.z = i.pos[2]

        pose.pose.orientation.x = i.quat[0]
        pose.pose.orientation.y = i.quat[1]
        pose.pose.orientation.z = i.quat[2]
        pose.pose.orientation.w = i.quat[3]

        path.poses.append(pose)

    return path


def callback(pose: PoseStamped):
    global pos, new_odom_received

    pos = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
    new_odom_received = True


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

    conf = load_obstacles_config()
    path = config_to_Path(conf)

    pub = rospy.Publisher("/obstacles_transforms", Path, queue_size=1)

    # rospy.Subscriber("/obstacles_transforms_odometry", PoseStamped, callback)
    t = 0
    rate = rospy.Rate(30)  # hz
    while not rospy.is_shutdown():
        path.header.stamp = rospy.Time.now()

        # if new_odom_received:
        # rospy.loginfo(str(rospy.Time.now().to_sec()))
        # path.poses[0].pose.position.x = pos[0]
        # path.poses[0].pose.position.y = pos[1]
        # pub.publish(path)
        # new_odom_received = False

        # path.poses[0].pose.position.x = np.random.uniform(-1, 1)
        # path.poses[0].pose.position.y = np.random.uniform(3.5, 4.5)
        # input("Press Enter to continue...")

        period = 15

        path.poses[0].pose.position.x = np.cos(2*np.pi*t/period)
        # path.poses[0].pose.position.y = 4+0.5*np.sin(2*np.pi*t/period)
        # path.poses[0].pose.position.y = 4
        # path.poses[0].pose.position.z = 1+0.5*np.cos(2*np.pi*t/period)

        path.poses[1].pose.position.x = -np.cos(2*np.pi*t/period)
        # path.poses[0].pose.position.y = 4+0.5*np.sin(2*np.pi*t/period)
        # path.poses[0].pose.position.y = 4
        # path.poses[1].pose.position.z = 1-0.5*np.cos(2*np.pi*t/period)

        # print("t: ", t, " x: ", path.poses[0].pose.position.x, " y: ", path.poses[0].pose.position.y)

        t += rate.sleep_dur.to_sec()

        pub.publish(path)

        rate.sleep()
