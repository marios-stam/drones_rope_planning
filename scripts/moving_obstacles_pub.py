#!/usr/bin/env python3
# print working directory
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
        cylinder.quat = [0, 0, 0, 1]

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
    global pos
    pos = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]


# Global Variables
pos = [0, 0, 0]

if __name__ == "__main__":
    rospy.init_node('moving_obstacles_pub')

    conf = load_obstacles_config()
    path = config_to_Path(conf)

    pub = rospy.Publisher("/obstacles_transforms", Path, queue_size=1)

    rospy.Subscriber("/obstacles_transforms_odometry", PoseStamped, callback)

    rate = rospy.Rate(2)  # hz
    while not rospy.is_shutdown():
        path.header.stamp = rospy.Time.now()
        pub.publish(path)

        # path.poses[0].pose.position.x = np.random.uniform(-1, 1)
        # path.poses[0].pose.position.y = np.random.uniform(3.5, 4.5)

        # input("Press Enter to continue...")
        rospy.loginfo(str(rospy.Time.now().to_sec()))
        path.poses[0].pose.position.x = pos[0]
        path.poses[0].pose.position.y = pos[1]

        rate.sleep()
