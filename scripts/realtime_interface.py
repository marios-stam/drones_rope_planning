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


def service_client():
    rospy.wait_for_service('/drones_rope_planning_node/ompl_realtime_planning')
    try:

        ompl_realtime_srv = rospy.ServiceProxy('/drones_rope_planning_node/ompl_realtime_planning', PlanningRequest)
        resp1 = ompl_realtime_srv(conf, start, goal)

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


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
        pitch = cyl['pitch'] * pi/180
        yaw = 0  # there is no point of a cylinder having yaw rotation
        cylinder.quat = transformations.quaternion_from_euler(roll, pitch, yaw)

        cyl_config.append(cylinder)

    return cyl_config


def callback(path: Path):
    global conf

    for i, pose in enumerate(path.poses):
        conf[i].pos = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z]
        conf[i].quat = [pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]


times = 0
total_time = 0
conf = []

if __name__ == "__main__":
    rospy.init_node('realtime_interface')

    start = [0, 0, 0]
    goal = [69, 69, 69]

    conf = load_obstacles_config()

    rospy.Subscriber("/obstacles_transforms", Path, callback)

    # get ros parameter
    planning_freq = rospy.get_param("/planning//real_time_settings/planning_frequency")

    rate = rospy.Rate(planning_freq)
    while not rospy.is_shutdown():
        t0 = rospy.get_time()
        print("======================================================")
        # rospy.loginfo("Calling planning")
        service_client()
        dt = rospy.get_time()-t0
        total_time += dt
        times += 1
        print("Average time: ", total_time/times, "sec")

        rate.sleep()
