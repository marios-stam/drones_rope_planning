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

from nav_msgs.msg import Odometry

from drones_rope_planning.srv import PlanningRequest
from drones_rope_planning.msg import CylinderObstacleData

from nav_msgs.msg import Path
from functools import partial


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
        list of type:[radius,height,odom_name]
    """
    # load ros param
    obstacles_config = rospy.get_param("/obstacles/cylinders")
    print("obstacles_config: ", obstacles_config)

    cyl_config = []
    for index, cyl in enumerate(obstacles_config):
        cyl_data = {}
        cyl_data["radius"] = cyl["radius"]
        cyl_data["height"] = cyl["height"]
        cyl_data["odom_name"] = cyl["odom_name"]

        cyl_config.append(cyl_data)

    return cyl_config


def callback(odom: Odometry, id: int):

    config[id].pos = [odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z]
    config[id].quat = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]

    config[id].vel = [odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z]
    config[id].ang_vel = [odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z]


times = 0
total_time = 0

"""
config: list of type:
        [pos, quat, vel, ang_vel]
    
    It is used to store the current configuration of the environment obstacles
    and to send it to the service.
"""
config = []


if __name__ == "__main__":
    rospy.init_node('realtime_interface')

    conf = load_obstacles_config()

    # create odometry subscribers
    for index, cyl in enumerate(conf):
        odom_name = cyl["odom_name"]
        rospy.Subscriber(odom_name, Odometry, partial(callback, id=index))

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
        print("Average time of calling service: ", total_time/times, "sec")

        rate.sleep()
