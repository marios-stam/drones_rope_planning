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


def service_client():
    rospy.wait_for_service('/drones_rope_planning_node/ompl_realtime_planning')
    try:

        ompl_realtime_srv = rospy.ServiceProxy('/drones_rope_planning_node/ompl_realtime_planning', PlanningRequest)
        print(ompl_realtime_srv)
        resp1 = ompl_realtime_srv(conf, start, goal)

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


if __name__ == "__main__":
    rospy.init_node('realtime_interface')

    cyl = CylinderObstacleData()
    cyl.radius = 0.5
    cyl.height = 1

    cyl.pos = [0, 1, 2]

    start = [0, 0, 0]
    goal = [69, 69, 69]

    cyl2 = CylinderObstacleData()
    cyl2.radius = 0.75
    cyl2.height = 3

    cyl2.pos = [4, 5, 6]

    conf = [cyl, cyl2]
    service_client()
