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

from nav_msgs.msg import Path, Odometry


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

    # initialize velocities calculation
    N = len(obstacles_config)
    times.resize((N, 1))
    velocities.resize((N, 3))
    prev_pos.resize((N, 3))

    # set iniitial values
    prev_pos.fill(0)
    velocities.fill(0)
    times.fill(rospy.Time.now())

    cyl_config = []
    odom_topics = []
    for index, cyl in enumerate(obstacles_config):
        cylinder = CylinderObstacleData()

        cylinder.radius = cyl['radius']
        cylinder.height = cyl['height']
        cylinder.pos = [cyl['x'], cyl['y'], cyl['z']]

        roll = cyl['roll'] * pi/180
        pitch = cyl['pitch'] * pi/180
        yaw = 0  # there is no point of a cylinder having yaw rotation
        cylinder.quat = transformations.quaternion_from_euler(roll, pitch, yaw)

        cylinder.vel = [0, 0, 0]
        cylinder.angVel = [0, 0, 0]

        cyl_config.append(cylinder)

        odom_topics.append(cyl["odom_name"])

    return cyl_config, odom_topics


def callback(odom: Odometry, i: int):
    # global conf
    curr_t = rospy.Time.now()
    dt: rospy.Duration = curr_t - times[i][0]

    # set position and orientation to config
    conf[i].pos = [odom.pose.pose.position.x,    odom.pose.pose.position.y, odom.pose.pose.position.z]
    conf[i].quat = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]

    # calculate velocities
    delta_pos = np.array(conf[i].pos) - prev_pos[i]
    velocities[i] = delta_pos / dt.to_sec()

    # save position for future calculations
    prev_pos[i] = np.array(conf[i].pos)

    # set velocities to config
    conf[i].vel = velocities[i]
    # conf[i].angVel=[0,0,0] TODO: not implemented yet

    print(("Received odom for cyl: {} with pos: {}".format(i, conf[i].pos)))
    times[i][0] = curr_t


# GLOBAL VARIABLES
times_service_called = 0
total_time = 0
conf = []

prev_pos = np.array([], dtype=np.float32)
times = np.array([], dtype=rospy.Time)
velocities = np.array([], dtype=np.float32)


if __name__ == "__main__":
    rospy.init_node('realtime_interface')

    conf, odom_names = load_obstacles_config()

    for id, name in enumerate(odom_names):
        topic_name = "/pixy/vicon/{}/{}/odom".format(name, name)
        print("Subscribing to: ", topic_name)
        rospy.Subscriber(topic_name, Odometry, callback, callback_args=id)

    # start and goal are not used currently but planning to be used in the future as a high level interface with the planner
    start = [0, 0, 0]
    goal = [0, 0, 0]
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
        times_service_called += 1
        print("Average time of calling service: ", total_time/times_service_called, "sec")

        rate.sleep()
