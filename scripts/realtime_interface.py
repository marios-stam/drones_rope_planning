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

import rospkg

from drones_rope_planning.srv import PlanningRequest
from drones_rope_planning.msg import CylinderObstacleData

from nav_msgs.msg import Path, Odometry


def service_client():
    rospy.wait_for_service(planning_service_name)
    try:
        print("Calling service...")
        ompl_realtime_srv = rospy.ServiceProxy(planning_service_name, PlanningRequest)
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
        cylinder.pos = [0, 0, 0]

        roll = 0
        pitch = 0
        yaw = 0  # there is no point of a cylinder having yaw rotation
        cylinder.quat = transformations.quaternion_from_euler(roll, pitch, yaw)

        cylinder.vel = [0, 0, 0]
        cylinder.angVel = [0, 0, 0]

        cyl_config.append(cylinder)

        odom_topics.append(cyl["odom_name"])

    return cyl_config, odom_topics


def callback_demo(odom: Odometry, i: int):
    # global conf
    curr_t = rospy.Time.now()
    dt: rospy.Duration = curr_t - times[i][0]

    # set position and orientation to config
    conf[i].pos = [odom.pose.pose.position.x,    odom.pose.pose.position.y, odom.pose.pose.position.z]
    # conf[i].quat = [odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z, odom.pose.pose.orientation.w]
    conf[i].quat = [0, 0, 0, 1]

    # velocities and angular velocities
    conf[i].vel = [odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z]
    # conf[i].angVel = [odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z]


def callback_sim(odom: Odometry, i: int):
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

    # get args
    # args = rospy.myargv(argv=sys.argv)

    using_simulation = rospy.get_param("/is_simulation")
    if (using_simulation):
        # The configuration is in simulation mode
        callback = callback_sim
    else:
        # The configuration is in demo mode
        callback = callback_demo

    conf, odom_names = load_obstacles_config()

    for id, name in enumerate(odom_names):
        topic_name = "/pixy/vicon/{}/{}/odom".format(name, name)
        print("Subscribing to: ", topic_name)
        rospy.Subscriber(topic_name, Odometry, callback, callback_args=id)

    # start and goal are not used currently but planning to be used in the future as a high level interface with the planner
    start = [0, 0, 0]
    goal = [0, 0, 0]
    # get ros parameter
    planning_freq = rospy.get_param("/planning/real_time_settings/planning_frequency")

    print("Planning frequency: ", planning_freq)

    # check if the service is available
    planning_service_name = '/drones_rope_planning_node/ompl_realtime_planning'
    try:
        rospy.wait_for_service(planning_service_name, timeout=0.5)
    except:
        planning_service_name = "/planning"+planning_service_name
        rospy.wait_for_service(planning_service_name, timeout=0.5)

    rate = rospy.Rate(planning_freq)
    while not rospy.is_shutdown():
        t0 = rospy.get_time()
        print("======================================================")
        # rospy.loginfo("Calling planning")
        print("Calling planning")
        service_client()
        dt = rospy.get_time()-t0
        total_time += dt
        times_service_called += 1
        print("Average time of calling service: ", total_time/times_service_called, "sec")

        rate.sleep()
