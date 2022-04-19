#!/usr/bin/env python3
# print working directory
import sys
from drone_path_planning.msg import rigid_body_dynamic_path
from geometry_msgs.msg import TransformStamped, Quaternion
import math
import rospy
from tf import TransformListener, transformations
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from math import pi
import tf
import os
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from RigidBodyPlanners import *
import yaml

from ompl import base as ob
from ompl import geometric as og

from catenaries.srv import CatLowestPoint, CatLowestPointResponse
from std_msgs.msg import String
from catenary import catenaries

import rospkg
# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()
pkg_drone_path_planning_path = rospack.get_path('drone_path_planning')
pkg_drones_rope_planning_path = rospack.get_path('drones_rope_planning')


# get command line arguments
CALCULATE_PATH = len(sys.argv) == 1 or sys.argv[1] == '1' or sys.argv[1].lower() == 'true'

PLANNERS_COMPARISON = 0


class MeshMarker(Marker):
    def __init__(self, id, mesh_path, pos=[0, 0, 0], rot=[0, 0, 0, 1]):
        super().__init__()
        self.header.frame_id = "world"
        self.header.stamp = rospy.get_rostime()
        self.ns = "collada_mesh"
        self.id = id
        self.type = Marker.MESH_RESOURCE
        self.mesh_resource = mesh_path
        self.action = 0

        self.updatePose(pos, rot)

        scale_fac = 1
        self.scale.x = scale_fac
        self.scale.y = scale_fac
        self.scale.z = scale_fac

        self.color.r = 0.0
        self.color.g = 1.0
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


def getPath(data):
    path = Path()
    path.header.frame_id = "world"
    path.header.stamp = rospy.get_rostime()
    path.poses = []
    print("Path size:", len(data))
    for i in range(data.shape[0]):
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.header.stamp = rospy.get_rostime()

        pose.pose.position.x = data[i, 0]
        pose.pose.position.y = data[i, 1]
        pose.pose.position.z = data[i, 2]

        q = tf.quaternion_from_euler(0, 0, data[i, 3])
        pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

        # transformed_pose = transform(pose,  inverse=True)
#
        # pose.pose.orientation.x = data[i, 3]
        # pose.pose.orientation.y = data[i, 4]
        # pose.pose.orientation.z = data[i, 5]
        # pose.pose.orientation.w = data[i, 6]

        path.poses.append(pose)

    return path


def generate_dynamic_path_msg(data):
    dynamic_path_msg = rigid_body_dynamic_path()

    drones_distances = np.zeros((len(data), 1))
    drones_angles = np.zeros((len(data), 1))

    path = Path()
    path.header.frame_id = "world"
    path.header.stamp = rospy.get_rostime()
    path.poses = []

    print("Path size:", len(data))
    for i in range(data.shape[0]):
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.header.stamp = rospy.get_rostime()

        pose.pose.position.x = data[i, 0]
        pose.pose.position.y = data[i, 1]
        pose.pose.position.z = data[i, 2]

        q = tf.quaternion_from_euler(0, 0, data[i, 3])
        pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
        path.poses.append(pose)

        drones_distances[i] = data[i, 4]
        drones_angles[i] = data[i, 5]

    dynamic_path_msg.Path = path
    dynamic_path_msg.drones_distances = drones_distances
    dynamic_path_msg.drones_angles = drones_angles

    return dynamic_path_msg


def calculate_path_FCL(robot_mesh_name, env_mesh_name):
    robot_mesh_name += ".stl"
    env_mesh_name += ".stl"

    print("robot_mesh_name:", robot_mesh_name)
    print("env_mesh_name:", env_mesh_name)

    planner = PlannerSepCollision(env_mesh_name, robot_mesh_name, catenaries.lowest_point_optimized, use_mesh_improvement=False)

    start = [-0.5, 3, 0.75]
    goal = [+0.5, 5, 0.75]

    start_pose = PoseStamped()
    start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z = start
    q = tf.quaternion_from_euler(0, 0, 0)
    start_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    goal_pose = PoseStamped()
    goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z = goal
    q = tf.quaternion_from_euler(0, 0, 0)
    goal_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    planner.set_start_goal(start_pose.pose, goal_pose.pose)
    if PLANNERS_COMPARISON:
        # list of all available og planners (maybe some are missing)
        planners = ['RRT', 'ABITstar', 'AITstar', 'BFMT', 'BITstar', 'BKPIECE1', 'BiEST',
                    'EST', 'FMT', 'InformedRRTstar', 'KPIECE1', 'KStarStrategy', 'KStrategy', 'LBKPIECE1', 'LBTRRT', 'LazyLBTRRT',
                    'LazyPRM', 'LazyPRMstar', 'LazyRRT', 'NearestNeighbors', 'NearestNeighborsLinear', 'NumNeighborsFn',
                    'PDST', 'PRM', 'PRMstar', 'ProjEST',
                    'QRRT', 'RRT', 'RRTConnect', 'RRTXstatic', 'RRTsharp', 'RRTstar', 'SBL', 'SORRTstar', 'SPARS', 'SPARStwo', 'SST']
        # planners = ['RRT', 'RRTConnect', 'RRTXstatic', 'RRTsharp']
        # solution_time ,states_tried,path_cost
        length_optim_objective = ob.PathLengthOptimizationObjective(
            planner.ss.getSpaceInformation())

        planners_results = np.zeros((4, len(planners)))
        for i, planner_name in enumerate(planners):
            print(
                "================================ %s ================================" % planner_name)
            planner_class = eval('og.' + planner_name)
            print(planner_class)
            try:
                planner.set_planner(planner_class)
                t0 = rospy.get_time()

                path, time, states_tried, avrg_time_per_valid_check = planner.solve(
                    timeout=60.0)
                path_cost = planner.path.cost(length_optim_objective)
                path_cost = float(path_cost.value())
            except Exception as e:
                print(e)
                continue

            planners_results[0, i] = rospy.get_time() - t0
            planners_results[1, i] = states_tried
            planners_results[2, i] = avrg_time_per_valid_check
            planners_results[3, i] = path_cost

            print("===============================================================")

            # save matrix as csv file at every iteration1
            file_name = pkg_drone_path_planning_path+'/resources/planners_results.csv'
            np.savetxt(file_name, planners_results,
                       delimiter=",", fmt='%f', header=",".join(planners))

    # planner.set_planner(og.RRTstar)
    planner.set_planner(og.RRT)
    opt_objective = getBalancedObjective(
        planner.ss.getSpaceInformation(),  rope_length=planner.L, cost_threshold=11)

    # planner.set_optim_objective(opt_objective)
    path = planner.solve(timeout=60.0)[0]
    # planner.visualize_path()


def get_cat_lowest_function_service():
    print("Waiting for cat_lowest_function service...")
    rospy.wait_for_service('catenary_lowest_point')
    try:
        catenary_lowest = rospy.ServiceProxy(
            'catenary_lowest_point', CatLowestPoint)

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

    return catenary_lowest


def load_saved_path(filename='path.txt'):
    try:
        path_file = pkg_drones_rope_planning_path + '/resources/paths/{}'.format(filename)
        print("Loading path from file:", path_file)
        data = np.loadtxt(path_file)
    except Exception as e:
        try:
            # get the file path of droen_path_planning package
            path_file = pkg_drone_path_planning_path + '/resources/paths/{}'.format(filename)
            print("Loading path from file:", path_file)
            data = np.loadtxt(path_file)
        except Exception as e:
            print("No path file found")
            exit(0)
    return data


def get_start_goal_poses(start, goal):
    start_pose = PoseStamped()
    start_pose.pose.position.x, start_pose.pose.position.y, start_pose.pose.position.z = start['x'], start['y'], start['z']
    q = tf.quaternion_from_euler(0, 0, 0)
    start_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    goal_pose = PoseStamped()
    goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z = goal['x'], goal['y'], goal['z']
    q = tf.quaternion_from_euler(0, 0, 0)
    goal_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])

    return start_pose, goal_pose


def get_planner_from_parameters(start: list = None):
    (calc_new_path, rope_length, env_mesh, use_mesh_improvement, use_dynamic_goal, optimal_objective,
     val_check_resolution, safety_distances, start_from_yaml, goal, bounds, planner_algorithm, timeout) = load_parameters(use_parameters_from_ros)

    # robot marker initialization
    robot_mesh = "robot-scene-triangle"
    mesh = "package://drone_path_planning/resources/collada/{}.dae".format(robot_mesh)
    rb = MeshMarker(id=0, mesh_path=mesh)
    robPub = rospy.Publisher('rb_robot',  Marker, queue_size=10)

    # Environment marker initialization
    mesh = "package://drone_path_planning/resources/collada/{}.dae".format(
        env_mesh)
    env = MeshMarker(id=1, mesh_path=mesh)
    env.color.r, env.color.g, env.color.b = 1, 0, 0
    env.updatePose([0, 0, 0], [0, 0, 0, 1])
    envPub = rospy.Publisher('rb_environment',  Marker, queue_size=10)

    robot_mesh += ".stl"
    env_mesh += ".stl"

    print("robot_mesh_name:", robot_mesh)
    print("env_mesh_name:", env_mesh)

    if not calc_new_path:
        print("Using saved path...")
        solved = True
        return solved, rb, robPub, env, envPub

    planner = PlannerSepCollision(env_mesh, robot_mesh, catenaries.lowest_point_optimized, rope_length, use_mesh_improvement=use_mesh_improvement)

    # Set bounds
    planner.set_bounds(bounds)
    planner.createSimpleSetup()

    # Set start and goal
    if start == None:
        start = start_from_yaml
    else:
        start_dict = {}
        start_dict['x'], start_dict['y'], start_dict['z'] = start[0], start[1], start[2]
        start = start_dict

    start_pose, goal_pose = get_start_goal_poses(start, goal)
    planner.set_start_goal(start_pose.pose, goal_pose.pose, use_goal_region=use_dynamic_goal)

    planner.set_planner(eval(planner_algorithm))

    # Set safety distances
    planner.set_safety_distances(safety_distances)

    # Set optimal objective
    si = planner.ss.getSpaceInformation()

    planner.set_optim_objective(optimal_objective)
    planner.set_valid_state_checker(val_checking_resolution=val_check_resolution)

    solved = planner.solve(timeout=timeout)[0]

    return solved, rb, robPub, env, envPub


def load_parameters(use_parameters_from_ros):
    if use_parameters_from_ros == "1":
        print("Using parameters from ROS...")
        # Load parameters from ros parameter server
        calc_new_path = rospy.get_param('planning/calculate_new_path')
        rope_length = rospy.get_param('planning/rope_length')
        env_mesh = rospy.get_param('planning/env_mesh')
        use_mesh_improvement = rospy.get_param('planning/use_mesh_improvement')
        use_dynamic_goal = rospy.get_param('planning/use_dynamic_goal')
        optimal_objective = rospy.get_param('planning/optimal_objective')
        val_check_resolution = rospy.get_param('planning/val_check_resolution')
        # safety distances
        safety_distances = rospy.get_param('planning/safety_distances')

        start = rospy.get_param('planning/start')
        goal = rospy.get_param('planning/goal')

        bounds = rospy.get_param('planning/bounds')

        planner_algorithm = rospy.get_param('planning/planner_algorithm')
        timeout = rospy.get_param('planning/timeout')
    else:
        print("Using parameters from file...")
        file_name = "/home/marios/thesis_ws/src/drone_path_planning/config/prob_definitiion.yaml"
        # load parameters from yaml file
        with open(file_name, 'r') as stream:
            try:
                parameters = yaml.load(stream, Loader=yaml.FullLoader)
            except yaml.YAMLError as exc:
                rospy.logerr(exc)
                exit(0)
        calc_new_path = parameters['calculate_new_path']
        rope_length = parameters['rope_length']
        env_mesh = parameters['env_mesh']
        use_mesh_improvement = parameters['use_mesh_improvement']
        use_dynamic_goal = parameters['use_dynamic_goal']
        optimal_objective = parameters['optimal_objective']
        val_check_resolution = parameters['val_check_resolution']
        # safety distances
        safety_distances = parameters['safety_distances']

        start = parameters['start']
        goal = parameters['goal']

        bounds = parameters['bounds']

        planner_algorithm = parameters['planner_algorithm']
        timeout = parameters['timeout']

    return calc_new_path, rope_length, env_mesh, use_mesh_improvement, use_dynamic_goal, optimal_objective, val_check_resolution, \
        safety_distances, start, goal, bounds, planner_algorithm, timeout


def load_meshes(robot_mesh, env_mesh):
    # robot marker initialization
    mesh = "package://drones_rope_planning/resources/collada/{}.dae".format(robot_mesh)
    rb = MeshMarker(id=0, mesh_path=mesh)
    robPub = rospy.Publisher('rb_robot',  Marker, queue_size=10)

    # Environment marker initialization
    mesh = "package://drones_rope_planning/resources/collada/{}.dae".format(env_mesh)
    env = MeshMarker(id=1, mesh_path=mesh)
    env.color.r, env.color.g, env.color.b = 1, 0, 0
    env.updatePose([0, 0, 0], [0, 0, 0, 1])
    envPub = rospy.Publisher('rb_environment',  Marker, queue_size=10)

    robot_mesh += ".stl"
    env_mesh += ".stl"

    print("robot_mesh_name:", robot_mesh)
    print("env_mesh_name:", env_mesh)

    return rb, robPub, env, envPub


def main():
    # 0: use parameters from file, 1: use parameters from ros
    finished_planning_pub = rospy.Publisher("/finished_planning", String, queue_size=10)

    robot_mesh = "custom_triangle_robot"
    # load ros parameter

    env_mesh = rospy.get_param("/planning/env_mesh")
    rb, robPub, env, envPub = load_meshes(robot_mesh, env_mesh)

    data = load_saved_path(filename='path.txt')

    # generate dynamic path msg
    # path = getPath(data)
    dynamic_path = generate_dynamic_path_msg(data)
    print("Loaded and generated dynamic path")

    trajPub = rospy.Publisher('rigiBodyPath',  Path, queue_size=10)

    trajPub.publish(dynamic_path.Path)

    dynamic_path_pub = rospy.Publisher('dynamicRigiBodyPath', rigid_body_dynamic_path, queue_size=10)

    print("Waiting for connections to the  /dynamicRigiBodyPath topic...")
    while dynamic_path_pub.get_num_connections() == 0:
        if rospy.is_shutdown():
            sys.exit()

    print("Publishing dynamic path...")
    dynamic_path_pub.publish(dynamic_path)
    print("Published dynamic path!")

    # transform
    br = tf.TransformBroadcaster()

    i = 0
    rate = rospy.Rate(10.0)  # hz
    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        br.sendTransform((0, 0, 0), tf.transformations.quaternion_from_euler(-math.pi/2, 0, 0), rospy.Time.now(),
                         "world", "ompl")

        if i == data.shape[0]-1:
            i = 0
        else:
            i += 1

        rb.updatePose(dynamic_path.Path.poses[i].pose.position,
                      dynamic_path.Path.poses[i].pose.orientation, frame="world")

        robPub.publish(rb)

        pos = (rb.pose.position.x, rb.pose.position.y, rb.pose.position.z)
        orientation = (rb.pose.orientation.x, rb.pose.orientation.y,
                       rb.pose.orientation.z, rb.pose.orientation.w)

        br.sendTransform(pos, orientation, rospy.Time.now(),
                         "rigid_body", "world")

        envPub.publish(env)
        # trajPub.publish(path)
        trajPub.publish(dynamic_path.Path)

        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("rb_path_planning")
    main()
