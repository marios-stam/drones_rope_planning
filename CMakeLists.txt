cmake_minimum_required(VERSION 3.0.2)

project(drones_rope_planning)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    roscpp
    message_filters
    sensor_msgs
    std_msgs
    tf
    tf_conversions
    eigen_conversions
#    moveit_ros_planning_interface
#    moveit_ros_move_group
    std_srvs
    message_generation
    ompl
    octomap_msgs
    
    octomap_ros
    pcl_conversions
    pcl_ros
    roscpp
    std_msgs
    nav_msgs
    execution
    
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system serialization)
find_package(Eigen3 REQUIRED)

find_package(ompl)
find_package(octomap REQUIRED)
find_package(fcl REQUIRED)

##===========MESSAGE AND SERVICE GENERATION===========
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  CylinderObstacleData.msg
  rigid_body_dynamic_path.msg
  #Message2.msg
)
## Generate services in the 'srv' folder
add_service_files(
   FILES
   PlanningRequest.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
)
##====================================================

# generate_messages(DEPENDENCIES std_msgs sensor_msgs) 
###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
    INCLUDE_DIRS
      include
    LIBRARIES
      ${PROJECT_NAME}_library
    CATKIN_DEPENDS
	    roscpp
	    message_filters
            sensor_msgs
	    std_msgs
            geometry_msgs
	    tf
	    tf_conversions
	    eigen_conversions
#            moveit_ros_planning_interface
#            moveit_ros_move_group
	    trajectory_msgs 
      octomap_msgs 
      octomap_ros 
      pcl_conversions 
      pcl_ros roscpp 
      message_runtime
#  DEPENDS system_lib
)

install(PROGRAMS
  scripts/path_vis.py
  scripts/path_vis_dynamic.py
  scripts/realtime_obstacles_viz.py
  scripts/realtime_interface.py
  scripts/moving_obstacles_pub.py
  scripts/path_vis_dynamic_realtime.py


  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${CMAKE_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  
  ${PCL_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
  ${cl_INCLUDE_DIRS}

  #marios
  include/traj_generation
)

# link_libraries(${OCTOMAP_LIBRARIES} ${fcl_LIBRARY_DIRS} ${OMPL_LIBRARY_DIRS})

## Declare a cpp library
file(GLOB ompl_example_2d_sources
    src/ompl_example_2d.cpp
    src/fcl_mesh.cpp
    src/stlloader.cpp

    src/catenaries/catenary.cpp
    src/catenaries/math_utils.cpp

    src/custom_mesh.cpp
    src/custom_mesh_robust.cpp

    src/problem_params.cpp
    src/optim_objectives.cpp
    src/custom_objectives.cpp
    
    src/moving_obstacles.cpp
    
    #fcl_checkers
    src/fcl_checker_base.cpp
    src/fcl_checker_realtime.cpp
    src/fcl_checker_offline.cpp
)

file(GLOB ompl_example_2d_headers
    include/ompl_example_2d/ompl_example_2d.hpp
    include/ompl_example_2d/fcl_mesh.hpp
    
    include/ompl_example_2d/stlloader.hpp
    include/ompl_example_2d/problem_params.hpp

    include/catenaries/catenary.hpp
    include/catenaries/math_utils.hpp
    
    include/custom_mesh.hpp
    include/custom_mesh_robust.hpp

    include/ompl_example_2d/custom_goals.hpp
    include/ompl_example_2d/optim_objectives.hpp

    include/ompl_example_2d/custom_objectives.hpp
    
    include/moving_obstacles.hpp

    #fcl_checkers
    include/fcl_checker_base.hpp
    include/fcl_checker_realtime.hpp
    include/fcl_checker_offline.hpp


) 




add_library(${PROJECT_NAME}_library 
  STATIC 
  ${ompl_example_2d_sources} ${ompl_example_2d_headers}

)

target_link_libraries(${PROJECT_NAME}_library
   ${catkin_LIBRARIES}
   ${catkin_LIBRARIES}
   ompl
   boost_serialization
   ${PCL_LIBRARIES}
   ${OCTOMAP_LIBRARIES}
   ${fcl_LIBRARIES}
  #  fcl
)
 
# add_dependencies(${PROJECT_NAME}_library ompl_example_2d_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

## Declare a cpp executable
add_executable(
  planning_realtime_node
  src/planning_realtime_node.cpp
)

add_executable(
  planning_offline_node
  src/planning_offline_node.cpp
)


add_executable(
  traj_generation_node 
  src/traj_generation/example1_node.cpp
  )

  target_link_libraries(traj_generation_node
  ${catkin_LIBRARIES}
)

add_executable(
  jerk_traj_generation_node 
  src/traj_generation/min_jerk_traj_generator.cpp
  )

  target_link_libraries(jerk_traj_generation_node
  ${catkin_LIBRARIES}
)


add_dependencies(planning_realtime_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(planning_realtime_node
   ${PROJECT_NAME}_library
  ${catkin_LIBRARIES}
  ompl
  boost_serialization
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${fcl_LIBRARIES}
  fcl
  # try to comment line above and uncomment line below if problems with linking to ompl lcmkibrary
#   ${OMPL_LIBRARIES}
)

add_dependencies(planning_offline_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(planning_offline_node
   ${PROJECT_NAME}_library
  ${catkin_LIBRARIES}
  ompl
  boost_serialization
  ${PCL_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
  ${fcl_LIBRARIES}
  fcl
)


# target_link_libraries(${PROJECT_NAME}_fcl_mesh
#    ${PROJECT_NAME}_library
#   ${catkin_LIBRARIES}
#   boost_serialization
#   ${PCL_LIBRARIES}
#   ${OCTOMAP_LIBRARIES}
#   ${fcl_LIBRARIES}
#   fcl
#   # try to comment line above and uncomment line below if problems with linking to ompl library
# #   ${OMPL_LIBRARIES}
# )

# target_link_libraries(${PROJECT_NAME}_fcl_checker
#    ${PROJECT_NAME}_library
#   ${catkin_LIBRARIES}
#   boost_serialization
#   ${PCL_LIBRARIES}
#   ${OCTOMAP_LIBRARIES}
#   ${fcl_LIBRARIES}
#   fcl
# )

## Declare a cpp executable
# add_executable(
#   ${PROJECT_NAME}_trajectory_tending
#   src/trajectoryNodeTending.cpp
# )

# add_dependencies(${PROJECT_NAME}_trajectory ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(planning_realtime_node ${catkin_EXPORTED_TARGETS})
add_dependencies(planning_offline_node ${catkin_EXPORTED_TARGETS})


#############
## Testing ##
#############

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
# Add gtest based cpp test target and link libraries
#catkin_add_gtest(${PROJECT_NAME}-test
#  test/test_elevation_mapping.cpp
#  test/ElevationMapTest.cpp
#  test/WeightedEmpiricalCumulativeDistributionFunctionTest.cpp
#)
#if(TARGET ${PROJECT_NAME}-test)
#  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME}_library)
#endif()
