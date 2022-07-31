#!/usr/bin/env python
from cProfile import label
from turtle import color
from unicodedata import name
from matplotlib import offsetbox
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


"""
etting new start
New start valid: 0
Nearest obstacle id: 0
obs_vel=-0.237107, 0.750488, 0.803546
Obstacle with id:0cyl_rotation=0.350025, 0.021582, 0.577798, 0.736998
cyl_axis=0.436299, -0.490996, 0.754034
Robot_start: -0.107240 2.873296 0.500000
Robot_goal: 0.000000 6.000000 0.750000
Obstacle pos: -0.830282 3.190341 0.567464
Dstart: -0.172000 -0.094804 0.037791
"""

"""
Setting new start
New start valid: 0
Nearest obstacle id: 0
obs_vel=-0.532222, 1.248130, -0.019457
Obstacle with id:0cyl_rotation=-0.362973, -0.559158, 0.499291, 0.553445
cyl_axis=-0.981385, -0.156594, 0.111186
Robot_start: 0.350823 5.203044 1.260305
Robot_goal: 0.000000 2.000000 0.750000
Obstacle pos: -0.530213 4.861351 0.167219
Dstart: -0.020602 -0.011881 -0.198581
"""

# 27-05-2022
"""
obs_vel=-0.001620, 0.001751, -0.001402
Obstacle with id:0cyl_rotation=-0.036006, 0.923107, 0.382847, 0.002467
cyl_axis=-0.023014, 0.706995, -0.706844
Robot_start: -1.048370 3.695504 1.196604
Robot_goal: 0.000000 6.000000 1.200000
Obstacle pos: 0.074282 4.393348 1.626196
Dstart: 0.030995 0.140202 0.139222
"""
# obs_pos = [-0.830282, 3.190341, 0.567464]
# cyl_axis = [0.436299, -0.490996, 0.754034]
# obs_vel = [-0.237107, 0.750488, 0.803546]
# robot_start = [-0.107240, 2.873296, 0.500000]
# robot_goal = [0.000000, 6.000000, 0.750000]
# dstart = [-0.172000, -0.094804, 0.037791]

# obs_pos = [-0.530213, 4.861351, 0.167219]
# cyl_axis = [-0.981385, -0.156594, 0.111186]
# obs_vel = [-0.532222, 1.248130, -0.019457]
# robot_start = [0.350823, 5.203044, 1.260305]
# robot_goal = [0.000000, 2.000000, 0.750000]
# dstart = [-0.020602 - 0.011881 - 0.198581]

# Marios Test
# obs_pos = [0.2, 4, 1]
# cyl_axis = [-1, 0, 0]
# obs_vel = [0, 0.01, 1]
# robot_start = [0, 3, 1]
# robot_goal = [0, 6.000000, 1]
# get cross product of cyl_Axis and obs_vel
# dstart = np.cross(cyl_axis, obs_vel)
# dstart *= -1  # it meakes sense like this so NEED TO CHANGE IN REAL ONE

obs_pos = [0.074282, 4.393348, 1.626196]
cyl_axis = [-0.023014, 0.706995, -0.706844]
obs_vel = [-0.001620, 0.001751, -0.001402]
robot_start = [-1.048370, 3.695504, 1.196604]
robot_goal = [0.000000, 6.000000, 1.200000]
dstart = [0.030995, 0.140202, 0.139222]

# ===============================================================================================
soa = np.array([
    [obs_pos[0], obs_pos[1], obs_pos[2], cyl_axis[0], cyl_axis[1], cyl_axis[2]]
])

X, Y, Z, U, V, W = zip(*soa)
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
# plot vector with name
ax.quiver(X, Y, Z, U, V, W, color="r")

vec = [obs_pos[0], obs_pos[1], obs_pos[2], obs_vel[0], obs_vel[1], obs_vel[2]]
ax.quiver(vec[0], vec[1], vec[2], vec[3], vec[4], vec[5], color="b")


# plot a point
ax.scatter(robot_start[0], robot_start[1], robot_start[2], color="g")
ax.scatter(robot_goal[0], robot_goal[1], robot_goal[2], color="r")

dstart = [robot_start[0], robot_start[1], robot_start[2], dstart[0], dstart[1], dstart[2]]
# ax.quiver(dstart[0], dstart[1], dstart[2], dstart[3], dstart[4], dstart[5], color="g")
ax.quiver(obs_pos[0], obs_pos[1], obs_pos[2], dstart[3], dstart[4], dstart[5], color="g")

# get the plane
point = np.array([obs_pos[0], obs_pos[1], obs_pos[2]])
normal = np.array([dstart[3], dstart[4], dstart[5]])

# a plane is a*x+b*y+c*z+d=0
# [a,b,c] is the normal. Thus, we have to calculate
# d and we're set
d = -point.dot(normal)

# create x,y
xx, yy = np.meshgrid(range(5), range(5))
# add offset to points (turn to numpy arrays)
xx = xx + obs_pos[0]
yy = yy + obs_pos[1]


# calculate corresponding z
z = (-normal[0] * xx - normal[1] * yy - d) * 1. / normal[2]

# plot the surface
# ax.plot_surface(xx, yy, z)

ax.set_xlim([-2, 2])
ax.set_ylim([1, 6])
ax.set_zlim([-0.5, 10])
# show x,y,z axis
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
plt.show()
