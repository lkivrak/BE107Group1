import h5py
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# importing data from data files
data_robot = h5py.File("data_robotTracks.h5", "r")
robot_time = data_robot['time'] # is time the title?
robot_translation = data_robot['t_vecs']
robot_rotation = data_robot['r_vecs']

data_fly = h5py.File("data_flyTracksv0.h5","r")
fly_time = data_fly['time']
fly_x = data_fly['pos_x']
fly_y = data_fly['pos_y']
fly_angle = data_fly['angle']

# extracting rotation and translation components
robot_x = robot_translation[:,0]
robot_y = robot_translation[:,1]
robot_z = robot_translation[:,2]

robot_rx = robot_rotation[:,0]
robot_ry = robot_rotation[:,1]
robot_rz = robot_rotation[:,2]

# Find fly angle
fly_ang = np.arctan([fly_x,fly_y])

# Find robot angle
robot_ang = robot_rz

fig3 = plt.figure(3)
# preliminary fly data analysis
ax3 = fig3.add_subplot(111)
ax3.scatter(robot_time, robot_ang)


# example of plotting data
fig = plt.figure(1)
ax = fig.add_subplot(111, projection = '3d')
ax.scatter(robot_rx, robot_ry, robot_rz)
ax.scatter(robot_rx, robot_ry)

fig2 = plt.figure(2)
# preliminary fly data analysis
ax2 = fig2.add_subplot(111)
ax2.scatter(fly_x, fly_y)

plt.show()
