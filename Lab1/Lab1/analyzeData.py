import h5py
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# importing data from data files
data_robot = h5py.File("data_robotTracks.h5", "r")
robot_time = data_robot['time'] # is time the title?
robot_translation = data_robot['t_vecs']
robot_rotation = data_robot['r_vecs']

data_fly = h5py.File("flyTracksTrial2.h5","r")
fly_time = data_fly['time']
fly_x = data_fly['pos_x']
fly_y = data_fly['pos_y']
fly_angle = data_fly['angle']


fly_time_short = fly_time[:100]
fly_x_short = fly_x[:100]
fly_y_short = fly_y[:100]
print(np.std(fly_x_short))
print(np.std(fly_y_short))
print(np.std(fly_x[10000:]))
print(np.std(fly_y[10000:]))
print(np.std(fly_x))
print(np.std(fly_y))


# extracting rotation and translation components
robot_x = robot_translation[:,0]
robot_y = robot_translation[:,1]
robot_z = robot_translation[:,2]

robot_rx = robot_rotation[:,0]
robot_ry = robot_rotation[:,1]
robot_rz = robot_rotation[:,2]

print(np.std(robot_rx))
print(np.std(robot_ry))

fig2 = plt.figure(2)
# preliminary fly data analysis
ax2 = fig2.add_subplot(111)
ax2.scatter(fly_time, fly_angle)

# example of plotting data
#fig = plt.figure(1)
#ax = fig.add_subplot(111, projection = '3d')
#ax.scatter(robot_rx, robot_ry, robot_rz)


#fig2 = plt.figure(2)
# preliminary fly data analysis
#ax2 = fig2.add_subplot(111)
#ax2.scatter(fly_x, fly_y)

# 2D robot
#fig3 = plt.figure(3)
#ax3 = fig3.add_subplot(111)
#ax3.scatter(robot_rx, robot_ry)

# histogram of robot position
#fig4 = plt.figure(4)
#ax4 = fig4.add_subplot(111)
#ax4.hist2d(robot_rx, robot_ry)

# robot x position vs time
#fig5 = plt.figure(5)
#ax5 = fig5.add_subplot(111)
#ax5.scatter(robot_time, robot_rx)

# robot y position vs time
#fig6 = plt.figure(6)
#ax6 = fig6.add_subplot(111)
#ax6.scatter(robot_time, robot_ry)

# robot position vs time
#fig7 = plt.figure(7)
#ax7 = fig7.add_subplot(111, projection = '3d')
#ax7.scatter(robot_time, robot_rx, robot_ry)

# fly position vs time
fig8 = plt.figure(8)
ax8 = fig8.add_subplot(111)
ax8.scatter(fly_time_short, fly_x_short)

fig9 = plt.figure(9)
ax9 = fig9.add_subplot(111)
ax9.scatter(fly_time_short, fly_y_short)

fig10 = plt.figure(10)
ax10 = fig10.add_subplot(111, projection = '3d')
ax10.scatter(fly_time_short, fly_x_short, fly_y_short)

# fly histogram
fig11 = plt.figure(11)
ax11 = fig11.add_subplot(111)
ax11.hist2d(fly_x, fly_y)




plt.show()
