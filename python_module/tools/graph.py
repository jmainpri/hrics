import numpy
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import sys



# speed = numpy.genfromtxt('/home/rafi/workspace/move3d/move3d-launch/matlab/quan_motion/hrics_feature_speed_0.csv', delimiter=',')
#
# fig = plt.figure()
# main = fig.add_axes([0.1, 0.1, 0.8, 0.8])
#
# main.plot(speed[0], speed[1], 'r')
# main.set_xlabel('Time')
# main.set_ylabel('Speed')
# main.set_title("Graph of Speed vs Time")
#
# fig.savefig('/home/rafi/workspace/move3d/move3d-launch/matlab/quan_motion/hrics_feature_speed_0.png', bbox_inches='tight')
#
# ################################################################33
#
# curv = numpy.genfromtxt('/home/rafi/workspace/move3d/move3d-launch/matlab/quan_motion/hrics_feature_curv_0.csv', delimiter=',')
#
# fig = plt.figure()
# main = fig.add_axes([0.1, 0.1, 0.8, 0.8])
#
# main.plot(curv[0], curv[1], 'r')
# main.set_xlabel('Time')
# main.set_ylabel('Curviture')
# main.set_title("Graph of Curviture vs Time")
#
#
# fig.savefig('/home/rafi/workspace/move3d/move3d-launch/matlab/quan_motion/hrics_feature_curv_0.png', bbox_inches='tight')

################################################################33
# fig = plt.figure()
# main = fig.add_axes([0.1, 0.1, 0.8, 0.8])
# main.plot(curv[0], curv[1], 'r', label="curv")
# main.plot(speed[0],speed[1], 'b', label="speed")
# plt.axvline(curv[0,10])
# main.set_title("Graph of Curviture and Speed vs Time")
# main.legend(loc=2);
# fig.savefig('/home/rafi/workspace/move3d/move3d-launch/matlab/quan_motion/curv_speed.png', bbox_inches='tight')


# Curv vs speed
replans = [ 18, 16, 13, 10, 20, 30, 10, 27, 24, 31, 18, 20, 28, 44, 6, 34, 26, 3, 11 ]

for i in range(18):
    speed = numpy.genfromtxt('/home/rafi/workspace/move3d/move3d-launch/matlab/quan_motion/hrics_feature_speed_'+str(i)+'.csv', delimiter=',')
    curv = numpy.genfromtxt('/home/rafi/workspace/move3d/move3d-launch/matlab/quan_motion/hrics_feature_curv_'+str(i)+'.csv', delimiter=',')

    fig = plt.figure()
    main = fig.add_axes([0.1, 0.1, 0.8, 0.8])
    main.plot(curv[0], curv[1], 'r', label="curv")
    main.plot(speed[0],speed[1], 'b', label="speed")
    plt.axvline(curv[0,replans[i]], color='g', label="replan")
    main.set_title("Graph of Curviture and Speed vs Time")
    main.legend(loc=2);
    fig.savefig('/home/rafi/workspace/move3d/move3d-launch/matlab/quan_motion/curv_speed_'+str(i)+'.png', bbox_inches='tight')

    pos = numpy.genfromtxt('/home/rafi/workspace/move3d/move3d-launch/matlab/quan_motion/hrics_feature_pos_'+str(i)+'.csv', delimiter=',')
    fig = plt.figure()
    ax = Axes3D(fig)
    ax.plot(pos[1], pos[2], pos[3])
    ax.scatter(pos[1][replans[i]], pos[2][replans[i]], pos[3][replans[i]], color='red')
    fig.savefig('/home/rafi/workspace/move3d/move3d-launch/matlab/quan_motion/pos_'+str(i)+'.png', bbox_inches='tight')

# Pos









#dist = numpy.genfromtxt('/home/rafi/workspace/move3d/move3d-launch/matlab/quan_motion/hrics_feature_dist_0.csv', delimiter=',')



















# 0 , w : 1.00 , distance name : rShoulderX , rShoulderX
# 1 , w : 1.00 , distance name : rShoulderX , rElbowZ
# 2 , w : 1.00 , distance name : rShoulderX , rWristX
# 3 , w : 1.00 , distance name : rElbowZ , rShoulderX
# 4 , w : 1.00 , distance name : rElbowZ , rElbowZ
# 5 , w : 1.00 , distance name : rElbowZ , rWristX
# 6 , w : 1.00 , distance name : rWristX , rShoulderX
# 7 , w : 1.00 , distance name : rWristX , rElbowZ
# 8 , w : 1.00 , distance name : rWristX , rWristX
