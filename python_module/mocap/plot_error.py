import numpy
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import MocapCommon
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


# ChestFront        0
# ChestBack         1
# SternumFront      2
# SternumBack       3
# rShoulderFront    4
# rShoulderBack     5
# rWristOuter       6
# rWristInner       7
# rPalm             8

NB_HUMAN    = 2
NB_MARKERS  = MocapCommon.get_nb_markers(True, True)


# drops = numpy.genfromtxt('/home/rafi/logging_nine/2/[5900-9000]markers_dropped.csv', delimiter=',')
drops = numpy.genfromtxt('/home/rafi/Desktop/trials/4/[0489-0589]markers_dropped.csv', delimiter=',')
humans = []


for h in range(NB_HUMAN):
    markers = []
    for m in range(NB_MARKERS):
        # Selects each row from the human
        markers.append( drops[:, h*NB_MARKERS+m] )

    humans.append(markers)

# Humans is an array of array.  Each one is a human from the trial
# Each subarray contains an array for each marker
# Each of these marker arrays contains the drops for the entire run.

# drop_max = []

# for config in drops:
#     drop_max.append(numpy.amax(config[:8]))

ChestFront = drops[:,0]
ChestBack = drops[:,1]
SternumFront = drops[:,2]
SternumBack = drops[:,3]
rShoulderFront = drops[:,4]
rShoulderBack = drops[:,5]
rWristOuter = drops[:,6]
rWristInner = drops[:,7]
rPalm = drops[:,8]


# Huge subplot one for each joint
fig = plt.figure()
for m in range(NB_MARKERS):
    ax = fig.add_subplot(NB_MARKERS, 1, m)
    ax.tick_params(axis='y', labelsize=6)
    for h in range(NB_HUMAN):
        # Hack for color.  Should change later
        if h < 1:
            ax.plot(humans[h][m], 'r', label="Active")
        else:
            ax.plot(humans[h][m], 'g', label="Passive")


# # fig.ylim([0, numpy.amax(drops) ])
# ax = fig.add_subplot('911')
# # main = fig.add_axes([0.1, 0.1, 0.8, 0.8])
# ax.plot(ChestFront, 'r', label="ChestFront")
# ax.set_ylim([0, numpy.amax(drops) ])

# ax = fig.add_subplot('912')
# ax.plot(ChestBack, 'r', label="ChestBack")
# ax.set_ylim([0, numpy.amax(drops) ])

# ax = fig.add_subplot('913')
# ax.plot(SternumFront, 'r', label="SternumFront")
# ax.set_ylim([0, numpy.amax(drops) ])

# ax = fig.add_subplot('914')
# ax.plot(SternumBack, 'r', label="SternumBack")
# ax.set_ylim([0, numpy.amax(drops) ])

# ax = fig.add_subplot('915')
# ax.plot(rShoulderFront, 'r', label="rShoulderFront")
# ax.set_ylim([0, numpy.amax(drops) ])

# ax = fig.add_subplot('916')
# ax.plot(rShoulderBack, 'r', label="rShoulderBack")
# ax.set_ylim([0, numpy.amax(drops) ])

# ax = fig.add_subplot('917')
# ax.plot(rWristOuter, 'r', label="rWristOuter")
# ax.set_ylim([0, numpy.amax(drops) ])

# ax = fig.add_subplot('918')
# ax.plot(rWristInner, 'r', label="rWristInner")
# ax.set_ylim([0, numpy.amax(drops) ])

# ax = fig.add_subplot('919')
# ax.plot(rPalm, 'r', label="rPalm")
# ax.set_ylim([0, numpy.amax(drops) ])
# # main.set_title("Graph of Curviture and Speed vs Time")
# # main.legend(loc=2);

fig.tight_layout()
fig.savefig('/home/rafi/Desktop/[1188-1256]per_joint.png', bbox_inches='tight')


fig = plt.figure()

for h in range(NB_HUMAN):
    ax = fig.add_subplot(NB_HUMAN,1,h)
    start = h*NB_MARKERS
    end = NB_MARKERS+h*NB_MARKERS

    drop_max = []
    for config in drops[:,start:end]:
        print config
        drop_max.append(numpy.amax(config))



    if h < 1:
        ax.plot(drop_max, 'r', label="Active")
    else:
        ax.plot(drop_max, 'g', label="Passive")

fig.savefig('/home/rafi/Desktop/[1188-1256]max.png', bbox_inches='tight')





# main = fig.add_axes([0.1, 0.1, 0.8, 0.8])
# main.plot(drop_max, 'r', label="Drop Max")
# fig.savefig('/home/rafi/Desktop/out1.png', bbox_inches='tight')


# curv = numpy.genfromtxt('/home/rafi/workspace/move3d/move3d-launch/matlab/quan_motion/hrics_feature_curv_'+str(i)+'.csv', delimiter=',')

# fig = plt.figure()
# main = fig.add_axes([0.1, 0.1, 0.8, 0.8])
# main.plot(curv[0], curv[1], 'r', label="curv")
# main.plot(speed[0],speed[1], 'b', label="speed")
# plt.axvline(curv[0,replans[i]], color='g', label="replan")
# main.set_title("Graph of Curviture and Speed vs Time")
# main.legend(loc=2);
# fig.savefig('/home/rafi/workspace/move3d/move3d-launch/matlab/quan_motion/curv_speed_'+str(i)+'.png', bbox_inches='tight')

# pos = numpy.genfromtxt('/home/rafi/workspace/move3d/move3d-launch/matlab/quan_motion/hrics_feature_pos_'+str(i)+'.csv', delimiter=',')
# fig = plt.figure()
# ax = Axes3D(fig)
# ax.plot(pos[1], pos[2], pos[3])
# ax.scatter(pos[1][replans[i]], pos[2][replans[i]], pos[3][replans[i]], color='red')
# fig.savefig('/home/rafi/workspace/move3d/move3d-launch/matlab/quan_motion/pos_'+str(i)+'.png', bbox_inches='tight')

# # Pos









#dist = numpy.genfromtxt('/home/rafi/workspace/move3d/move3d-launch/matlab/quan_motion/hrics_feature_dist_0.csv', delimiter=',')