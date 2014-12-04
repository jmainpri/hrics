import sys
import mocap.MocapCommon as MocapCommon
import numpy as np
import rospy
import cv2
from os import listdir
from os.path import isfile, join
import os
import matplotlib.pyplot as plt
from itertools import product

# Returns a dictionary of humans.  Keys are integers 0,1,...N for N Human
# Each human is a dictionary of markers.  Keys are ChestFront Chestback etc
# The value for a marker is its entire x,y,z matrix
# Each human additionaly has a key for time, which is the time matrix in [[Sec, Nsec], ...]
def get_human_dict(filename):

    # Load matrix into human dictionary
    raw = np.genfromtxt(m_file, delimiter=',', dtype='string')

    # Dictionary of human number to
    humans = {}
    # Get time matrix
    time = raw[:,:2].astype(np.float).T
    # Remove time exta columns from matrix
    raw = raw[:,3:]

    # Get # per human
    # Iterate in groups of 4, for # per human * 4

    # [0, 4, 8 ...., nbper*4], [h*nbper+m]
    for h in range(NB_HUMAN):
        # markers = []
        cells = {}
        print 'Human : ', h
        for m in range(0,NB_MARKERS*4,4):
            idx = (NB_MARKERS*4*h)+m
            marker_name = raw[0][idx]
            cells[marker_name] = raw[:,idx+1:idx+4].astype(np.float).T

        cells['time'] = time
        humans[h] = cells

    return humans

def get_dt(time):
    ret = np.zeros(len(time))
    time_sec = time[0] + (time[1]*10**-9)
    dt = np.diff(time_sec)

    # Pad with inf
    dt = np.pad(dt, (1,0), 'constant', constant_values=(np.inf, np.inf))
    return dt

def get_velocity(dt, pos, window=[0., 0., -2/6., -3/6., 6/6., -1/6., 0.]):
    ret = np.ndarray(pos.shape)

    for i,row in enumerate(pos):
        ret[i] = np.convolve(row, window, 'same')
        ret[i] /= dt**1

    return ret

def get_accel(dt, pos, window=[0, -1/12.0, 16/12.0, -30/12.0, 16/12.0, -1/12.0, 0]):
    ret = np.ndarray(pos.shape)

    for i,row in enumerate(pos):
        ret[i] = np.convolve(row, window, 'same')
        ret[i] /= dt**2

    return ret


# def get_curv(dt, vel):
#     Eigen::Vector3d v1 = vel_buff[i][j].normalized();
#     Eigen::Vector3d v2 = vel_buff[i-1][j].normalized();

#     double angle = atan2( v1.cross(v2).norm() , v1.dot(v2) ) ;
#     curviture[i][j] = angle;



if __name__ == '__main__':
    base_dir = '/home/rafi/aterm_experiment/'

    # Read set up to constants
    setup = MocapCommon.read_setup(base_dir)
    NB_HUMAN    = setup[0]
    ELBOW_PADS  = setup[1]
    RARM_ONLY   = setup[2]
    NB_MARKERS  = MocapCommon.get_nb_markers(ELBOW_PADS, RARM_ONLY)
    print 'num human', NB_HUMAN
    print 'num markers', NB_MARKERS


    blocks = sorted([ name for name in os.listdir(base_dir) if os.path.isdir(os.path.join(base_dir, name)) ])
    for block in blocks:
        path = os.path.join(base_dir, block)
        runs = sorted([ name for name in os.listdir(path) if os.path.isdir(os.path.join(path, name)) ])

        for run in runs:
            file_path = os.path.join(path, run)
            out_path = os.path.join('/home/rafi/Desktop/feature_plots',block,run)
            print
            print 'Calculating features for block : ', block, ' run : ', run
            print 'in folder : ', file_path
            print'================================================'
            print 'Outpath : ', out_path

            # Make output directory
            # if not os.path.exists(out_path):
            #     os.makedirs(out_path)


            # Only consider runs that have splits in them.
            if os.path.isfile(os.path.join(file_path, 'Splits.csv')) :
                splits = MocapCommon.read_splits(file_path+'/')
                m_file = os.path.join(file_path,'markers_fixed.csv')
                o_file = os.path.join(file_path,'objects_fixed.csv')

                humans = get_human_dict(m_file)

                dt = get_dt(humans[0]['time'])
                vel = get_velocity(dt, humans[0]['rPalm'])
                squared_vel = [np.linalg.norm(x) for x in vel.T]
                squared_accel = [np.linalg.norm(x) for x in get_accel(dt, humans[0]['rPalm']).T]

                print humans[0]['rPalm'].T[0][1]
                print humans[1]['rPalm'].T[0][1]

                for split in splits:

                    fig = plt.figure()
                    main = fig.add_axes([0.1, 0.1, 0.8, 0.8])

                    main.plot(range(split[0],split[1]), squared_vel[split[0]:split[1]], label=r'Velocity')
                    main.plot(range(split[0],split[1]), squared_accel[split[0]:split[1]], label=r'Accel')
                    main.set_xlabel('time')
                    main.set_ylabel('velocity')
                    main.legend(loc ='upper left')
                    # plt.ylim(0,1)
                    plt.show()

                # print humans[0]['time'][0]

            # sys.exit()



# Write setup to mapping



