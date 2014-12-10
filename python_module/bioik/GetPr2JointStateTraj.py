#!/usr/bin/env python
# spins off a thread to listen for joint_states messages
# and provides the same information (or subsets of) as a service

import roslib
# roslib.load_manifest('joint_states_listener')
import rospy
from sensor_msgs.msg import JointState
import threading
import sys
import tf

# holds the latest states obtained from joint_states messages
class LatestJointStates:

    def __init__(self):
        print "init node"
        rospy.init_node('joint_states_listener')
        rospy.on_shutdown(self.on_shutdown)
        
        # joint state
        self.names = []
        self.position = []
        self.velocity = []
        self.effort = []

        # robot pose
        self.robot_pose = None

        # threads
        self.thread_joint_state = None
        self.thread_pose = None

        # trajectory
        self.trajectory = []
        self.start_time = 0.0
        self.prev_time = 0.0
        self.folder = "."

    # thread for tf listener of the robot
    def start_tf_listener(self):
        print "lock thread tf"
        self.lock_tf = threading.Lock()
        self.thread_pose = threading.Thread(target=self.pose_listener)
        self.thread_pose.start()

    # get the transform between the base footprint and odom_combined
    def pose_listener(self):
        print "start listener"
        listener = tf.TransformListener()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():

            try:
                (trans,rot) = listener.lookupTransform('/odom_combined', '/base_footprint', rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            self.lock_tf.acquire()
            self.robot_pose = sum( [list(trans), list(rot)],[])
            self.lock_tf.release()
            # print "pose : " , self.robot_pose
            rate.sleep()
        
    # start the joint state thread
    def start_thread(self):
        print "lock thread joint state"
        self.lock = threading.Lock()
        self.thread_joint_state = threading.Thread(target=self.joint_states_listener)
        self.thread_joint_state.start()
        
    # thread function: listen for joint_states messages
    def joint_states_listener(self):
        rospy.Subscriber('joint_states', JointState, self.joint_states_callback)
        rospy.spin()

    # callback function: when a joint_states message arrives, save the values
    def joint_states_callback(self, msg):
        
        self.lock.acquire()
        self.names = msg.name
        self.position = msg.position
        self.velocity = msg.velocity
        self.effort = msg.effort

        time = msg.header.stamp.to_sec()

        if( len(self.trajectory) == 0 ): # start recording when time 
            if( time >= self.start_time ):
                self.start_time = time
                print "start_time : ", self.start_time
            self.prev_time = self.start_time

            # for i, name in enumerate(self.names):
            #     print "_pr2_map[\"%s\"] = %d;" % ( name , i )
 
        if( time >= self.start_time ):

            if( len(self.trajectory) == 0 ): # start recording when time 
                print "start recording : %.6f" % time

            dt = time-self.prev_time
            self.lock_tf.acquire()
            self.trajectory.append([[dt],self.position,self.robot_pose])
            self.lock_tf.release()
            self.prev_time = time
        
        # print "time : ", time
        # print "positions : ", self.position
        # print self.name
        self.lock.release()

    def on_shutdown(self):

        if len(self.trajectory) > 0 :
            print "Logging stored marker values to disk..."
 
            with open(self.folder + "/joint_state_traj.csv", 'w') as traj_file:
                
#                line_str = ''
#                for name in self.names:
#                    line_str += str(name) + ','
#                line_str = line_str.rstrip(',')
#                line_str += '\n'
#                traj_file.write(line_str)
                print "nb of dofs : ", len(self.names)

                line_str = ''
                for q in self.trajectory:
                    for q_i in q:
                        for q_ii in q_i:
                            line_str += str(q_ii) + ','
                    line_str = line_str.rstrip(',')
                    line_str += '\n'
                traj_file.write(line_str)
            print "...done, exiting"

# run the server
if __name__ == "__main__":

    latestjointstates = LatestJointStates()

    for index in range(1, len(sys.argv)):
        if sys.argv[index] == "-t" and index+1 < len(sys.argv):
            latestjointstates.start_time  = float(sys.argv[index+1])

    latestjointstates.start_tf_listener()
    latestjointstates.start_thread()

    print "LatestJointStates server started, waiting for queries"
    rospy.spin()

