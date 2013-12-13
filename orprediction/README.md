OR PREDICTION
==========================
##How to run Prediction Node:

###First run the skeleton tracker:

    rosrun openni_tracker openni_tracker

###Run ORKinect in background mode with namespace human:
A python script named hiddenSkeletonTracker.py is in the examples folder.

    export ROS_NAMESPACE = human
    python hiddenSkeletonTracker.py
    
###Launch the Human URDF

    roslaunch human_urdf human_urdf.launch
    
###Launch the base pointing node:
Again a script named base_pointing_pub.py can be found in the examples folder

    python base_pointing_pub.py
    
###Launch the prediction node:
The prediction launch script can also be found in the examples folder

    python prediction.py
    
##Description of pipeline operation

###Openni Tracker

The openni tracker takes kinect data and publishes joint values to /tf

###ORKinect

ORKinect subscribes to the tf frames published by Openni Tracker and publishes joint states values on the topic /human_state

###ORPrediction

The prediction node subscribes to the human state topic published by ORKinect, does some calculations and publishes motion class markers on the topic class_markers

###base_pointing_pub

The base pointing publisher is required at all times.  Its use is to publish the angle between the PR2s base frame and head frame

###Human URDF

The human urdf launch file subscribes to the ORKinect joint states and publishes joint states on the topic /human/joint_states

##Configuring RViZ

    Set fixed frame to /base_footprint
    Add a RobotModel described by the topic /human/robot_description
    Add a MarkerArray on the topic /human/class_markers
