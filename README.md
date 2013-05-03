hrics-or-plugins
================

hrics-or-plugins is a Human Robot Interaction Cost Space (HRICS) plugin suite for Openrave

== How to run the kinect update ==

To use the get the kinect data you need to install ros (fuerte) though
this might work in groovy (not tested).

=== Install and launch openni_tracker ===

First your need to install the ROS package named openni_tracker in fuerte

    http://ros.org/wiki/openni_tracker
  
On ubuntu this can be performed by running the following command :
    
    sudo apt-get install ros-fuerte-openni-tacker
    
Make sure that it works by running :
    
    roslaunch openni_tacker openni.launch
    
And then:

    rosrun openni_tacker openni_tacker
    
In case your not able to make that part work there is a package relying on the 
new version of NiTE in the repository.

=== Install and launch the openrave plugin ===

This plugin relies on the 0.8.2 version of OR.

To start you need add the openrave plugin path in your environment,
Modify the following line, for bash, with right path:
    
    export OPENRAVE_PLUGINS=/home/jmainpri/workspace/hrics-openrave-plugin/plugins:$OPENRAVE_PLUGINS
    
Then you can run the python example in the example folder:
    
    cd examples/
    python examples
    
If the follwing line is commented

    orEnv.Load("../ormodels/human_wpi.xml")
    
The human will not be loaded and openrave will draw the kinect raw data
with spheres and line segment
