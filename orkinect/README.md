OR KINECT
==========================

MODES:
    (a) Recording (Listening)
    (b) Playback


# (a) Recording

### (i) Set options

Before one can begin recording, there are a few settings that must be edited in the class_runSensors.py file.  Namely:

    self.prob.SendCommand('SetCustomTracker 1')

This tells the orkinect plugin which kinect topic to subscribe to. In order to have more than 1 kinect, you must use the multi tracker.

    0 will use openni_tracker
    1 will use the custom openni_tracker_multi
    
    self.prob.SendCommand('SetNumKinect 1') #still need to call as 1 if using default tracker.

Sets the number of kinects to subscribe to.  This needs to be set to 1 even if using the original skeleton tracker

If you would like to use more than 1 kinect, make sure you uncomment the line that loads model human_wpi_blue

    self.prob.SendCommand('EnableCamera 1')
    
This will tell the plugin to subscribe to the camera topic and save images to disk. (defult save location: /home/rafihayne/workspace/statFiles/snapshots)

If enable camera is set to true (1) the camera node must be running, or no recording can be done.

    
### (ii) Launch Ros

Now that we have set the settings, we must run the appropriate nodes.

Run the following commands in individual terminal windows:
Start ROS:

    roscore

Run the skeleton tracker:

    rosrun openni_tracker openni_tracker

OR for two kinects

    rosrun openni_tracker_multi tracker1 (You will be prompted to select a kinect)
    rosrun openni_tracker_multi tracker2

Run the camera node:

    roslaunch openni_launch openni.launch device_id:="#1" camera:=camera0 (This launches the first plugged in kinect, and renames the topics it publishes to camera0.  The plugin subscibes to topic camera0)
    roslaunch openni_launch openni.launch device_id:="#2" camera:=camera1

Run the wiimote node:
If you would like to record with a wiimote you must also run the wiimote node

    rosrun wiimote wiimote_node.py

You will be prompted to pair the wiimote now.  I have found the best way to do this is to lay the wiimote flat on the table, and then press the 1 and 2 buttons together while stabelizing the wiimote with your other hand.  If the wiimote shakes too much while pressing these buttons it will hang on calibration.

### (iii) Listening

Now that we have the proper nodes running, it is time to start listening.

Once again, open another terminal and run ipython.
                   
In ipython import the class_runSensors module, create a kinect subscriber, and tell it to listen:

    import class_runSensors; k = class_runSensors.kinect_subscriber(); k.listen()

If you've done everything correctly, you will now see an openrave window that tracks users in the kinects fov.

### (iv) Recording

Default save location: /home/rafihayne/workspace/statFiles/recorded_motion/motion_saved_0000X_0000Y.csv  where X is the kinect that is recording and Y is an incrementing unique file id.

Default image save location: /home/rafihayne/workspace/statFiles/snapshots/0_1375371202_384590.png
Where the first number is the kinect that is recording, and the following numbers are the timestamps that match the two last cells of each line in a motion csv file.
        
There are two options for recording.  You can start/stop recording from the terminal, or with a wiimote.  In general the terminal method is better for long recording, whereas the wiimote is good for recording short motions.

From the terminal: 
still in ipython simply tell the kinect subscriber object to begin recording

    k.rec(1)

to stop recording:

    k.rec(0)
            
With the wiimote:
create a wiimote subscriber object in ipython*:

    w = wiimote_subscriber(k.prob); w.run()

* you must pass the wiimote the problem that the kinect subscriber uses to communicate with the orkinect plugin. If you have been following the readme direcly this is k.prob

Now, when the A button on the wiimote is pressed the plugin will begin to record motion.  As soon as the A button is released, the pluin will halt recording.

# (b) Playback

### (i) Set directory

Before one can begin recording, there are a few settings that must be edited in the class_runSensors.py file.  Namely, the directory of saved motion files, the files to be played back, and enable/disabling of the camera.

    self.dir = "/home/rafihayne/workspace/statFiles/recorded_motion/"
    self.files = ["motion_saved_00000_00000.csv", "motion_saved_00001_00000.csv" ]

Here you put the motion you would like to play back.  There should be 1 motion file for each kinect or human that you want to play back. (The above example is playing back both humans).

    self.prob.SendCommand('EnableCamera 1')
    
Notice that EnableCamera is present in both the listen() and play() functions.  For playback edit the one in play()


### (ii) Launch Ros

Now that we have set the settings, we must run the appropriate nodes.

Run the following commands in individual terminal windows:
Start ROS:

    roscore
    
Run the image viewer:

    rosrun image_view image_view image:="/orkinect/kinect0"
    rosrun image_view image_view image:="/orkinect/kinect1"

### (iii) Playback

There are two options for playback: controlled and uncontrolled.

uncontrolled:

    run ipython

In ipython import the class_runSensors module, create a kinect subscriber, and tell it to play uncontrolled:

    import class_runSensors; k = class_runSensors.kinect_subscriber(); k.play(0)

controlled: almost identical to above:

    import class_runSensors; k = class_runSensors.kinect_subscriber(); k.play(1)

In order to use keyboard controlls, the ipython window must have focus.  When you start playing, openrave will steal focus; so click back onto the ipython window.

Controlls:

    u: Rewind 25 frames
    i: Rewind 1 frame
    o: Fast Forward 1 frame
    p: Fast Forward 25 frames
    q: quit (ipython will tell you that you must press ctrl+d to fully exit, but this kills the while loop listening to keyboard controlls)

space: Prints the current frame number

    1: Set the frame your desired segmented file starts on
    2: Set the frame your desired segmented file ends on
    s: Segment the file using the previously set splits.
