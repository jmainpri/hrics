#================================LINKS
#0   base
#1   PelvisBody
#2   PlevisDummyTransX
#3   PlevisDummyTransY
#4   PlevisDummyTransZ
#5   PlevisDummyRotX
#6   PlevisDummyRotY
#7   torso
#8   TorsoDummyX
#9   TorsoDummyY
#10  head
#11  HeadDummyZ
#12  HeadDummyY
#13  rHumerus
#14  rSholderDummyX
#15  rSholderDummyZ
#16  rRadius
#17  rElbowDummy
#18  rHand
#19  rWristDummyX
#20  rWristDummyY
#21  lHumerus
#22  lShoulderDummyX
#23  lShoulderDummyZ
#24  lRadius
#25  lElbowDummy
#26  lHand
#27  lWristDummyX
#28  lWristDummyY
#29  rFemur
#30  rHipDummyX
#31  rHipDummyY
#32  rTibia
#33  rFoot
#34  rAnkleDummyX
#35  rAnkleDummyY
#36  lFemur
#37  lHipDummyX
#38  lHipDummyY
#39  lTibia
#40  lFoot
#41  lAnkleDummyX
#42  lAnkleDummyY

#==================================JOINTS
#0   PelvisTransX
#1   PelvisTransY
#2   PelvisTransZ
#3   PelvisRotX
#4   PelvisRotY
#5   PelvisRotZ
#6   TorsoX
#7   TorsoY
#8   TorsoZ
#9   HeadZ
#10  HeadY
#11  HeadX
#12  rShoulderX
#13  rShoulderZ
#14  rShoulderY
#15  rArmTrans
#16  rElbowZ
#17  rWristX
#18  rWristY
#19  rWristZ
#20  lShoulderX
#21  lShoulderZ
#22  lShoulderY
#23  lArmTrans
#24  lElbowZ
#25  lWristX
#26  lWristY
#27  lWristZ
#28  rHipX
#29  rHipY
#30  rHipZ
#31  rKnee
#32  rAnkleX
#33  rAnkleY
#34  rAnkleZ
#35  lHipX
#36  lHipY
#37  lHipZ
#38  lKnee
#39  lAnkleX
#40  lAnkleY
#41  lAnkleZ

#---------------- Left arm :

#21  lHumerus
# p : -0.007111 0.319938 0.432528
#22  lShoulderDummyX
# p : 0 0 0 
#23  lShoulderDummyZ
# p : 0 0 0 
#24  lRadius
# p : 0.003544 0.575555 0.43217
#25  lElbowDummy
# p : 0 0 0
#26  lHand
# p : 0.001872 0.753806 0.44719
# p : 0.001872 0.857954 0.472072
# p : 0.063221 0.78332 0.437595


from openravepy import *
import os
import sys
from numpy import *
from TransformMatrix import *
from rodrigues import *

orEnv = Environment()
h = []

def get_joint_transform(j):

    t = j.GetFirstAttached().GetTransform()
    p = j.GetAnchor()
    t[0,3] = p[0]
    t[1,3] = p[1]
    t[2,3] = p[2]
    return t


def draw_joint_frame(bodies,id):

    h.append( misc.DrawAxes( orEnv, get_joint_transform( bodies[0].GetJoints()[id] ) ) )


def draw_left_arm():

    h = []
    bodies = orEnv.GetBodies()
    draw_joint_frame(bodies,20)
    draw_joint_frame(bodies,21)
    draw_joint_frame(bodies,22)
    draw_joint_frame(bodies,23)
    draw_joint_frame(bodies,24)
    draw_joint_frame(bodies,25)
    draw_joint_frame(bodies,26)
    draw_joint_frame(bodies,27)


def print_parent_to_child(id_parent,id_child):
    
    bodies = orEnv.GetBodies()
    T_p = get_joint_transform( bodies[0].GetJoints()[id_parent] )
    T_c = get_joint_transform( bodies[0].GetJoints()[id_child] )
    T_p_inv = linalg.inv(T_p)
    T_origin = dot(T_c,T_p_inv)
    print "---------------------------------------------" 
    print bodies[0].GetJoints()[id_child].GetName()
    print T_origin


def get_joint_transform_by_name( name ):

    bodies = orEnv.GetBodies()

    for j in bodies[0].GetJoints():
        if j.GetName() == name:
            joint = j
            break

    return get_joint_transform(joint)

def launch_kinect():
    
    global orEnv
    global h
    
    orEnv.SetViewer('qtcoin')
    
    print "start"
    orEnv.SetDebugLevel(DebugLevel.Info)
    orEnv.Reset()
    orEnv.Load("../ormodels/human_wpi.xml")
    
    print "draw frame"
    T = MakeTransform( eye(3), transpose(matrix([0,0,0])))
    h.append( misc.DrawAxes( orEnv, matrix(T), 1 ) )
    
    print "try to create problem"
    prob = RaveCreateProblem(orEnv,'Kinect')
    
    #T = MakeTransform(yaw_pitch_roll_rotation([0,0,0]),transpose(matrix([1,0,0])))
    #orEnv.GetViewer().SetCamera([0.262839 -0.733602 -0.623389 0.0642694 2.99336 -0.755646 2.81558])
    #sys.stdin.readline()

    robots = orEnv.GetBodies()
    links = robots[0].GetLinks() #index and linkname at top of file
    joints = robots[0].GetJoints()

    #------------------------------
    print "Upper Body\n\n"

    #0   PelvisTransX
    #1   PelvisTransY
    #2   PelvisTransZ
    #3   PelvisRotX
    #4   PelvisRotY
    #5   PelvisRotZ
    #6   TorsoX
    #7   TorsoY
    #8   TorsoZ
    #9   HeadZ
    #10  HeadY
    #11  HeadX


    print "Can't get PelvisTransX: ID is 0"

    print_parent_to_child(0,1)
    print_parent_to_child(1,2)
    print_parent_to_child(2,3)
    print_parent_to_child(3,4)
    print_parent_to_child(4,5)
    print_parent_to_child(5,6)
    print_parent_to_child(6,7)
    print_parent_to_child(7,8)
    print_parent_to_child(8,9)
    print_parent_to_child(9,10)
    print_parent_to_child(10,11)



    #------------------------------
    print "\nLeft Arm \n"
    
    #20  lShoulderX
    #21  lShoulderZ
    #22  lShoulderY
    #23  lArmTrans
    #24  lElbowZ
    #25  lWristX
    #26  lWristY
    #27  lWristZ

    print_parent_to_child(6,20)
    print_parent_to_child(20,21)
    print_parent_to_child(21,22)
    print_parent_to_child(22,23)
    print_parent_to_child(23,24)
    print_parent_to_child(24,25)
    print_parent_to_child(25,26)
    print_parent_to_child(26,27)

    T1 = get_joint_transform_by_name("lShoulderX")
    T2 = get_joint_transform_by_name("lElbowZ")
    T3 = get_joint_transform_by_name("lWristZ")

    # lHumerus
    print "lHumerus : "
    p = [0.007111, -0.326332, 0.437434, 0]
    T_inv = linalg.inv(T1)
    print dot(T_inv,p)

    # lRadius
    print "lRadius : "
    p = [0.003544, 0.575555, 0.43217, 0]
    T_inv = linalg.inv(T2)
    print dot(T_inv,p)

    # lHand
    print "lHand : "
    p = [0.001872, 0.753806, 0.44719, 0]
    T_inv = linalg.inv(T3)
    print dot(T_inv,p)
    p = [0.001872, 0.857954, 0.472072, 0]
    T_inv = linalg.inv(T3)
    print dot(T_inv,p)
    p = [0.063221, 0.78332, 0.437595, 0]
    T_inv = linalg.inv(T3)
    print dot(T_inv,p)

    print "Fixed lShoulderX from lElbowZ:"
    print_parent_to_child(20,24)

    #------------------------------
    print "\nRight Arm\n"
    #12  rShoulderX
    #13  rShoulderZ
    #14  rShoulderY
    #15  rArmTrans
    #16  rElbowZ
    #17  rWristX
    #18  rWristY
    #19  rWristZ


    print_parent_to_child(6,12)
    print_parent_to_child(12,13)
    print_parent_to_child(13,14)
    print_parent_to_child(14,15)
    print_parent_to_child(15,16)
    print_parent_to_child(16,17)
    print_parent_to_child(17,18)
    print_parent_to_child(18,19)

    T1 = get_joint_transform_by_name("rShoulderX")
    T2 = get_joint_transform_by_name("rElbowZ")
    T3 = get_joint_transform_by_name("rWristZ")

    # rHumerus
    print "rHumerus : "
    p = [-0.007111, 0.319938, 0.432528, 0]
    T_inv = linalg.inv(T1)
    print dot(T_inv,p)

    # rRadius
    print "rRadius : "
    p = [0.003544, -0.581679, 0.43217, 0]
    T_inv = linalg.inv(T2)
    print dot(T_inv,p)

    # rHand
    print "rHand : "
    p = [0.001872, -0.75993, 0.44719, 0]
    T_inv = linalg.inv(T3)
    print dot(T_inv,p)
    p = [0.001872, -0.864078, 0.472072, 0]
    T_inv = linalg.inv(T3)
    print dot(T_inv,p)
    p = [0.063221, -0.789444, 0.437595, 0]
    T_inv = linalg.inv(T3)
    print dot(T_inv,p)

    print "Fixed rShoulderX from rElbowZ:"
    print_parent_to_child(12,16)



    #------------------------------
    print "\nRight Leg\n"
    #28  rHipX
    #29  rHipY
    #30  rHipZ
    #31  rKnee
    #32  rAnkleX
    #33  rAnkleY
    #34  rAnkleZ

    print_parent_to_child(0,28)
    print_parent_to_child(28,29)
    print_parent_to_child(29,30)
    print_parent_to_child(30,31)
    print_parent_to_child(31,32)
    print_parent_to_child(32,33)
    print_parent_to_child(33,34)

    #------------------------------
    print "\Left Leg\n"
    #35  lHipX
    #36  lHipY
    #37  lHipZ
    #38  lKnee
    #39  lAnkleX
    #40  lAnkleY
    #41  lAnkleZ

    print_parent_to_child(0,35)
    print_parent_to_child(35,36)
    print_parent_to_child(36,37)
    print_parent_to_child(37,38)
    print_parent_to_child(38,39)
    print_parent_to_child(39,40)
    print_parent_to_child(40,41)


    sys.stdin.readline()
    
if __name__ == "__main__":
    print "main function"
    launch_kinect()

