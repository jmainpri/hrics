// openni_tracker.cpp

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>
#include <openni_tracker/confidence_array.h>

#include <unistd.h>
#include <stdlib.h>
#include <termios.h>
#include <fcntl.h>

#include "NiTE.h"

using std::string;

#define MAX_USERS 10

#define CHECK_RC(nRetVal, what)										\
	if (nRetVal != XN_STATUS_OK)									\
	{																\
		ROS_ERROR("%s failed: %s", what, xnGetStatusString(nRetVal));\
		return nRetVal;												\
	}
	
#define USER_MESSAGE(msg) \
	{printf("[%08llu] User #%d:\t%s\n",ts, user.getId(),msg);}
	
bool g_visibleUsers[MAX_USERS] = {false};
nite::SkeletonState g_skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};

bool g_bNeedPose   = false;
char g_strPose[20] = "";

std::vector<nite::JointType> g_jointTypes;

openni_tracker::confidence_array confidences;

//----------------------------------------------------------------------------
//----------------------------------------------------------------------------
//----------------------------------------------------------------------------

void setJointTypesVector()
{
    g_jointTypes.clear();

    g_jointTypes.push_back( nite::JOINT_HEAD );
    g_jointTypes.push_back( nite::JOINT_NECK );
    g_jointTypes.push_back( nite::JOINT_TORSO );
    
    g_jointTypes.push_back( nite::JOINT_LEFT_SHOULDER );
    g_jointTypes.push_back( nite::JOINT_LEFT_ELBOW );
    g_jointTypes.push_back( nite::JOINT_LEFT_HAND );
    
    g_jointTypes.push_back( nite::JOINT_RIGHT_SHOULDER );
    g_jointTypes.push_back( nite::JOINT_RIGHT_ELBOW );
    g_jointTypes.push_back( nite::JOINT_RIGHT_HAND );
    
    g_jointTypes.push_back( nite::JOINT_LEFT_HIP );
    g_jointTypes.push_back( nite::JOINT_LEFT_KNEE );
    g_jointTypes.push_back( nite::JOINT_LEFT_FOOT );
    
    g_jointTypes.push_back( nite::JOINT_RIGHT_HIP );
    g_jointTypes.push_back( nite::JOINT_RIGHT_KNEE );
    g_jointTypes.push_back( nite::JOINT_RIGHT_FOOT );
}

bool publishConfidence( const nite::UserData& user, nite::JointType const& joint_type, string const& frame_id, string const& child_frame_id )
{
    const nite::SkeletonJoint& joint = user.getSkeleton().getJoint( joint_type );
    
    openni_tracker::confidence confidence;
    
    char child_frame_no[128];
    snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), user.getId());
    
    confidence.header.stamp = ros::Time::now();
    confidence.child_frame_id = child_frame_no;
    confidence.confidence = joint.getPositionConfidence();
    
    confidences.confidence_array.push_back( confidence );
    
    return true;
}

bool publishConfidence( const nite::UserData& user, const std::string& frame_id )
{
    publishConfidence( user, nite::JOINT_HEAD,           frame_id, "head");
    publishConfidence( user, nite::JOINT_NECK,           frame_id, "neck");
    publishConfidence( user, nite::JOINT_TORSO,          frame_id, "torso");

    publishConfidence( user, nite::JOINT_LEFT_SHOULDER,  frame_id, "left_shoulder");
    publishConfidence( user, nite::JOINT_LEFT_ELBOW,     frame_id, "left_elbow");
    publishConfidence( user, nite::JOINT_LEFT_HAND,      frame_id, "left_hand");

    publishConfidence( user, nite::JOINT_RIGHT_SHOULDER, frame_id, "right_shoulder");
    publishConfidence( user, nite::JOINT_RIGHT_ELBOW,    frame_id, "right_elbow");
    publishConfidence( user, nite::JOINT_RIGHT_HAND,     frame_id, "right_hand");

    publishConfidence( user, nite::JOINT_LEFT_HIP,       frame_id, "left_hip");
    publishConfidence( user, nite::JOINT_LEFT_KNEE,      frame_id, "left_knee");
    publishConfidence( user, nite::JOINT_LEFT_FOOT,      frame_id, "left_foot");

    publishConfidence( user, nite::JOINT_RIGHT_HIP,      frame_id, "right_hip");
    publishConfidence( user, nite::JOINT_RIGHT_KNEE,     frame_id, "right_knee");
    publishConfidence( user, nite::JOINT_RIGHT_FOOT,     frame_id, "right_foot");
    
    return true;
}

void publishTransform( const nite::UserData& user, nite::JointType const& joint_type, string const& frame_id, string const& child_frame_id ) 
{
    static tf::TransformBroadcaster br;
    
    const nite::SkeletonJoint& joint = user.getSkeleton().getJoint(joint_type);
   

    nite::Point3f pos = joint.getPosition();
    double x = -pos.x / 1000.0;
    double y = pos.y / 1000.0;
    double z = pos.z / 1000.0;

    nite::Quaternion rot = joint.getOrientation();

//    XnFloat* m = joint_orientation.orientation.elements;
//    KDL::Rotation rotation(m[0], m[1], m[2],
//    					   m[3], m[4], m[5],
//    					   m[6], m[7], m[8]);
//    double qx, qy, qz, qw;
//    rotation.GetQuaternion(qx, qy, qz, qw);

    char child_frame_no[128];
    snprintf(child_frame_no, sizeof(child_frame_no), "%s_%d", child_frame_id.c_str(), user.getId());

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x, y, z));
    //transform.setRotation(tf::Quaternion(0, 0, 0, 0));
    transform.setRotation(tf::Quaternion(rot.x, -rot.y, -rot.z, rot.w));

    double yaw,pitch,roll;
    //transform.getBasis().getEulerYPR(yaw,pitch,roll);
    //std::cout << yaw << " , " << pitch << " , "  << roll << std::endl;
    
    // #4994
    tf::Transform change_frame;
    change_frame.setOrigin(tf::Vector3(0, 0, 0));
    tf::Quaternion frame_rotation;
	
    frame_rotation.setRPY(1.5708, 0, 1.5708);
    change_frame.setRotation(frame_rotation);

    transform = change_frame * transform;
    
    transform.getBasis().getEulerYPR(yaw,pitch,roll);
    //std::cout << yaw << " , " << pitch << " , "  << roll << std::endl;
    if( isnan(yaw) || isnan(pitch) || isnan(roll) )
    {
        transform.getBasis().setEulerZYX(0,0,0);
    }
    
    //transform.getBasis().getEulerYPR(yaw,pitch,roll);
    //std::cout << yaw << " , " << pitch << " , "  << roll << std::endl;

    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_no));
}

bool publishTransforms( const nite::UserData& user, const std::string& frame_id ) 
{
    bool track = false;
    
    for(int i=0;i<int(g_jointTypes.size());i++)
    {
        if( 0.5 < user.getSkeleton().getJoint( g_jointTypes[i] ).getPositionConfidence() )
        {
            track = true;
            break;
        }    
    }
    
    if( !track )
        return false;
    
    publishTransform( user, nite::JOINT_HEAD,           frame_id, "head");
    publishTransform( user, nite::JOINT_NECK,           frame_id, "neck");
    publishTransform( user, nite::JOINT_TORSO,          frame_id, "torso");

    publishTransform( user, nite::JOINT_LEFT_SHOULDER,  frame_id, "left_shoulder");
    publishTransform( user, nite::JOINT_LEFT_ELBOW,     frame_id, "left_elbow");
    publishTransform( user, nite::JOINT_LEFT_HAND,      frame_id, "left_hand");

    publishTransform( user, nite::JOINT_RIGHT_SHOULDER, frame_id, "right_shoulder");
    publishTransform( user, nite::JOINT_RIGHT_ELBOW,    frame_id, "right_elbow");
    publishTransform( user, nite::JOINT_RIGHT_HAND,     frame_id, "right_hand");

    publishTransform( user, nite::JOINT_LEFT_HIP,       frame_id, "left_hip");
    publishTransform( user, nite::JOINT_LEFT_KNEE,      frame_id, "left_knee");
    publishTransform( user, nite::JOINT_LEFT_FOOT,      frame_id, "left_foot");

    publishTransform( user, nite::JOINT_RIGHT_HIP,      frame_id, "right_hip");
    publishTransform( user, nite::JOINT_RIGHT_KNEE,     frame_id, "right_knee");
    publishTransform( user, nite::JOINT_RIGHT_FOOT,     frame_id, "right_foot");
    
    return true;
}
	
void updateUserState(const nite::UserData& user, unsigned long long ts)
{
	if (user.isNew())
		USER_MESSAGE("New")
	else if (user.isVisible() && !g_visibleUsers[user.getId()])
		USER_MESSAGE("Visible")
	else if (!user.isVisible() && g_visibleUsers[user.getId()])
		USER_MESSAGE("Out of Scene")
	else if (user.isLost())
		USER_MESSAGE("Lost")

	g_visibleUsers[user.getId()] = user.isVisible();

	if(g_skeletonStates[user.getId()] != user.getSkeleton().getState())
	{
		switch(g_skeletonStates[user.getId()] = user.getSkeleton().getState())
		{
		case nite::SKELETON_NONE:
			USER_MESSAGE("Stopped tracking.")
			break;
		case nite::SKELETON_CALIBRATING:
			USER_MESSAGE("Calibrating...")
			break;
		case nite::SKELETON_TRACKED:
			USER_MESSAGE("Tracking!")
			break;
		case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
		case nite::SKELETON_CALIBRATION_ERROR_HANDS:
		case nite::SKELETON_CALIBRATION_ERROR_LEGS:
		case nite::SKELETON_CALIBRATION_ERROR_HEAD:
		case nite::SKELETON_CALIBRATION_ERROR_TORSO:
			USER_MESSAGE("Calibration Failed... :-|")
			break;
		}
	}
}

int main(int argc, char **argv) 
{
    ros::init( argc, argv, "openni_tracker" );  
    ros::NodeHandle nh;
    
    std::cout << "Start OPENNI TRACKER node NiTe2" << std::endl;
    //string configFilename = ros::package::getPath("openni_tracker") + "/openni_tracker.xml";
    
    nite::UserTracker userTracker;
    nite::UserTrackerFrameRef userTrackerFrame;
	nite::Status niteRc;
	
	string configFilename = ros::package::getPath("openni_tracker") + "/openni_tracker.xml";
	nite::NiTE::initialize();
	
	niteRc = userTracker.create();
	
	if (niteRc != nite::STATUS_OK)
	{
		printf("Couldn't create user tracker\n");
		return 3;
	}
	printf("\nStart moving around to get detected...\n(PSI pose may be required for skeleton calibration, depending on the configuration)\n");


    ros::Rate r(30);
    ros::NodeHandle pnh("~");
    string frame_id("openni_depth_frame");
    pnh.getParam( "camera_frame_id", frame_id );
    ros::Publisher confidence_pub = pnh.advertise<openni_tracker::confidence_array>("openni_confidences", 1000);
	
	setJointTypesVector();
	
	while (ros::ok())
	{
		niteRc = userTracker.readFrame(&userTrackerFrame);
		
		if ( niteRc != nite::STATUS_OK )
		{
			printf("Get next frame failed\n");
			continue;
		}

		const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
		
		bool found_skel = false;
		
		confidences.confidence_array.clear();
		
		for (int i = 0; i < users.getSize(); ++i)
		{
			const nite::UserData& user = users[i];
			
			updateUserState( user, userTrackerFrame.getTimestamp() );
			
			if (user.isNew())
			{
				userTracker.startSkeletonTracking(user.getId());
				found_skel = true;
			}
			else if (user.getSkeleton().getState() == nite::SKELETON_TRACKED)
			{
				//const nite::SkeletonJoint& head = user.getSkeleton().getJoint(nite::JOINT_HEAD);
				//printf("%d. (%5.2f, %5.2f, %5.2f)\n", user.getId(), head.getPosition().x, head.getPosition().y, head.getPosition().z);
				printf("Tracking user : %d\n", user.getId());
				if( publishTransforms( user, frame_id ) )
				    found_skel = true;
				    
			    publishConfidence( user, frame_id );
			}
		}
		
		confidence_pub.publish( confidences );
		
		if( !found_skel )
		{
		    printf("No tracking\n");
		}
		r.sleep();
	}

	nite::NiTE::shutdown();
}
