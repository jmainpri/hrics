#ifndef JOINTLISTENER_HPP
#define JOINTLISTENER_HPP

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>

#include <openrave/openrave.h>

#include <Eigen/Dense>

#include "recordMotion.hpp"

namespace HRICS
{

enum kinect_frames
{
  HEAD = 0,
  NECK = 1,
  TORSO = 2,

  SHOULDER_LEFT = 6,
  ELBOW_LEFT = 7,
  HAND_LEFT = 8,

  SHOULDER_RIGHT = 3,
  ELBOW_RIGHT = 4,
  HAND_RIGHT = 5,

  HIP_LEFT = 12,
  KNEE_LEFT = 13,
  FOOT_LEFT = 14,

  HIP_RIGHT = 9,
  KNEE_RIGHT = 10,
  FOOT_RIGHT = 11
};

class JointListener
{
public:

    ros::NodeHandle nh_;
    //ros::Publisher state_pub_;

    JointListener(OpenRAVE::EnvironmentBasePtr penv, ros::NodeHandle nh);

    void listen_cb(const sensor_msgs::JointState::ConstPtr& msg);

    void listen();

    void setKinectFrame(int KinID, double TX = 0, double TY = 0, double TZ = 0, double RotZ = 0, double RotY = 0);

private:

    void drawFrame(const Eigen::Affine3d& t);
    void draw();
    void tryToRecord();
    void checkStartingPos();
    void setEigenPositions(int id);
    void setHumanConfiguration(int id, OpenRAVE::RobotBasePtr human);

    bool rec_state_;

    //void setConfidence( std::string name, double conf );
    //void readConfidence(const openni_tracker::confidence_array& msg );

    ros::Subscriber sub_;
    ros::Rate* rate_;
    OpenRAVE::RobotBasePtr human_;
    OpenRAVE::EnvironmentBasePtr env_;
    HRICS::RecordMotion* motion_recorder_;
};
}

#endif // JOINTLISTENER_HPP
