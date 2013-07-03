#ifndef SKELETONLISTENER_HPP
#define SKELETONLISTENER_HPP

#include <ros/ros.h>
#include <tf/transform_listener.h>
//#include "openni_tracker/confidence.h"
//#include "openni_tracker/confidence_array.h"

#include "recordMotion.hpp"

#include <openrave/openrave.h>

#include <Eigen/Dense>

/*
enum kinect_frames
{
  HEAD = 0,
  NECK = 1,
  TORSO = 2,

  SHOULDER_LEFT = 3,
  ELBOW_LEFT = 4,
  HAND_LEFT = 5,

  SHOULDER_RIGHT = 6,
  ELBOW_RIGHT = 7,
  HAND_RIGHT = 8,

  HIP_LEFT = 9,
  KNEE_LEFT = 10,
  FOOT_LEFT = 11,

  HIP_RIGHT = 12,
  KNEE_RIGHT = 13,
  FOOT_RIGHT = 14
};
*/

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

class SkeletonListener
{
public:
    SkeletonListener(OpenRAVE::EnvironmentBasePtr penv);

    void listen_once();

    void listen();

    void setKinectFrame(int KinID, double TX = 0, double TY = 0, double TZ = 0, double RotZ = 0, double RotY = 0);

    void setMotionRecorder(std::vector<HRICS::RecordMotion*> motion_recorder);

    void setRecord(bool buttonState);

    void setNumKinect(int num);

    void setTracker(bool cust_tracker) {custom_tracker_ = cust_tracker;}

private:

    void drawFrame(const Eigen::Affine3d& t);
    void draw();

    std::vector<bool> check_active_user();

    void tryToRecord();

    void setEigenPositions(int id);
    void setHumanConfiguration(int id, OpenRAVE::RobotBasePtr human);

    void printDofNames();
    //void setConfidence( std::string name, double conf );
    //void readConfidence(const openni_tracker::confidence_array& msg );
    void set_joint_name_map();

    ros::NodeHandle* node_;
    ros::Subscriber sub_;
    ros::Rate* rate_;
    tf::TransformListener* listener_;

    int listen_iter_;

    std::vector< std::vector<tf::StampedTransform> > transforms_;
    std::vector<Eigen::VectorXd> confidences_;
    std::vector< std::vector<Eigen::Vector3d> > pos_;
    std::vector<bool> user_is_tracked_;
    std::vector<int> tracked_user_id_;
    std::vector<int> active_kinect_;
    std::vector<int> users_id_to_kinect_;
    std::vector<Eigen::Affine3d> kinect_to_origin_; //Changed to a vector to support multiple kinects.
    int max_num_skel_;

    bool button_pressed_;
    bool custom_tracker_;
    int num_kinect_;

    std::vector<std::string> users_;
    void init_users();

    OpenRAVE::EnvironmentBasePtr env_;
    std::vector<OpenRAVE::RobotBasePtr> humans_;
    std::vector<boost::shared_ptr<void> > graphptrs_;

    bool print_;

    std::vector<std::string> names_;
    std::vector<HRICS::RecordMotion*> _motion_recorders;
};

#endif // SKELETONLISTENER_HPP
