#ifndef SKELETONLISTENER_HPP
#define SKELETONLISTENER_HPP

#include <ros/ros.h>
#include <tf/transform_listener.h>
//#include "openni_tracker/confidence.h"
//#include "openni_tracker/confidence_array.h"

#include <openrave/openrave.h>

#include <Eigen/Dense>

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

class SkeletonListener
{
public:
    SkeletonListener(OpenRAVE::EnvironmentBasePtr penv);

    void listen();

private:

    void drawFrame(const Eigen::Affine3d& t);
    void draw();

    void setEigenPositions(int id);
    void setHumanConfiguration(int id);
    void setKinectFrame();
    void printDofNames();
    //void setConfidence( std::string name, double conf );
    //void readConfidence(const openni_tracker::confidence_array& msg );
    void set_joint_name_map();

    ros::NodeHandle* node_;
    ros::Subscriber sub_;

    std::vector< std::vector<tf::StampedTransform> > transforms_;
    std::vector<Eigen::VectorXd> confidences_;
    std::vector<Eigen::Vector3d> pos_;
    std::vector<bool> user_is_tracked_;
    std::vector<int> tracked_user_id_;
    Eigen::Affine3d kinect_to_origin_;
    int max_num_skel_;

    OpenRAVE::EnvironmentBasePtr env_;
    OpenRAVE::RobotBasePtr human_;
    std::vector<boost::shared_ptr<void> > graphptrs_;

    bool print_;

    std::vector<std::string> names_;
};

#endif // SKELETONLISTENER_HPP
