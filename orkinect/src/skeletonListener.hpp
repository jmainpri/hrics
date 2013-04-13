#ifndef SKELETONLISTENER_HPP
#define SKELETONLISTENER_HPP

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <openrave/openrave.h>

#include <Eigen/Dense>

class SkeletonListener
{
public:
    SkeletonListener(OpenRAVE::EnvironmentBasePtr penv);

    void listen();

private:

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

      KNEE_RIGHT = 9,
      HIP_LEFT = 10,
      FOOT_LEFT = 11,

      KNEE_LEFT = 12,
      HIP_RIGHT = 13,
      FOOT_RIGHT = 14
    };

    void draw();
    void setEigenPositions(int id);
    void setHumanConfiguration(int id);

    ros::NodeHandle* node_;
    std::vector< std::vector<tf::StampedTransform> > transforms_;
    std::vector<Eigen::Vector3d> pos_;
    std::vector<bool> user_is_tracked_;
    std::vector<int> tracked_user_id_;
    int max_num_skel_;

    OpenRAVE::EnvironmentBasePtr env_;
    OpenRAVE::RobotBasePtr human_;
    std::vector<boost::shared_ptr<void> > graphptrs_;

    bool print_;
};

#endif // SKELETONLISTENER_HPP
