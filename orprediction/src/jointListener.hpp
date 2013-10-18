#ifndef JOINTLISTENER_HPP
#define JOINTLISTENER_HPP

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <openrave/openrave.h>

#include <Eigen/Dense>
#include "classifyMotion.hpp"
#include "recordMotion.hpp"

#include <math.h>

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

    double likelihood_to_range(double x);

    ros::NodeHandle nh_;
    //ros::Publisher state_pub_;

    JointListener(OpenRAVE::EnvironmentBasePtr penv, ros::NodeHandle nh);

    void listen_cb(const sensor_msgs::JointState::ConstPtr& msg);

    void listen();

    void setKinectFrame(int KinID, double TX = 0, double TY = 0, double TZ = 0, double RotZ = 0, double RotY = 0);

    void classifyLibrary();

private:

    void drawFrame(const Eigen::Affine3d& t);
    void draw();
    void tryToRecord();
    bool checkRestingPos(double offset);
    double getRestingOffset();
    std::vector<double> classifyMotion( const motion_t& motion );
    void setMatrixCol(Eigen::MatrixXd& matrix, int j, confPtr_t q);
    void setEigenPositions(int id);
    void setHumanConfiguration(int id, OpenRAVE::RobotBasePtr human);
    void getRestingRot();
    void draw_classes(std::vector<double> likelihood);
    void load_classes();
    std::vector<Eigen::Affine3d> update_classes();

    bool rest_state_;
    bool motion_started_;


    //void setConfidence( std::string name, double conf );
    //void readConfidence(const openni_tracker::confidence_array& msg );

    ros::Publisher marker_pub_;
    ros::Subscriber sub_;
    ros::Rate* rate_;
    OpenRAVE::RobotBasePtr human_;
    OpenRAVE::EnvironmentBasePtr env_;
    HRICS::RecordMotion* motion_recorder_;
    HRICS::ClassifyMotion* m_classifier_;
    std::vector<Eigen::Affine3d> class_offsets_; //Right wrist offsets from pelvis for motion classes
    std::vector<boost::shared_ptr<void> > graphptrs_; //Pointers to class spheres
    Eigen::Affine3d rest_rot_;
    std::vector<Eigen::Affine3d> updated_offsets_;
};
}

#endif // JOINTLISTENER_HPP
