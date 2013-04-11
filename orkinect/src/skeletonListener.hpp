#ifndef SKELETONLISTENER_HPP
#define SKELETONLISTENER_HPP

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <openrave/openrave.h>

class SkeletonListener
{
public:
    SkeletonListener(OpenRAVE::EnvironmentBasePtr penv);

    void listen();

private:

    void draw();

    ros::NodeHandle* node_;
    std::vector< std::vector<tf::StampedTransform> > transforms_;
    std::vector<bool> user_is_tracked_;
    bool tracking_;
    int max_num_skel_;

    OpenRAVE::EnvironmentBasePtr env_;
    std::vector<boost::shared_ptr<void> > graphptrs_;

    bool print_;
};

#endif // SKELETONLISTENER_HPP
