#ifndef DISTANCEFEATURE_HPP
#define DISTANCEFEATURE_HPP

#include <openrave/openrave.h>

#define UNIX

#include <libmove3d/planners/feature_space/features.hpp>
#include <libmove3d/planners/hri_costspace/human_trajectories/HRICS_human_features.hpp>
#include <libmove3d/planners/API/project.hpp>


namespace HRICS
{

class FeaturesOpenRAVE
{
public:
    FeaturesOpenRAVE() {}

    void init_dist( std::string human_name_1, std::string human_name_2 );
    DistanceFeature* getDistFeat() { return dist_feat_;}
    std::vector<Eigen::VectorXd> getDistBuffer() { return dist_buffer; }
    void printDistances();
    //void printMinDistances();
    void bufferDistance();

    void init_vel( std::string human_name );
    VelocityFeature* getVelFeat() { return vel_feat_;}
    std::vector< std::vector<Eigen::Vector3d> > getVelBuffer() { return vel_buffer; }
    void printVelocities();
    void bufferVelocity(double dt);

private:
    DistanceFeature* dist_feat_;
    std::vector<Eigen::VectorXd> dist_buffer;

    VelocityFeature* vel_feat_;
    std::vector< std::vector<Eigen::Vector3d> > vel_buffer;
    Move3D::confPtr_t q_last_;
    double dt_tmp_;
};

};

#endif // DISTANCEFEATURE_HPP
