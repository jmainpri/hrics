#ifndef DISTANCEFEATURE_HPP
#define DISTANCEFEATURE_HPP

#include <openrave/openrave.h>

#define UNIX

#include <libmove3d/planners/feature_space/features.hpp>
#include <libmove3d/planners/hri_costspace/human_trajectories/HRICS_human_features.hpp>
#include <libmove3d/planners/API/project.hpp>


#include "positionFeature.hpp"


namespace HRICS
{

class FeaturesOpenRAVE
{
public:
    FeaturesOpenRAVE( std::string human_name_1, std::string human_name_2);

    void init_dist( std::string human_name_1, std::string human_name_2 );
    DistanceFeature* getDistFeat() { return dist_feat_;}
    std::vector<Eigen::VectorXd> getDistBuffer() { return dist_buffer; }
    void printDistances();
    void printMinDistances();
    void bufferDistance();

    void init_vel( std::string human_name );
    VelocityFeature* getVelFeat() { return vel_feat_;}
    std::vector< std::vector<Eigen::Vector3d> > getVelBuffer() { return vel_buffer; }
    void printVelocities();
    void bufferVelocity(double dt);

    std::vector< std::vector<double> > getCurviture();
    std::vector< std::vector < double > > getSpeed();
    int getMaxWristDistance(std::string human_name);

    void init_pos( std::string human_name );
    PositionFeature* getPosFeat() { return pos_feat_;}
    std::vector< std::vector<Eigen::Vector3d> > getPosBuffer() { return pos_buffer; }
    void printPosition();
    void bufferPosition();

    void init_col( std::string human_name );
    CollisionFeature* getColFeat() { return col_feat_;}
    std::vector<Move3D::FeatureVect > getColBuffer() { return col_buffer; }
    void bufferCollision();

private:
    DistanceFeature* dist_feat_;
    std::vector<Eigen::VectorXd> dist_buffer;

    VelocityFeature* vel_feat_;
    std::vector< std::vector<Eigen::Vector3d> > vel_buffer;
    Move3D::confPtr_t q_last_;
    double dt_tmp_;

    PositionFeature* pos_feat_;
    std::vector<std::vector<Eigen::Vector3d> > pos_buffer;

    CollisionFeature* col_feat_;
    std::vector<Move3D::FeatureVect > col_buffer;
    std::vector<int> col_active_dofs;
    std::string col_human; //CollisionFeature should have a robot getter

    Move3D::Robot* human_1_;
    Move3D::Robot* human_2_;
};

};

#endif // DISTANCEFEATURE_HPP
