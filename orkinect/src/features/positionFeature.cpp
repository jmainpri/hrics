#include "positionFeature.hpp"

PositionFeature::PositionFeature( Move3D::Robot* active ) :
    human_active_(active)
{
//    human_active_joints_.push_back( human_active_->getJoint("Pelvis") );
//    human_active_joints_.push_back( human_active_->getJoint("rShoulderX") );
//    human_active_joints_.push_back( human_active_->getJoint("rElbowZ") );

    human_active_joints_.push_back( human_active_->getJoint("rWristX") );
}

// Compute velocity between two configurations
std::vector<Eigen::Vector3d> PositionFeature::getPosition( )
{
    std::vector<Eigen::Vector3d> pos(human_active_joints_.size());

    for( int i=0;i<human_active_joints_.size();i++)
    {
        pos[i] = human_active_joints_[i]->getVectorPos();
    }

    return pos;
}
