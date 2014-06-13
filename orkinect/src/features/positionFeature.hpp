#ifndef POSITIONFEATURE_HPP
#define POSITIONFEATURE_HPP

#include <libmove3d/planners/API/project.hpp>

class PositionFeature
{
public:
    PositionFeature( Move3D::Robot* active );

    std::vector<Eigen::Vector3d> getPosition( );
    Move3D::Robot* getRobot() { return human_active_; }

private:

    Move3D::Robot* human_active_;
    std::vector<Move3D::Joint*> human_active_joints_;
};

#endif // POSITIONFEATURE_HPP
