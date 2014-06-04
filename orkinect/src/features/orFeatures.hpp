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

    FeaturesOpenRAVE() {};
    FeaturesOpenRAVE(std::string human_1, std::string human_2);

public:
    DistanceFeature* getDistFeat() {return dist_feat_;}

private:
    DistanceFeature* dist_feat_;

};

};

#endif // DISTANCEFEATURE_HPP
