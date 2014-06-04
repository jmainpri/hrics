#include "orFeatures.hpp"

#include <boost/bind.hpp>
#include <iomanip>

using namespace HRICS;
using namespace Move3D;

using std::cout;
using std::endl;

FeaturesOpenRAVE::FeaturesOpenRAVE( std::string human_name_1, std::string human_name_2 )
{
    Move3D::Robot* human1 = Move3D::global_Project->getActiveScene()->getRobotByName(human_name_1);
    Move3D::Robot* human2 = Move3D::global_Project->getActiveScene()->getRobotByName(human_name_2);

    //Init distance feature
    dist_feat_ = new DistanceFeature( human1, human2 );
}
