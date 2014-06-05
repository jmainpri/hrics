#include "orFeatures.hpp"

#include <boost/bind.hpp>
#include <iomanip>

using namespace HRICS;
using namespace Move3D;

using std::cout;
using std::endl;

void FeaturesOpenRAVE::init_dist( std::string human_name_1, std::string human_name_2 )
{
    Robot* human1 = global_Project->getActiveScene()->getRobotByName(human_name_1);
    Robot* human2 = global_Project->getActiveScene()->getRobotByName(human_name_2);

    dist_feat_ = new DistanceFeature( human1, human2 );
}

void FeaturesOpenRAVE::printDistances()
{
    Eigen::VectorXd dist = dist_feat_->computeDistances();

    cout << "dist.size(): " << dist.size() << endl;

    for (int i = 0; i < dist.size(); i ++)
    {
        cout << dist_feat_->getDistanceNames()[i] << ": " << dist[i] << endl;
    }
}

//void FeaturesOpenRAVE::printMinDistances()
//{
//    std::vector< std::pair<int,double> > minDistances;

//    for (int i = 0; i < dist_buffer.size(); i ++)
//    {
//        for (int j = 0; j < dist_buffer[i].size(); j++)
//        {
//            if ( dist_buffer[i][j] != 0.0 )
//            {
//                minDistances.push_back( std::make_pair( i, std::min(minDistances[i].second, dist_buffer[i][j]) ) );
//            }
//        }
//    }

//    cout << endl << "Min distances for each joint: " << endl;
//    for (int i = 0; i < minDistances.size(); i++)
//    {
//        cout << dist_feat_->getDistanceNames()[i] << " " << minDistances[i].second << "at frame: " << minDistances[i].first << endl;
//    }
//}

void FeaturesOpenRAVE::bufferDistance()
{
    dist_buffer.push_back( dist_feat_->computeDistances() );
}

void FeaturesOpenRAVE::init_vel( std::string human_name )
{
    Robot* human1 = global_Project->getActiveScene()->getRobotByName(human_name);

    vel_feat_ = new VelocityFeature( human1 );
}

void FeaturesOpenRAVE::bufferVelocity(double dt)
{
    confPtr_t q_cur = vel_feat_->getRobot()->getCurrentPos();

    if( q_last_.get() == NULL ){
        q_last_ = vel_feat_->getRobot()->getCurrentPos();
        vel_buffer.push_back( vel_feat_->getVelocity( *q_cur, *q_cur, dt ) );
        return;
    }

    if( q_last_->equal( *q_cur) ){
        cout << "Configurations are equal" << endl;
        vel_buffer.push_back( vel_buffer.back() );
        return;
    }

    vel_buffer.push_back( vel_feat_->getVelocity( *q_last_, *q_cur, dt ) );
    q_last_ = q_cur;
}
