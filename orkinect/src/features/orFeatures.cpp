#include "orFeatures.hpp"

#include <boost/bind.hpp>
#include <iomanip>

using namespace HRICS;
using namespace Move3D;

using std::cout;
using std::endl;

FeaturesOpenRAVE::FeaturesOpenRAVE( std::string human_name_1, std::string human_name_2 )
{
    Robot* human_1_ = global_Project->getActiveScene()->getRobotByName(human_name_1);
    Robot* human_2_ = global_Project->getActiveScene()->getRobotByName(human_name_2);
}


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

//    cout << "dist_buffer.size()" << dist_buffer.size() << endl;
//    cout << "dist_buffer[0].size() " << dist_buffer[0].size() << endl;


////    for (int i = 0; i < dist_buffer.size(); i ++)
////    {
////        for (int j = 0; j < dist_buffer[i].size(); j++)
////        {

////            if ( dist_buffer[i][j] != 0.0 && i < minDistances.size() )
////            {
////                minDistances.push_back( std::make_pair( j, std::min(minDistances[i].second, dist_buffer[i][j]) ) );
////            }
////            else
////            {
////                minDistances.push_back( std::make_pair(j, dist_buffer[i][j]) );
////            }
////        }
////    }

//    cout << endl << "Min distances for each joint: " << endl;
//    for (int i = 0; i < minDistances[i].size(); i++)
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

std::vector< std::vector < double > > FeaturesOpenRAVE::getCurviture()
{
    std::vector< std::vector < double > > curviture;

    curviture.resize(vel_buffer.size());

    //Find the curviture
    for (int i = 0; i < vel_buffer.size(); i++) //Each frame
    {
        curviture[i].resize(vel_buffer[0].size());

        for (int j = 0; j < vel_buffer[0].size(); j++) //Each Joint
        {
            if (i == 0)
                curviture[i][j] = 0.0;
            else
            {
                Eigen::Vector3d v1 = vel_buffer[i][j].normalized();
                Eigen::Vector3d v2 = vel_buffer[i-1][j].normalized();

                double angle = atan2( v1.cross(v2).norm() , v1.dot(v2) ) ;
                curviture[i][j] = angle;
            }
        }
    }

    //Smooth the curviture with a moving average
    std::vector<std::vector<double> > smoothed_curv = curviture;

    int numFrames = curviture.size();
    for ( int i = 0; i < numFrames; i++ )
    {
        for ( int j = 0; j < curviture[0].size(); j++)
        {
            double smoothed = 0;

            for ( int k = -3; k <= 3; k++)
            {

                if ( (i + k) < 0)
                    smoothed += curviture[0][j];
                else if (i + k > numFrames)
                    smoothed += curviture[numFrames][j];
                else
                    smoothed += curviture[i+k][j];
            }
            smoothed = smoothed/7;

            smoothed_curv[i][j] = smoothed;
        }
    }

    return smoothed_curv;
}

std::vector< std::vector < double > > FeaturesOpenRAVE::getSpeed()
{
    std::vector< std::vector < double > > speed;

    speed.resize(vel_buffer.size());

    for (int i = 0; i < vel_buffer.size(); i++) //Each frame
    {
        speed[i].resize(vel_buffer[0].size());

        for (int j = 0; j < vel_buffer[0].size(); j++) //Each Joint
        {
            speed[i][j] = vel_buffer[i][j].norm();
        }
    }

    return speed;
}

//Returns the frame id
int FeaturesOpenRAVE::getMaxWristDistance(std::string human_name)
{
    int pelv_id;
    int wrist_id;
    Robot* human = global_Project->getActiveScene()->getRobotByName(human_name);

    if ( !human->getUseLibmove3dStruct() ) //We're using OpenRAVE, use different joints
    {
        wrist_id =  human->getJoint("rWristX")->getId();
        pelv_id =  human->getJoint("PelvisRotX")->getId();
    }
    else    //Using Move3D
    {
        wrist_id =  human->getJoint("rWristX")->getId();
        pelv_id =  human->getJoint("Pelvis")->getId();
    }

    double max_dist = 0;
    int max_frame = 0;

    for (int i = 0; i < pos_buffer.size(); i++) //Each frame
    {
        //double new_dist = ( pos_buffer[i][pelv_id] - pos_buffer[i][wrist_id] ).norm();
        double new_dist = ( pos_buffer[i][0] ).norm();

        if ( new_dist > max_dist )
        {
            max_dist = new_dist;
            max_frame = i;
        }

    }

    return max_frame;
}

void FeaturesOpenRAVE::init_pos( std::string human_name )
{
    Robot* human = global_Project->getActiveScene()->getRobotByName(human_name);

    pos_feat_ = new PositionFeature( human );
}

void FeaturesOpenRAVE::bufferPosition()
{
    pos_buffer.push_back( pos_feat_->getPosition( ) );

}

void FeaturesOpenRAVE::printPosition()
{
    std::cout << "Trying to print pos: " << std:: endl;
    //std::vector<Eigen::Vector3d>  pos =
            pos_feat_->getPosition();

//    cout << "pos.size(): " << pos.size() << endl;

//    for (int i = 0; i < pos.size(); i ++)
//    {
//        cout << "pos[" << i << "]: " << pos[i] << endl;
//    }
}

void FeaturesOpenRAVE::init_col( std::string human_name )
{
    col_human = human_name;
    Robot* human = global_Project->getActiveScene()->getRobotByName(human_name);
    col_feat_ = new CollisionFeature( human );
    col_feat_->init();

    if ( !human->getUseLibmove3dStruct() ) //We're using OpenRAVE, use different joints
    {
        col_active_dofs.push_back( human->getJoint("rShoulderX")->getId() );
        col_active_dofs.push_back( human->getJoint("rElbowZ")->getId() );
        col_active_dofs.push_back( human->getJoint("rWristX")->getId() );
    }
    else    //Using Move3D
    {
        col_active_dofs.push_back( human->getJoint("rShoulderX")->getId() );
        col_active_dofs.push_back( human->getJoint("rElbowZ")->getId() );
        col_active_dofs.push_back( human->getJoint("rWristX")->getId() );
    }

}

void FeaturesOpenRAVE::bufferCollision()
{
    Robot* human = global_Project->getActiveScene()->getRobotByName(col_human);
    confPtr_t q = human->getCurrentPos();

    col_buffer.push_back( col_feat_->getFeatures( *q, col_active_dofs ) );

}

