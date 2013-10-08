#include "jointListener.hpp"

#include <Eigen/Geometry>

#include "../../orcommon/include/orcommon.hpp"

using namespace HRICS;
using std::cout;
using std::endl;

JointListener::JointListener(OpenRAVE::EnvironmentBasePtr penv, ros::NodeHandle nh) : nh_(nh)
{
    cout << "Enter constructer" << endl;

    env_ = penv;

    cout << "start suscriber" << endl;
    sub_ = nh_.subscribe("/human_state", 10, &JointListener::listen_cb, this );

    rest_state_ = 0;
    human_ = env_->GetRobot( "human_model" );
    motion_recorder_ = new HRICS::RecordMotion(human_);
    motion_recorder_->setRobotId(0);
    motion_recorder_->setBuffSize(200);
    motion_recorder_->setNumKeep(100);
    m_classifier_ = new HRICS::ClassifyMotion();
    motion_started_ = false;

    if (m_classifier_->load_model())
        cout << "Successfully loaded classifier" <<endl;
    else
        cout << "Couldn't initialize classifier" << endl;


//    OpenRAVE::RaveTransform<float> Tcam;
//    Tcam.rot.x = 0.420745;
//    Tcam.rot.y = -0.574182;
//    Tcam.rot.z = 0.605033;
//    Tcam.rot.w = -0.356683;
//    Tcam.trans.x = -2.61378;
//    Tcam.trans.y = -0.0481125;
//    Tcam.trans.z =  0.888456;

    // OpenRAVE::RaveTransformMatrix<float> T;
    // T.m[0] = -0.704743; T.m[1] = 0.304894; T.m[2] = -0.640607;  T.trans.x =  0; 1.11025;
    // T.m[4] = -0.310475; T.m[5] =-0.944433; T.m[6] = -0.10794;   T.trans.y = 0.112955;
    // T.m[8] = -0.637921; T.m[9] = 0.122822; T.m[10] =  0.760244; T.trans.z = -0.589495;

    // env_->GetViewer()->SetCamera( Tcam );
    //env_->GetViewer()->setCamera( 0.262839, -0.733602, -0.623389, 0.0642694, 2.99336, -0.755646, 2.81558 );

   // printDofNames();

}

void JointListener::listen_cb(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (msg->position.size() != human_->GetJoints().size())
    {
        cout << "Can't update robot! msg size: " << msg->position.size() << " robot joint size: " << human_->GetJoints().size() << endl;
    }

    else
    {

        human_->SetDOFValues(msg->position); //Update human from JointState
        double curOffset = getRestingOffset();
        motion_recorder_->bufferCurrentConfig( curOffset ); //Buffer the new config

        bool resting = checkRestingPos( curOffset );

        if(!resting && !motion_started_) //We're not resting, and we haven't found the true start of the motion yet
        {
            cout << "We're not resting anymore" << endl;
            motion_recorder_->findTrueStart();
            motion_started_ = true;
        }
        else if(resting)
        {
            cout << "We're resting" << endl;
            motion_started_ = false;
        }

        if (motion_started_ && motion_recorder_->getCurrentMotion().size() > 20)
        {
            cout << "We got a motion!" << endl;
            //motion_recorder_->saveCurrentToCSV();

            int m_class = classifyMotion( motion_recorder_->getCurrentMotion() );
            cout << "Class: " << m_class << endl;
        }
    }

}

void JointListener::listen()
{
    rate_ = new ros::Rate(40.0);

    motion_t m = motion_recorder_->loadFromCSV("/home/rafi/Desktop/motion_saved_00000_00000.csv");
    m = motion_recorder_->fixPelvisFrame(m);
    int m_class = classifyMotion( m );
    cout << "True Class: 0 Predicted Class: " << m_class << endl;

    m = motion_recorder_->loadFromCSV("/home/rafi/Desktop/motion_saved_00000_00001.csv");
    m = motion_recorder_->fixPelvisFrame(m);
    m_class = classifyMotion( m );
    cout << "True Class: 1 Predicted Class: " << m_class << endl;

    m = motion_recorder_->loadFromCSV("/home/rafi/Desktop/motion_saved_00000_00002.csv");
    m = motion_recorder_->fixPelvisFrame(m);
    m_class = classifyMotion( m );
    cout << "True Class: 2 Predicted Class: " << m_class << endl;

    m = motion_recorder_->loadFromCSV("/home/rafi/Desktop/motion_saved_00000_00003.csv");
    m = motion_recorder_->fixPelvisFrame(m);
    m_class = classifyMotion( m );
    cout << "True Class: 3 Predicted Class: " << m_class << endl;

    m = motion_recorder_->loadFromCSV("/home/rafi/Desktop/motion_saved_00000_00004.csv");
    m = motion_recorder_->fixPelvisFrame(m);
    m_class = classifyMotion( m );
    cout << "True Class: 4 Predicted Class: " << m_class << endl;

    m = motion_recorder_->loadFromCSV("/home/rafi/Desktop/motion_saved_00000_00005.csv");
    m = motion_recorder_->fixPelvisFrame(m);
    m_class = classifyMotion( m );
    cout << "True Class: 5 Predicted Class: " << m_class << endl;

    m = motion_recorder_->loadFromCSV("/home/rafi/Desktop/motion_saved_00000_00006.csv");
    m = motion_recorder_->fixPelvisFrame(m);
    m_class = classifyMotion( m );
    cout << "True Class: 6 Predicted Class: " << m_class << endl;

    m = motion_recorder_->loadFromCSV("/home/rafi/Desktop/motion_saved_00000_00007.csv");
    m = motion_recorder_->fixPelvisFrame(m);
    m_class = classifyMotion( m );
    cout << "True Class: 7 Predicted Class: " << m_class << endl;

//    while (nh_.ok())
//    {
//        rate_->sleep();
//        ros::spinOnce();
//    }
}

bool JointListener::checkRestingPos(double offset)
{
    if (offset > .15)
    {
        return false;
    }
    else
        return true;

}

double JointListener::getRestingOffset()
{
    Eigen::Vector3d p = or_vector_to_eigen( human_->GetJoint("rWristX")->GetAnchor() );
    Eigen::Affine3d T = get_joint_transform( human_->GetJoint("PelvisRotX") );
    p = T.inverse()*p;

    Eigen::Vector3d pInitial( -0.20, 0.10, -0.05 );  //TODO recalculate ideal initial resting offset
    double offset = (p-pInitial).norm();

    return offset;
}

int JointListener::classifyMotion( const motion_t& motion )
{
    std::vector<double> likelihood;

    Eigen::MatrixXd matrix( 13, motion.size()-1 );

    for (int jn=0; jn<(motion.size()-1); jn++)
    {
        setMatrixCol( matrix, jn, motion[jn].second );
    }

    likelihood = m_classifier_->classify_motion( matrix );



//    // for all collumns (a configuration per collumn)
//    for (int j=1; j<int(motion.size()); j++)
//    {
//        Eigen::MatrixXd matrix( 13, j );

//        for (int jn=0; jn<j; jn++)
//        {
//            setMatrixCol( matrix, jn, motion[jn].second );
//        }

//        likelihood = m_classifier_->classify_motion( matrix );
//        for (int i = 0; i < int(likelihood.size()); i++){
//            cout << likelihood[i] << ", ";
//        }
//        cout << endl;
//        cout << std::max_element(likelihood.begin(),likelihood.end()) - likelihood.begin() << " ";
//    }

    return std::max_element(likelihood.begin(),likelihood.end()) - likelihood.begin();
}

void JointListener::setMatrixCol(Eigen::MatrixXd& matrix, int j, confPtr_t q)
{
//    matrix(0,j) = j;        // Index
//    matrix(1,j) = (*q)[0];  // PelvisTransX
//    matrix(2,j) = (*q)[1];  // PelvisTransY
//    matrix(3,j) = (*q)[2];  // PelvisTransZ
//    matrix(4,j) = (*q)[5];  // PelvisRotZ
//    matrix(5,j) = (*q)[6];  // TorsoX
//    matrix(6,j) = (*q)[7];  // TorsoY
//    matrix(7,j) = (*q)[8];  // TorsoZ

//    matrix(8,j) =  (*q)[12];  // rShoulderX
//    matrix(9,j) =  (*q)[13];  // rShoulderZ
//    matrix(10,j) = (*q)[14];  // rShoulderY
//    matrix(11,j) = (*q)[15];  // rArmTrans
//    matrix(12,j) = (*q)[16];  // rElbowZ

    matrix(0,j) = j;        // Index
    matrix(1,j) = q[0];  // PelvisTransX
    matrix(2,j) = q[1];  // PelvisTransY
    matrix(3,j) = q[2];  // PelvisTransZ
    matrix(4,j) = q[5];  // PelvisRotZ
    matrix(5,j) = q[6];  // TorsoX
    matrix(6,j) = q[7];  // TorsoY
    matrix(7,j) = q[8];  // TorsoZ

    matrix(8,j) =  q[12];  // rShoulderX
    matrix(9,j) =  q[13];  // rShoulderZ
    matrix(10,j) = q[14];  // rShoulderY
    matrix(11,j) = q[15];  // rArmTrans
    matrix(12,j) = q[16];  // rElbowZ
}

void JointListener::tryToRecord()
{
    if( !rest_state_ ) //We have left the rest pose.  Start recording.
    {
        motion_recorder_->m_is_recording = true;
        motion_recorder_->saveCurrentConfig();
    }
    else if ( rest_state_ )
    {

        if (motion_recorder_->m_is_recording) //If we're in the rest state and we're currently recording, stop recording
        {
            //motion_recorder_->saveCurrentToCSV();
            motion_recorder_->clearCurrentMotion();
            motion_recorder_->m_is_recording = false;
        }

    }
}



/**
void JointListener::setConfidence( std::string name, double conf )
{
    //cout << "set : " << name << endl;

    for(int i=0;i<int(body_names_.size());i++)
    {
        if( name.find( body_names_[i] ) != std::string::npos )
        {
            std::string skel_id = name.substr(body_names_[i].size());
            //cout << "--" << name.substr(body_names_[i].size()) << "--" << endl;
            int id;

            if( string_to_num<int>( id, skel_id, std::dec) )
            {
                confidences_[id][i] = conf;
            }
        }
    }

    for(int i=0; i<int(tracked_user_id_.size()); i++)
    {
        cout << "confidence for (" << i  << "): " << confidences_[tracked_user_id_[i]].transpose() << endl;
    }
}

void JointListener::readConfidence(const openni_tracker::confidence_array& msg )
{
    //cout << "read confidence" << endl;

    for(int i=0;i<int(msg.confidence_array.size()); i++)
    {
        setConfidence( msg.confidence_array[i].child_frame_id , msg.confidence_array[i].confidence );
    }
}
**/


/*

void JointListener::printDofNames()
{
    if( humans_.empty() )
        return;

    const std::vector<OpenRAVE::KinBody::JointPtr> joints = humans_[0].robot_->GetJoints();

    for( int i=0;i<int(joints.size());i++)
    {
        cout << joints[i]->GetName() << " : " << joints[i]->GetDOFIndex() << endl;
    }
}

void JointListener::setEigenPositions(int id)
{
    //cout << "clear pos at id : " << id << endl;
    pos_[id].clear();
    pos_[id].resize(15);
    Eigen::Affine3d tempTransform;

    //TODO replace with somehing more clever!!!
    for (int k = 0; k < num_kinect_; k++)
    {
        if (id < (k+1)*max_num_skel_)
        {
            //            cout << "Using transform k: " << k << " on ID: " << id << endl;
            tempTransform = kinect_to_origin_[k];
            break;
        }
    }

    for(int i=0;i<15;i++)
    {
        tf::Vector3 p = transforms_[id][i].getOrigin();
        pos_[id][i][0] = p[0];
        pos_[id][i][1] = p[1];
        pos_[id][i][2] = p[2];

        pos_[id][i] = tempTransform * pos_[id][i]; //kinect_to_origin is the kinect frame transfor

    }
}

void JointListener::drawFrame(const Eigen::Affine3d &T)
{
    if( print_ )
    {
        cout << "draw Frame" << endl;
        cout << T.matrix() << endl;
    }

    OpenRAVE::GraphHandlePtr fig1,fig2,fig3;

    Eigen::Matrix3d m = T.linear();
    Eigen::Vector3d p = T.translation();

    OpenRAVE::Vector pos;
    OpenRAVE::Vector right,up,dir;

    pos.x = p(0); pos.y = p(1); pos.z = p(2);

    right.x = m(0,0); up.x = m(0,1); dir.x = m(0,2);
    right.y = m(1,0); up.y = m(1,1); dir.y = m(1,2);
    right.z = m(2,0); up.z = m(2,1); dir.z = m(2,2);

    fig1 = env_->drawarrow( pos, pos+0.5*right, 0.02, OpenRAVE::Vector(1,0,0,1));
    fig2 = env_->drawarrow( pos, pos+0.5*up,    0.02, OpenRAVE::Vector(0,1,0,1));
    fig3 = env_->drawarrow( pos, pos+0.5*dir,   0.02, OpenRAVE::Vector(0,0,1,1));

    graphptrs_.push_back( fig1 );
    graphptrs_.push_back( fig2 );
    graphptrs_.push_back( fig3 );
}

void JointListener::draw()
{
    if( tracked_user_id_.empty() )
        return;

    OpenRAVE::GraphHandlePtr figure;
    std::vector<OpenRAVE::RaveVector<float> > vpoints;
    std::vector<float> vcolors;

    for(int i=0; i<int(tracked_user_id_.size()); i++)
    {
        for(int j = 0; j<15; j++)
        {
            // float x = transforms_[ tracked_user_id_[i] ][j].getOrigin().getX();
            // float y = transforms_[ tracked_user_id_[i] ][j].getOrigin().getY();
            // float z = transforms_[ tracked_user_id_[i] ][j].getOrigin().getZ();

            //            if( j == 3 || j == 4 || j == 5 )
            //                continue;

            float x = pos_[tracked_user_id_[i]][j][0];
            float y = pos_[tracked_user_id_[i]][j][1];
            float z = pos_[tracked_user_id_[i]][j][2];

            OpenRAVE::RaveVector<float> pnt(x,y,z);
            vpoints.push_back(pnt);
            vcolors.push_back(1);
            vcolors.push_back(0);
            vcolors.push_back(0);
        }

        if( print_ )
            cout << "draw spheres for user : " <<  tracked_user_id_[i] << endl;
    }

    figure = env_->plot3( &vpoints[0].x, vpoints.size(), sizeof(vpoints[0]), 0.05, &vcolors[0], 1 );
    graphptrs_.push_back( figure );
}



*/
