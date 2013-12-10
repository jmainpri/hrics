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
    sub_ = nh_.subscribe("human_state", 10, &JointListener::listen_cb, this );
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("class_markers", 1);

    rest_state_ = 0;
    human_ = env_->GetRobot( "human_model" );
    motion_recorder_ = new HRICS::RecordMotion(human_);
    motion_recorder_->setRobotId(0);
    motion_recorder_->setBuffSize(200);
    motion_recorder_->setNumKeep(100);
    m_classifier_ = new HRICS::ClassifyMotion();
    motion_started_ = false;

    if (m_classifier_->load_model(6))
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
    std::vector<double> likelihood;

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
            motion_recorder_->saveCurrentToCSV();
        }
        else if(resting)
        {
            //cout << "We're resting" << endl;
            motion_started_ = false;
            getRestingRot();
        }

        if (motion_started_)
        {
            //cout << "We got a motion!" << endl;
//            motion_recorder_->saveCurrentToCSV();
            motion_t m = motion_recorder_->getCurrentMotion();
            m = motion_recorder_->fixPelvisFrame(m);
            likelihood = classifyMotion( m );
        }
    }

    //graphptrs_.clear();
    draw_classes(likelihood);

}

void JointListener::listen()
{
    rate_ = new ros::Rate(40.0);

    cout << "loading classes" << endl;
    load_classes();

    while (nh_.ok())
    {
        rate_->sleep();
        ros::spinOnce();
    }

//    motion_t m = motion_recorder_->loadFromCSV("/home/rafi/Desktop/classes/class_1.csv");
//    //m = motion_recorder_->fixPelvisFrame(m);
//    int m_class = classifyMotion( m );
//    cout << "class: " << m_class << endl;

//    m = motion_recorder_->loadFromCSV("/home/rafi/Desktop/classes/class_2.csv");
//    //m = motion_recorder_->fixPelvisFrame(m);
//    m_class = classifyMotion( m );
//    cout << "class: " << m_class << endl;

//     m = motion_recorder_->loadFromCSV("/home/rafi/Desktop/classes/class_3.csv");
//    //m = motion_recorder_->fixPelvisFrame(m);
//     m_class = classifyMotion( m );
//    cout << "class: " << m_class << endl;

//     m = motion_recorder_->loadFromCSV("/home/rafi/Desktop/classes/class_4.csv");
//    //m = motion_recorder_->fixPelvisFrame(m);
//     m_class = classifyMotion( m );
//    cout << "class: " << m_class << endl;

//     m = motion_recorder_->loadFromCSV("/home/rafi/Desktop/classes/class_5.csv");
//    //m = motion_recorder_->fixPelvisFrame(m);
//     m_class = classifyMotion( m );
//    cout << "class: " << m_class << endl;

//     m = motion_recorder_->loadFromCSV("/home/rafi/Desktop/classes/class_6.csv");
//    //m = motion_recorder_->fixPelvisFrame(m);
//     m_class = classifyMotion( m );
//    cout << "class: " << m_class << endl;

//     m = motion_recorder_->loadFromCSV("/home/rafi/Desktop/classes/class_7.csv");
//    //m = motion_recorder_->fixPelvisFrame(m);
//     m_class = classifyMotion( m );
//    cout << "class: " << m_class << endl;

//     m = motion_recorder_->loadFromCSV("/home/rafi/Desktop/classes/class_8.csv");
//    //m = motion_recorder_->fixPelvisFrame(m);
//     m_class = classifyMotion( m );
//    cout << "class: " << m_class << endl;
}

//void JointListener::classifyLibrary()
//{
//    std::ofstream s;
//    s.open( "/home/rafi/Desktop/JimsLib/classification.csv" );

//    for (int i = 0; i < 8; i++)
//    {
//        int correct_count = 0;

//        for (int j = 0; j < 25; j++)
//        {

//            if ( !(m_classifier_->load_model(j+1)) )
//            {
//                cout << "Couldn't initialize classifier" << endl;
//                return;
//            }

//            std::ostringstream filename;
//            filename << "/home/rafi/Desktop/JimsLib/motion_saved_00000_";

//            filename << std::setfill('0') << std::setw(5) << (j*8)+i << ".csv";

//            motion_t m = motion_recorder_->loadFromCSV(filename.str());
//            m = motion_recorder_->fixPelvisFrame(m);
//            int m_class = classifyMotion( m );
//            if (m_class == i)
//                correct_count++;

//            s << m_class << ",";
//        }
//        s << correct_count;
//        s << endl;
//    }

//    s.close();

//}

bool JointListener::checkRestingPos(double offset)
{
    if (offset > .2)
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
    //cout << "p: " << p[0] << " " << p[1] << " " << p[2] << endl;

    Eigen::Vector3d pInitial( 0.10, 0.20, -0.1 );  //TODO recalculate ideal initial resting offset
    double offset = (p-pInitial).norm();

    return offset;
}

//Gets the rotation matrix of the torso while resting.
void JointListener::getRestingRot()
{
    Eigen::Affine3d Torso = get_joint_transform(human_->GetJoint("TorsoZ"));

    Eigen::Matrix3d rot;
//    rot(0,0) = -1; rot(0,1) = 0; rot(0,2) = 0; //180deg
//    rot(1,0) = 0; rot(1,1) = -1; rot(1,2) = 0;
//    rot(2,0) = 0; rot(2,1) = 0; rot(2,2) = 1;

    rot(0,0) = -0.8660254; rot(0,1) = -0.5; rot(0,2) = 0; // 5pi/6
    rot(1,0) = 0.5; rot(1,1) = -0.8660254; rot(1,2) = 0;
    rot(2,0) = 0; rot(2,1) = 0; rot(2,2) = 1;

    rest_rot_.linear() = rot*Torso.linear();

    updated_offsets_ = update_classes();
}

std::vector<double> JointListener::classifyMotion( const motion_t& motion )
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

    //return std::max_element(likelihood.begin(),likelihood.end()) - likelihood.begin();
    return likelihood;
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

void JointListener::load_classes()
{

    confPtr_t q;
    human_->GetDOFValues( q ); //Current Position


    motion_t temp_m = motion_recorder_->loadFromCSV("/home/rafi/workspace/gmm-gmr-true/data/gestures/8classes_fixed/traj_classes/class_0.csv");
    human_->SetJointValues( temp_m[55].second ); //Around the final point for the class
    Eigen::Affine3d p = get_joint_transform( human_->GetJoint("rWristX") );
    Eigen::Affine3d T = get_joint_transform(human_->GetJoint("PelvisRotZ"));
    class_offsets_.push_back(T.inverse()*p);


    temp_m = motion_recorder_->loadFromCSV("/home/rafi/workspace/gmm-gmr-true/data/gestures/8classes_fixed/traj_classes/class_1.csv");
    human_->SetJointValues( temp_m[55].second ); //Around the final point for the class
    p = get_joint_transform( human_->GetJoint("rWristX") );
    T = get_joint_transform(human_->GetJoint("PelvisRotZ"));
    class_offsets_.push_back(T.inverse()*p);


    temp_m = motion_recorder_->loadFromCSV("/home/rafi/workspace/gmm-gmr-true/data/gestures/8classes_fixed/traj_classes/class_2.csv");
    human_->SetJointValues( temp_m[55].second ); //Around the final point for the class
    p = get_joint_transform( human_->GetJoint("rWristX") );
    T = get_joint_transform(human_->GetJoint("PelvisRotZ"));
    class_offsets_.push_back(T.inverse()*p);


    temp_m = motion_recorder_->loadFromCSV("/home/rafi/workspace/gmm-gmr-true/data/gestures/8classes_fixed/traj_classes/class_3.csv");
    human_->SetJointValues( temp_m[55].second ); //Around the final point for the class
    p = get_joint_transform( human_->GetJoint("rWristX") );
   T = get_joint_transform(human_->GetJoint("PelvisRotZ"));
    class_offsets_.push_back(T.inverse()*p);


    temp_m = motion_recorder_->loadFromCSV("/home/rafi/workspace/gmm-gmr-true/data/gestures/8classes_fixed/traj_classes/class_4.csv");
    human_->SetJointValues( temp_m[58].second ); //Around the final point for the class
    p = get_joint_transform( human_->GetJoint("rWristX") );
    T = get_joint_transform(human_->GetJoint("PelvisRotZ"));
    class_offsets_.push_back(T.inverse()*p);

    temp_m = motion_recorder_->loadFromCSV("/home/rafi/workspace/gmm-gmr-true/data/gestures/8classes_fixed/traj_classes/class_5.csv");
    human_->SetJointValues( temp_m[58].second ); //Around the final point for the class
    p = get_joint_transform( human_->GetJoint("rWristX") );
    T = get_joint_transform(human_->GetJoint("PelvisRotZ"));
    class_offsets_.push_back(T.inverse()*p);


    temp_m = motion_recorder_->loadFromCSV("/home/rafi/workspace/gmm-gmr-true/data/gestures/8classes_fixed/traj_classes/class_6.csv");
    human_->SetJointValues( temp_m[58].second ); //Around the final point for the class
    p = get_joint_transform( human_->GetJoint("rWristX") );
    T = get_joint_transform(human_->GetJoint("PelvisRotZ"));
    class_offsets_.push_back(T.inverse()*p);


    temp_m = motion_recorder_->loadFromCSV("/home/rafi/workspace/gmm-gmr-true/data/gestures/8classes_fixed/traj_classes/class_7.csv");
    human_->SetJointValues( temp_m[60].second ); //Around the final point for the class
    p = get_joint_transform( human_->GetJoint("rWristX") );
    T = get_joint_transform(human_->GetJoint("PelvisRotZ"));
    class_offsets_.push_back(T.inverse()*p);


    human_->SetJointValues(q); //Restore human state
}

std::vector<Eigen::Affine3d> JointListener::update_classes()
{
    std::vector<Eigen::Affine3d> ret;
    ret.resize(8);

    Eigen::Affine3d Torso = get_joint_transform(human_->GetJoint("TorsoZ"));
    Eigen::Affine3d Pelv = get_joint_transform(human_->GetJoint("PelvisRotZ"));

    for (int i = 0; i < 8; i++)
    {
        Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

//        cout << "\n\n class: " << i << endl;
//        cout << "torso wrist:" << class_offsets_[i].rotation().format(CleanFmt) << endl;
//        cout << "world torso:" << T.rotation().format(CleanFmt) << endl;

//        Eigen::Matrix3d rot;
//        rot(0,0) = -1; rot(0,1) = 0; rot(0,2) = 0;
//        rot(1,0) = 0; rot(1,1) = -1; rot(1,2) = 0;
//        rot(2,0) = 0; rot(2,1) = 0; rot(2,2) = 1;

        Eigen::Affine3d T;
        T.linear() = rest_rot_.linear();
        T.translation() = Pelv.translation();


        ret[i] = T*class_offsets_[i]; //Works with base frame
        //cout << "world wrist" << ret[i].rotation().format(CleanFmt) << endl;
    }

    return ret;
}


//void JointListener::draw_classes(std::vector<double> likelihood) //Openrave markers
//{

//    std::vector<Eigen::Affine3d> offsets = update_classes();

//    double mostLikely = std::max_element(likelihood.begin(),likelihood.end()) - likelihood.begin();

//    OpenRAVE::GraphHandlePtr figure;
//    std::vector<OpenRAVE::RaveVector<float> > vpoints;
//    std::vector<float> vcolors;

//    //std::vector< std::vector<Eigen::Vector3d> > pos_;
//    //
//    if (likelihood.size() == 8)
//    {
//        for(int c = 0; c<8; c++)
//        {

//            float x = offsets[c].translation()[0];
//            float y = offsets[c].translation()[1];
//            float z = offsets[c].translation()[2];

//            OpenRAVE::RaveVector<float> pnt(x,y,z);
//            vpoints.push_back(pnt);
//            if (c == mostLikely)
//            {
//                vcolors.push_back(0);
//                vcolors.push_back(1);
//                vcolors.push_back(0);
//            }
//            else if ( std::abs(likelihood[c]-likelihood[mostLikely]) <= 100 && c != mostLikely )
//            {
//                vcolors.push_back(1);
//                vcolors.push_back(1);
//                vcolors.push_back(0);
//            }
//            else if ( std::abs(likelihood[c]-likelihood[mostLikely]) <= 150 && c != mostLikely )
//            {
//                vcolors.push_back(1);
//                vcolors.push_back(0);
//                vcolors.push_back(0);
//            }
//            else
//            {
//                vcolors.push_back(0.66);
//                vcolors.push_back(0.66);
//                vcolors.push_back(0.66);
//            }
//        }
//    }
//    else
//    {
//        for(int c = 0; c < 8; c++)
//        {
//            float x = offsets[c].translation()[0];
//            float y = offsets[c].translation()[1];
//            float z = offsets[c].translation()[2];

//            OpenRAVE::RaveVector<float> pnt(x,y,z);
//            vpoints.push_back(pnt);
//            vcolors.push_back(0.66);
//            vcolors.push_back(0.66);
//            vcolors.push_back(0.66);
//        }
//    }

//    figure = env_->plot3( &vpoints[0].x, vpoints.size(), sizeof(vpoints[0]), 0.07, &vcolors[0], 1 );
//    graphptrs_.push_back( figure );
//}

double JointListener::likelihood_to_range(double x)
{
    double from_min = -708.396;
    double from_max = 0;
    double to_min = 0;
    double to_max = 1;
    x = (x - from_min) * (to_max - to_min) / (from_max - from_min) + to_min;
    return x;
}


void JointListener::draw_classes(std::vector<double> likelihood)
{

    std::vector<visualization_msgs::Marker> markers;
    visualization_msgs::MarkerArray c_array = visualization_msgs::MarkerArray();

    double mostLikely = std::max_element(likelihood.begin(),likelihood.end()) - likelihood.begin();
    double leastLikely = std::min_element(likelihood.begin(),likelihood.end()) - likelihood.begin();

    for(int c = 0; c < 8; c++)
    {

//        float x = offsets[c].translation()[0];
//        float y = offsets[c].translation()[1];
//        float z = offsets[c].translation()[2];


//        float x = class_offsets_[c].translation()[0];
//        float y = class_offsets_[c].translation()[1];
//        float z = class_offsets_[c].translation()[2];

//        Eigen::Vector3d p = class_offsets_[c].translation();
        if(updated_offsets_.size() != 8)
            continue;
        Eigen::Vector3d p = updated_offsets_[c].translation();
        //Eigen::Quaterniond q = (Eigen::Quaterniond)offsets[c].rotation();

        //Rotate the point with the quaternion




        visualization_msgs::Marker marker;
        marker.header.frame_id = "/base"; //PelvisDummyTransZ proper spot, no rotation//TorsoDummyX okay
        marker.header.stamp = ros::Time::now();
        marker.ns = "motion_class";
        marker.id = c;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.pose.position.x = p.x();
        marker.pose.position.y = p.y();
        marker.pose.position.z = p.z();
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;
        marker.scale.x = .1;
        marker.scale.y = .1;
        marker.scale.z = .1;

        if(likelihood.size() != 0)
        {
            double val = likelihood_to_range(likelihood[c]);
            marker.color.r = 1-val;
            marker.color.g = val;
            marker.color.b = 0;
            marker.color.a = 1; //Alpha
        }
        else
        {
            marker.color.r = 0.66;
            marker.color.g = 0.66;
            marker.color.b = 0.66;
            marker.color.a = 1; //Alpha
        }
        c_array.markers.push_back(marker);

    }
    marker_pub_.publish(c_array);


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
