#include "skeletonListener.hpp"
#include "skeletonDrawing.hpp"

#include <Eigen/Geometry>

template <typename T>
std::string num_to_string ( T Number )
{
    std::ostringstream ss;
    ss << Number;
    return ss.str();
}

template <class T>
bool string_to_num(T& t,
                   const std::string& s,
                   std::ios_base& (*f)(std::ios_base&))
{
    std::istringstream iss(s);
    return !(iss >> f >> t).fail();
}

using std::cout;
using std::endl;

Eigen::Vector3d or_vector_to_eigen(const OpenRAVE::Vector& pos)
{
    Eigen::Vector3d p;
    p[0] = pos.x;
    p[1] = pos.y;
    p[2] = pos.z;
    return p;
}

Eigen::Vector3d or_vector_to_eigen(const std::vector<double>& pos)
{
    Eigen::Vector3d p;
    p[0] = pos[0];
    p[1] = pos[1];
    p[2] = pos[2];
    return p;
}

std::vector<double> eigen_vector_to_or(const Eigen::Vector3d& pos)
{
    std::vector<double> p( 3 );
    p[0] = pos[0];
    p[1] = pos[1];
    p[2] = pos[2];
    return p;
}

double angle_limit_PI(double angle){

    while (angle < -M_PI){
        angle += 2*M_PI;
    }
    while (angle > M_PI){
        angle -= 2*M_PI;
    }
    return angle;
}

void print_config(const std::vector<double>& q)
{
    for( int i=0;i<int(q.size());i++)
    {
        cout << "q[" << i << "] = " << q[i] << endl;
    }
}

SkeletonListener::SkeletonListener(OpenRAVE::EnvironmentBasePtr penv)
{
    cout << "Enter constructer" << endl;

    env_ = penv;

    int argc = 0;
    char** argv;
    ros::init( argc, argv, "orkinect" );
    node_ = new ros::NodeHandle;
    //sub_ = node_->subscribe( "openni_tracker/openni_confidences", 1, &SkeletonListener::readConfidence, this );

    cout << "start suscriber" << endl;

    max_num_skel_ = 10;

    cout << "resize vectors" << endl;

    user_is_tracked_.resize(max_num_skel_);
    transforms_.resize(max_num_skel_);
    confidences_.resize(max_num_skel_);
    pos_.resize(max_num_skel_);

    for(int i=0;i<max_num_skel_;i++)
    {
        user_is_tracked_[i] = false;
        transforms_[i].resize(15);
        confidences_[i] = Eigen::VectorXd::Zero(15);
    }


    humans_.clear();

    OpenRAVE::RobotBasePtr human1 = env_->GetRobot( "human_model" );
    OpenRAVE::RobotBasePtr human2 = env_->GetRobot( "human_model_blue" );

    if( human1 != NULL ) {
        TrackedHuman h;
        h.user_name_ = "";
        h.robot_ = human1;
        h.id_kinect_ = 0;
        h.is_tracked_ = false;
        cout << "Add Human Robot : " << human1->GetName() << endl;
        humans_.push_back( h );
    }

    if( human2 != NULL ) {
        TrackedHuman h;
        h.user_name_ = "";
        h.robot_ = human2;
        h.id_kinect_ = 1;
        h.is_tracked_ = false;
        cout << "Add Human Robot : " << human2->GetName() << endl;
        humans_.push_back( h );
    }

    OpenRAVE::RaveTransform<float> Tcam;
    Tcam.rot.x = 0.420745;
    Tcam.rot.y = -0.574182;
    Tcam.rot.z = 0.605033;
    Tcam.rot.w = -0.356683;
    Tcam.trans.x = -2.61378;
    Tcam.trans.y = -0.0481125;
    Tcam.trans.z =  0.888456;

    // OpenRAVE::RaveTransformMatrix<float> T;
    // T.m[0] = -0.704743; T.m[1] = 0.304894; T.m[2] = -0.640607;  T.trans.x =  0; 1.11025;
    // T.m[4] = -0.310475; T.m[5] =-0.944433; T.m[6] = -0.10794;   T.trans.y = 0.112955;
    // T.m[8] = -0.637921; T.m[9] = 0.122822; T.m[10] =  0.760244; T.trans.z = -0.589495;

    env_->GetViewer()->SetCamera( Tcam );
    //env_->GetViewer()->setCamera( 0.262839, -0.733602, -0.623389, 0.0642694, 2.99336, -0.755646, 2.81558 );

    //setKinectFrame();

    printDofNames();

    set_joint_name_map();

    //    listen_iter_= 0;
    button_pressed_ = false;
    print_ = false;


}

void SkeletonListener::init_users()
{
    //    if (!custom_tracker_)
    //    {
    //        for (int id = 0; id < max_num_skel_; id++)
    //        {
    //            kin_user a_user;
    //            a_user.active = false;
    //            a_user.ident = num_to_string(id);

    //            active_users_.push_back(a_user);
    //            cout << "added user: " << a_user.ident << endl;
    //        }
    //    }
    //    else
    //    {
    //        for (int k = 1; k <= num_kinect_; k++)
    //        {
    //            for (int id = 0; id < max_num_skel_; id++)
    //            {
    //                kin_user a_user;
    //                a_user.active = false;
    //                a_user.ident = num_to_string(k) + "_" + num_to_string(id);

    //                active_users_.push_back(a_user);
    //                cout << "added user: " << a_user.ident << endl;
    //            }
    //        }
    //    }

    if (!custom_tracker_)
    {
        for (int id = 0; id < max_num_skel_; id++)
        {
            users_id_to_kinect_.push_back(0);
            users_.push_back(num_to_string(id));
            cout << "added user: " << users_[id] << endl;
        }
    }
    else
    {
        for (int k = 1; k <= num_kinect_; k++)
        {
            for (int id = 0; id < max_num_skel_; id++)
            {
                users_id_to_kinect_.push_back(k-1);
                users_.push_back(num_to_string(k) + "_" + num_to_string(id));
                cout << "Added user: " << num_to_string(k) + "_" + num_to_string(id) << endl;
            }
        }
    }
}

Eigen::Affine3d get_joint_transform(OpenRAVE::KinBody::JointPtr joint)
{
    OpenRAVE::RaveTransformMatrix<double> t( joint->GetFirstAttached()->GetTransform() );
    OpenRAVE::Vector right,up,dir,pos;
    t.Extract( right, up, dir, pos);
    Eigen::Matrix3d rot;
    rot(0,0) = right.x; rot(0,1) = up.x; rot(0,2) = dir.x;
    rot(1,0) = right.y; rot(1,1) = up.y; rot(1,2) = dir.y;
    rot(2,0) = right.z; rot(2,1) = up.z; rot(2,2) = dir.z;

    Eigen::Affine3d T;
    T.linear() = rot;
    T.translation() = or_vector_to_eigen( joint->GetAnchor() );

    //cout << "T : " << endl << T.matrix() << endl;
    return T;
}

void SkeletonListener::setNumKinect(int num)
{
    num_kinect_ = num;

    //if(custom_tracker_) //TODO move this.  I don't like redoing initialization here.  figure out another way.
    {
        transforms_.resize( max_num_skel_ * num_kinect_);
        pos_.resize( max_num_skel_ * num_kinect_);
        user_is_tracked_.resize(max_num_skel_ * num_kinect_);
        user_tracking_stopped_.resize(max_num_skel_ * num_kinect_, false);
        user_tracking_counter_.resize(max_num_skel_ * num_kinect_, 0);

        for(int i=0;i<max_num_skel_ * num_kinect_;i++)
        {
            transforms_[i].resize(15);
        }
    }

    kinect_to_origin_.resize(num_kinect_);

    init_users(); //Generates users_ vector.  TODO I don't know where this should go either.
}

void SkeletonListener::set_joint_name_map()
{
    cout << "set names" << endl;

    body_names_.clear();

    body_names_.push_back("head_");
    body_names_.push_back("neck_");
    body_names_.push_back("torso_");

    body_names_.push_back("left_shoulder_");
    body_names_.push_back("left_elbow_");
    body_names_.push_back("left_hand_");

    body_names_.push_back("right_shoulder_");
    body_names_.push_back("right_elbow_" );
    body_names_.push_back("right_hand_" );

    body_names_.push_back("left_hip_");
    body_names_.push_back("left_knee_");
    body_names_.push_back("left_foot_");

    body_names_.push_back("right_hip_");
    body_names_.push_back("right_knee_");
    body_names_.push_back("right_foot_");
}

void SkeletonListener::setRecord(bool buttonState)
{
    button_pressed_ = buttonState;
}

void SkeletonListener::listen_once()
{
    tracked_user_id_.clear();

    // For all users possibly published by tracker
    for (int id=0; id<int(users_.size()); id++ )
    {
        user_is_tracked_[id] = false;

        if( !listener_->frameExists("/head_"+ users_[id]))
        {
            //cout << "/head_"+ users_[id] + " does not exist" << endl;
            continue;
        }

        ros::Time previous_time = transforms_[id][0].stamp_;

        try
        {
            for (int j = 0; j < int(body_names_.size()); j++)
            {
                listener_->lookupTransform( "/openni_depth_frame", body_names_[j] + users_[id] , ros::Time(0), transforms_[id][j]);
            }

            user_is_tracked_[id] = true;
        }

        catch(tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
        }

        // Check stamp to see if we are sill tracking
        if( previous_time == transforms_[id][0].stamp_ )
        {
            if( !user_tracking_stopped_[id] )
            {
                user_tracking_counter_[id]++;
            }
            if( user_tracking_counter_[id] > 100 )
            {
                user_tracking_stopped_[id] = true;
            }
            if( user_tracking_stopped_[id] )
            {
                user_is_tracked_[id] = false;
            }
        }
        else {
            user_tracking_stopped_[id] = false;
            user_tracking_counter_[id] = 0;
        }

        if (user_is_tracked_[id] )
        {
            //cout << "user_is_tracked_ : " << id << endl;
            tracked_user_id_.push_back(id);
        }
    }

    for(int i=0; i<int(humans_.size()); i++)
    {
        //cout << user_is_tracked_[ humans_[i].id_user_ ] << endl;

        //Check that the human is still tracked
        if( humans_[i].is_tracked_ && (!user_is_tracked_[ humans_[i].id_user_ ]) )
        {
            humans_[i].is_tracked_ = false;
            cout << "human " << humans_[i].id_user_ << " is not tracked!!!!" << endl;
        }
    }

    for(int i=0; i < int(tracked_user_id_.size()); i++)
    {
        if(custom_tracker_)
        {
            if(users_id_to_kinect_[tracked_user_id_[i]] == 0)
            {
                if( !humans_[0].is_tracked_ )
                {
                    //cout << "0 is tracked" << endl;
                    humans_[0].is_tracked_ = true;
                    humans_[0].id_user_ = tracked_user_id_[i];
                    humans_[0].user_name_ = users_[humans_[0].id_user_];
                }

                // sets the pos vector (red spheres)
                setEigenPositions(tracked_user_id_[i]);

                if( humans_[0].is_tracked_ && (humans_[0].id_user_ == tracked_user_id_[i]))
                {
                    setHumanConfiguration( humans_[0].id_user_, humans_[0].robot_ );
                }
            }
            if(users_id_to_kinect_[tracked_user_id_[i]] == 1)
            {

                if( !humans_[1].is_tracked_ )
                {
                    //cout << "1 is tracked" << endl;
                    humans_[1].is_tracked_ = true;
                    humans_[1].id_user_ = tracked_user_id_[i];
                    humans_[1].user_name_ = users_[humans_[1].id_user_];
                }

                // sets the pos vector (red spheres)
                setEigenPositions(tracked_user_id_[i]);

                if( humans_[1].is_tracked_ && (humans_[1].id_user_ == tracked_user_id_[i]))
                {
                    setHumanConfiguration( humans_[1].id_user_, humans_[1].robot_ );
                }
            }
        }
        else
        {

            if( !(humans_[1].is_tracked_ && tracked_user_id_[i] == humans_[1].id_user_) )
            {
                if( !humans_[0].is_tracked_ )
                {
                    humans_[0].is_tracked_ = true;
                    humans_[0].id_user_ = tracked_user_id_[i];
                    humans_[0].user_name_ = users_[humans_[0].id_user_];
                }

                setEigenPositions(humans_[0].id_user_);

                if( humans_[0].is_tracked_ && (humans_[0].id_user_ == tracked_user_id_[i]))
                {
                    setHumanConfiguration( humans_[0].id_user_, humans_[0].robot_ );
                }
            }

            if( !(humans_[0].is_tracked_ && tracked_user_id_[i] == humans_[0].id_user_) )
            {

                if( !humans_[1].is_tracked_ )
                {
                    humans_[1].is_tracked_ = true;
                    humans_[1].id_user_ = tracked_user_id_[i];
                    humans_[1].user_name_ = users_[humans_[1].id_user_];
                }

                setEigenPositions(humans_[1].id_user_);

                if( humans_[1].is_tracked_ && (humans_[1].id_user_ == tracked_user_id_[i]))
                {
                    setHumanConfiguration( humans_[1].id_user_, humans_[1].robot_ );
                }
            }
        }
    }

    graphptrs_.clear();

    /*
    for(int i = 0; i < int(tracked_user_id_.size()); i++)
    {
        //cout << "set egien positions" << endl;
        setEigenPositions(tracked_user_id_[i]);

        //cout << "set configuration" << endl;
        if( !humans_.empty())
        {
            if( i < humans_.size() )
            {

                if(!custom_tracker_) {
                    setHumanConfiguration( tracked_user_id_[i], humans_[i].robot_ );
                }
                else {
                    //cout << tracked_user_id_[i] << "   " << humans_[users_id_to_kinect_[tracked_user_id_[i]]].robot_ << "   " << i << endl;
                    setHumanConfiguration( tracked_user_id_[i], humans_[users_id_to_kinect_[tracked_user_id_[i]]].robot_ );
                }

            }
        }
    }
    */

    if( !_motion_recorders.empty())
    {
        tryToRecord();
    }

    draw();

    if ( humans_.empty() && !tracked_user_id_.empty() )
    {
        for(int i = 0; i < int(tracked_user_id_.size()); i++)
        {
            SkeletonDrawing::drawLineSegmentModel(tracked_user_id_[i], env_, graphptrs_, pos_);
        }
    }

    ros::spinOnce();
    rate_->sleep();
}


void SkeletonListener::listen()
{
    //global_motionRecorder->setRobot(_strRobotName);
    rate_ = new ros::Rate(40.0);
    listener_ = new tf::TransformListener;

    while (node_->ok())
    {
        listen_once();
    }
}


/**
void SkeletonListener::setConfidence( std::string name, double conf )
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

void SkeletonListener::readConfidence(const openni_tracker::confidence_array& msg )
{
    //cout << "read confidence" << endl;

    for(int i=0;i<int(msg.confidence_array.size()); i++)
    {
        setConfidence( msg.confidence_array[i].child_frame_id , msg.confidence_array[i].confidence );
    }
}
**/



void SkeletonListener::tryToRecord()
{
    if( button_pressed_ )
    {
        for (int i = 0; i < int(_motion_recorders.size()); i++ ) {
            _motion_recorders[i]->m_is_recording = true;
            _motion_recorders[i]->saveCurrentConfig();
        }
    }
    else if (!button_pressed_)
    {
        for (int i = 0; i < int(_motion_recorders.size()); i++ ) {
            if (_motion_recorders[i]->m_is_recording) {
                _motion_recorders[i]->saveCurrentToCSV();
                _motion_recorders[i]->clearCurrentMotion();
                _motion_recorders[i]->m_is_recording = false;
            }
        }
    }
}

void SkeletonListener::printDofNames()
{
    if( humans_.empty() )
        return;

    const std::vector<OpenRAVE::KinBody::JointPtr> joints = humans_[0].robot_->GetJoints();

    for( int i=0;i<int(joints.size());i++)
    {
        cout << joints[i]->GetName() << " : " << joints[i]->GetDOFIndex() << endl;
    }
}

void SkeletonListener::setEigenPositions(int id)
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

void SkeletonListener::drawFrame(const Eigen::Affine3d &T)
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

void SkeletonListener::draw()
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

void SkeletonListener::setKinectFrame( int KinID, double TX, double TY, double TZ, double RotZ, double RotY )
{
    Eigen::Affine3d tempTranslation;

    RotZ = M_PI * RotZ / 180;
    RotY = M_PI * RotY / 180;

    Eigen::Affine3d T_Tra,T_Pan,T_Til;
    Eigen::Matrix3d kin_cam;

    //         Y  X                  Z  Y
    //         | /                   | /
    // Kinect  |/ ____ Z  ,   World  |/_____X

    kin_cam(0,0) = 1.0;  kin_cam(0,1) =  0.0;  kin_cam(0,2) = 0.0;
    kin_cam(1,0) = 0.0;  kin_cam(1,1) =  1.0;  kin_cam(1,2) = 0.0;
    kin_cam(2,0) = 0.0;  kin_cam(2,1) =  0.0;  kin_cam(2,2) = 1.0;

    //    kinect_to_origin_.linear() = kin_cam;
    //    kinect_to_origin_.translation() = Eigen::Vector3d::Zero();
    tempTranslation.linear() = kin_cam;
    tempTranslation.translation() = Eigen::Vector3d::Zero();

    T_Tra.linear() = Eigen::Matrix3d::Identity();
    T_Tra.translation() << TX , TY , TZ ;

    T_Pan.linear() = Eigen::Matrix3d( Eigen::AngleAxisd( RotZ, Eigen::Vector3d::UnitZ() ));
    T_Pan.translation() = Eigen::Vector3d::Zero();
    //    cout << "T_Pan : " << endl << T_Pan.matrix() << endl;

    T_Til.linear() = Eigen::Matrix3d( Eigen::AngleAxisd( RotY, Eigen::Vector3d::UnitY() ));
    T_Til.translation() = Eigen::Vector3d::Zero();
    //    cout << "T_Til : " << endl << T_Til.matrix() << endl;

    //    kinect_to_origin_ =  T_Tra * T_Pan * T_Til * kinect_to_origin_;
    kinect_to_origin_[KinID] = T_Tra * T_Pan * T_Til * tempTranslation;
    cout << "Set kinect frame : " << KinID << endl;
    //kinect_to_origin_.push_back();  //TODO fix this so it uses kinect id.
}

void SkeletonListener::setMotionRecorder(std::vector<HRICS::RecordMotion*> motion_recorder)
{
    _motion_recorders = motion_recorder;
}


//! @ingroup KINECT
//! Computes a condfiguration
//! from a set of points
void SkeletonListener::setHumanConfiguration(int id, OpenRAVE::RobotBasePtr human)
{
    if( human == NULL)
        return;

    if( print_ )
        cout << "Set Human Configuration" << endl;

    //setEigenPositions(id);

    // Set pelvis
    OpenRAVE::KinBody::JointPtr joint = human->GetJoint("PelvisTransX");
    int index_dof = joint->GetDOFIndex();

    std::vector<double> q;
    human->GetDOFValues(q);

    Eigen::Vector3d pos;

    if( print_ )
        cout << "Index dof is : " << index_dof << endl;

    //    if( data.TORSO.confidence > 0 && data.HIP_LEFT.confidence > 0 && data.HIP_RIGHT.confidence > 0 )
    //    {
    q[index_dof+0] = pos_[id][TORSO][0];
    q[index_dof+1] = pos_[id][TORSO][1];
    q[index_dof+2] = pos_[id][TORSO][2] - 0.20 ; // Hack 1 meter

    // calcul de l'orientation du pelvis
    Eigen::Vector3d sum, midP;
    sum = pos_[id][HIP_LEFT] + pos_[id][HIP_RIGHT];
    midP = 0.5*sum;

    q[index_dof+5] = atan2( pos_[id][HIP_LEFT][1]-midP[1] , pos_[id][HIP_LEFT][0]-midP[0]  );
    q[index_dof+5] -= M_PI/2; // Hack +  Pi / 2
    q[index_dof+5] = angle_limit_PI( q[index_dof+5] );

    //    cout << "HIP_LEFT : " << pos_[id][HIP_LEFT] << endl;
    //    cout << "HIP_RIGHT : " << pos_[id][HIP_RIGHT] << endl;

    //    cout << " q[0] : " << q[0] << endl;
    //    cout << " q[1] : " << q[1] << endl;
    //    cout << " q[2] : " << q[2] << endl;
    //    cout << " q[3] : " << q[3] << endl;
    //    cout << " q[4] : " << q[4] << endl;
    //    cout << " q[5] : " << q[5] << endl;

    human->SetJointValues(q);
    //return;
    //    }

    //--------------------------------------------------------------
    Eigen::Affine3d Tinv,Tpelv;
    Eigen::Vector3d shoulder;
    //Eigen::Vector3d Xaxis; Xaxis << 1 , 0 , 0 ;
    Eigen::Affine3d TrotX,TrotXtmp;
    //Eigen::Vector3d Yaxis; Yaxis << 0 , 1 , 0 ;
    Eigen::Affine3d TrotY,TrotYtmp;

    joint = human->GetJoint("TorsoX");

    Tpelv = get_joint_transform( joint );
    drawFrame( Tpelv );
    //m_absPos = get_joint_transform( joint );

    Tinv = Tpelv.inverse();
    pos = Tinv*pos_[id][NECK];

    double TorsoX = atan2( -pos[1] , pos[2] );
    index_dof = human->GetJoint("TorsoX")->GetDOFIndex();
    q[index_dof] = TorsoX;

    TrotX.linear() = Eigen::Matrix3d( Eigen::AngleAxisd( TorsoX, Eigen::Vector3d::UnitX() ));
    TrotX.translation() = Eigen::Vector3d::Zero();
    TrotXtmp = Tpelv * TrotX;
    Tinv = TrotXtmp.inverse();
    pos = Tinv*pos_[id][NECK];

    double TorsoY = atan2( pos[0] , pos[2] );
    index_dof = human->GetJoint("TorsoY")->GetDOFIndex();
    q[index_dof] = TorsoY;

    TrotY.linear() = Eigen::Matrix3d( Eigen::AngleAxisd( TorsoY, Eigen::Vector3d::UnitY() ));
    TrotY.translation() = Eigen::Vector3d::Zero();
    TrotYtmp = TrotXtmp * TrotY;
    Tinv = TrotYtmp.inverse();
    pos = Tinv*pos_[id][SHOULDER_LEFT];

    double TorsoZ = atan2( -pos[0] , pos[1] );
    index_dof = human->GetJoint("TorsoZ")->GetDOFIndex();
    q[index_dof] = TorsoZ;

    human->SetJointValues(q);
    //return;
    //    }

    // -----------------------------------------------------------------
    // Set and update robot to
    // new position
    // -----------------------------------------------------------------
    Eigen::Affine3d Trot;
    Eigen::Affine3d Trot2,Trot3;
    Eigen::Affine3d Trot4;
    Eigen::Vector3d vect1,vect2,vect3;

    //    if( data.ELBOW_RIGHT.confidence > 0 && data.HAND_RIGHT.confidence > 0 )
    //    {
    human->SetJointValues(q);

    if( print_ )
        cout << "Set human torso configuration" << endl;

    joint = human->GetJoint("rShoulderX");
    Eigen::Affine3d T = get_joint_transform( joint );
    pos = T.translation();
    shoulder = T.translation();

    index_dof = human->GetJoint("rArmTrans")->GetDOFIndex();
    q[index_dof] = ( pos_[id][ELBOW_RIGHT] - pos ).norm() - 0.2066 ;

    joint = human->GetJoint("TorsoZ");

    Trot = get_joint_transform( joint );
    Trot.translation() = pos;

    Tinv = Trot.inverse();
    pos = Tinv*pos_[id][ELBOW_RIGHT];

    // calcul de la direction pour le bras droit
    Eigen::Vector3d dir,sub;
    dir = pos / pos.norm();

    // JP
    // selon x
    double alpha1r = atan2( -pos[2] , -pos[1] );

    if( print_ )
        cout << "get joint rShoulderX" << endl;
    index_dof = human->GetJoint("rShoulderX")->GetDOFIndex();
    q[index_dof] = alpha1r;

    if( print_ )
        cout << "alpha1r : " << alpha1r << endl;

    Trot2.linear() = Eigen::Matrix3d( Eigen::AngleAxisd( alpha1r, Eigen::Vector3d::UnitX() ));
    Trot2.translation() = Eigen::Vector3d::Zero();

    Trot3 = Trot*Trot2;
    Tinv = Trot3.inverse();
    pos = Tinv*pos_[id][ELBOW_RIGHT];

    // selon z
    double alpha2r = atan2( pos[0] , -pos[1] );

    if( print_ )
        cout << "get joint rShoulderZ" << endl;
    index_dof =  human->GetJoint("rShoulderZ")->GetDOFIndex();
    q[index_dof] = alpha2r;

    if( print_ )
        cout << "alpha2r : " << alpha2r << endl;

    Trot2.linear() = Eigen::Matrix3d( Eigen::AngleAxisd( alpha2r, Eigen::Vector3d::UnitZ() ));
    Trot2.translation() = Eigen::Vector3d::Zero();
    Trot4 = Trot3*Trot2;
    Tinv = Trot4.inverse();
    pos = Tinv*pos_[id][HAND_RIGHT];

    // selon y
    double alpha3r = atan2( pos[2], pos[0] );

    if( print_ )
        cout << "get joint rShoulderY" << endl;
    index_dof = human->GetJoint("rShoulderY")->GetDOFIndex();
    q[index_dof] = alpha3r;

    if( print_ )
        cout << "alpha3r : " << alpha3r << endl;

    vect1 = shoulder - pos_[id][ELBOW_RIGHT];
    vect1.normalize();

    vect2 = pos_[id][HAND_RIGHT] - pos_[id][ELBOW_RIGHT];
    vect2.normalize();

    // Elbow
    double alpha4r = M_PI - acos( vect1.dot(vect2) ) ;

    if( print_ )
        cout << "get joint rElbowZ" << endl;
    index_dof = human->GetJoint("rElbowZ")->GetDOFIndex();
    q[index_dof] = alpha4r;

    if( print_ )
        cout << "alpha4r : " << alpha4r << endl;

    if( print_ )
        print_config(q);

    human->SetJointValues(q);

    if( print_ )
        cout << "Set human right arm configuration" << endl;
    //        /return;
    //    }
    //---------------------------------------------------------------------------
    //---------------------------------------------------------------------------


    joint = human->GetJoint("lShoulderX");
    T = get_joint_transform( joint );
    pos = T.translation();
    shoulder = T.translation();

    index_dof = human->GetJoint("lArmTrans")->GetDOFIndex();
    q[index_dof] = ( pos_[id][ELBOW_LEFT] - pos ).norm() - 0.2066 ;

    joint = human->GetJoint("TorsoZ");

    Trot = get_joint_transform( joint );
    Trot.translation() = pos;

    //  p3d_mat4Copy( Trot , m_absPos );

    Tinv = Trot.inverse();
    pos = Tinv*pos_[id][ELBOW_LEFT];

    // calcul de la direction pour le bras droit
    //Eigen::Vector3d dir,sub;
    dir = pos / pos.norm();

    // JP //********************************************
    // selon x
    double alpha1l = atan2( pos[2] , pos[1] );

    index_dof = human->GetJoint("lShoulderX")->GetDOFIndex();
    q[index_dof] = alpha1l;

    Trot2.linear() = Eigen::Matrix3d(Eigen::AngleAxisd( alpha1l, Eigen::Vector3d::UnitX() ));
    Trot2.translation() = Eigen::Vector3d::Zero();
    Trot3 = Trot*Trot2;
    Tinv = Trot3.inverse();
    pos = Tinv*pos_[id][ELBOW_LEFT];

    // selon z
    double alpha2l = atan2( -pos[0] , pos[1] );

    index_dof = human->GetJoint("lShoulderZ")->GetDOFIndex();
    q[index_dof] = alpha2l;

    Trot2.linear() = Eigen::Matrix3d(Eigen::AngleAxisd( alpha2l, Eigen::Vector3d::UnitZ() ));
    Trot2.translation() = Eigen::Vector3d::Zero();
    Trot4 = Trot3*Trot2;
    Tinv = Trot4.inverse();
    pos = Tinv*pos_[id][HAND_LEFT];

    // selon y
    double alpha3l = -atan2( -pos[2], pos[0] );

    index_dof = human->GetJoint("lShoulderY")->GetDOFIndex();
    q[index_dof] = alpha3l;

    vect1 = shoulder - pos_[id][ELBOW_LEFT];
    vect1.normalize();

    vect2 = pos_[id][HAND_LEFT] - pos_[id][ELBOW_LEFT];
    vect2.normalize();

    // Elbow
    double alpha4l = -M_PI + acos( vect1.dot(vect2) ) ;

    index_dof = human->GetJoint("lElbowZ")->GetDOFIndex();
    q[index_dof] = alpha4l;

    //    }

    human->SetJointValues(q);
}
