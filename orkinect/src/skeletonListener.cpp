#include "skeletonListener.hpp"
#include "skeletonDrawing.hpp"

#include <Eigen/Geometry>

using std::cout;
using std::endl;

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

    for(int i=0;i<max_num_skel_;i++)
    {
        user_is_tracked_[i] = false;
        transforms_[i].resize(15);
        confidences_[i] = Eigen::VectorXd::Zero(15);
    }

    human_ = env_->GetRobot( "human_model" );

    if( human_ )
        cout << "Set Human Robot : " << human_->GetName() << endl;

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

    setKinectFrame();

    printDofNames();

    set_joint_name_map();

    print_ = false;
}

void SkeletonListener::set_joint_name_map()
{
    cout << "set names" << endl;

    names_.clear();

    names_.push_back("head_");
    names_.push_back("neck_");
    names_.push_back("torso_");

    names_.push_back("left_shoulder_");
    names_.push_back("left_elbow_");
    names_.push_back("left_hand_");

    names_.push_back("right_shoulder_");
    names_.push_back("right_elbow_" );
    names_.push_back("right_hand_" );

    names_.push_back("left_hip_");
    names_.push_back("left_knee_");
    names_.push_back("left_foot_");

    names_.push_back("right_hip_");
    names_.push_back("right_knee_");
    names_.push_back("right_foot_");
}

void SkeletonListener::listen()
{
    ros::Rate rate(10.0);

    tf::TransformListener listener;

    while (node_->ok())
    {
        tracked_user_id_.clear();

        for(int i=0;i<10;i++)
        {
            user_is_tracked_[i] = false;

            if( !listener.frameExists("/head_" + num_to_string(i)))
                continue;

            if( print_ )
                cout << "listening to user " << i << endl;

            ros::Time previous_time = transforms_[i][0].stamp_;

            try
            {
                for(int j=0;j<int(names_.size()); j++)
                {
                    //if( print_ )
                    //    cout << "listent to pair (" << i << " , " << j << ")" << endl;

                    listener.lookupTransform( "/openni_depth_frame", names_[j] + num_to_string(i) , ros::Time(0), transforms_[i][j]);
                }

                user_is_tracked_[i] = true;
            }
            catch (tf::TransformException ex){
                //ROS_ERROR("%s",ex.what());
            }

            if( previous_time == transforms_[i][0].stamp_ )
            {
                user_is_tracked_[i] = false;
            }

            if( user_is_tracked_[i] )
            {
                tracked_user_id_.push_back( i );

                if( print_ )
                    cout << "tracking id : " << i << endl;
            }

            //listener_.lookupTransform("/openni_depth_frame", "/turtle1", ros::Time(0), transform);
        }

        //cout << "Camera : " << env_->GetViewer()->GetCameraTransform() << endl;

        graphptrs_.clear();

        for(int i=0; i<int(tracked_user_id_.size()); i++)
        {
            setEigenPositions( tracked_user_id_[i] );

            if( human_ )
                setHumanfiguration( tracked_user_id_[i] );
        }

        draw();

        if( human_ == NULL && !tracked_user_id_.empty() )
            SkeletonDrawing::drawLineSegmentModel( env_, graphptrs_, pos_ );

        ros::spinOnce();
        rate.sleep();
    }
}

/**
void SkeletonListener::setConfidence( std::string name, double conf )
{
    //cout << "set : " << name << endl;

    for(int i=0;i<int(names_.size());i++)
    {
        if( name.find( names_[i] ) != std::string::npos )
        {
            std::string skel_id = name.substr(names_[i].size());
            //cout << "--" << name.substr(names_[i].size()) << "--" << endl;
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

void SkeletonListener::printDofNames()
{
    if( human_ == NULL )
        return;

    const std::vector<OpenRAVE::KinBody::JointPtr> joints = human_->GetJoints();

    for( int i=0;i<int(joints.size());i++)
    {
        cout << joints[i]->GetName() << " : " << joints[i]->GetDOFIndex() << endl;
    }
}

void SkeletonListener::setEigenPositions(int id)
{
    pos_.clear();
    pos_.resize(15);
    for(int i=0;i<15;i++)
    {
        tf::Vector3 p = transforms_[id][i].getOrigin();
        pos_[i][0] = p[0];
        pos_[i][1] = p[1];
        pos_[i][2] = p[2];

        pos_[i] = kinect_to_origin_ * pos_[i];
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

            float x = pos_[j][0];
            float y = pos_[j][1];
            float z = pos_[j][2];

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

void SkeletonListener::setKinectFrame()
{
    Eigen::Affine3d T_Tra,T_Pan,T_Til;
    Eigen::Matrix3d kin_cam;

    double TX = 0; double TY = 0; double TZ = 0;
    double RotZ = 0; double RotY = 0;

    //         Y  X                  Z  Y
    //         | /                   | /
    // Kinect  |/ ____ Z  ,   World  |/_____X

    //  kin_cam(0,0) = 0.0;  kin_cam(0,1) = 0.0;  kin_cam(0,2) = 1.0;
    //  kin_cam(1,0) = 1.0;  kin_cam(1,1) = 0.0;  kin_cam(1,2) = 0.0;
    //  kin_cam(2,0) = 0.0;  kin_cam(2,1) = 1.0;  kin_cam(2,2) = 0.0;

    kin_cam(0,0) = 1.0;  kin_cam(0,1) =  0.0;  kin_cam(0,2) = 0.0;
    kin_cam(1,0) = 0.0;  kin_cam(1,1) = -1.0;  kin_cam(1,2) = 0.0;
    kin_cam(2,0) = 0.0;  kin_cam(2,1) =  0.0;  kin_cam(2,2) = 1.0;

    kinect_to_origin_.linear() = kin_cam;
    kinect_to_origin_.translation() = Eigen::Vector3d::Zero();

    T_Tra.linear() = Eigen::Matrix3d::Identity();
    T_Tra.translation() << TX , TY , TZ ;

    T_Pan.linear() = Eigen::Matrix3d( Eigen::AngleAxisd( RotZ, Eigen::Vector3d::UnitZ() ));
    T_Pan.translation() = Eigen::Vector3d::Zero();
    cout << "T_Pan : " << endl << T_Pan.matrix() << endl;

    T_Til.linear() = Eigen::Matrix3d( Eigen::AngleAxisd( RotY, Eigen::Vector3d::UnitY() ));
    T_Til.translation() = Eigen::Vector3d::Zero();
    cout << "T_Til : " << endl << T_Til.matrix() << endl;

    kinect_to_origin_ =  T_Tra * T_Pan * T_Til * kinect_to_origin_;

    //    Eigen::AngleAxisd( TorsoX, Eigen::Vector3d::UnitX() ));

    //    p3d_mat4Pos( T_Tra, mKinectTX, mKinectTY, mKinectTZ, 0, 0, 0 );
    //    p3d_mat4Pos( T_Pan, 0, 0, 0, 0, 0, mKinectRotZ );
    //    p3d_mat4Pos( T_Til, 0, 0, 0, 0, mKinectRotY, 0 );

    //    // Trans * Pan * Tilt * Cam = KinectToOrigin
    //    p3d_mat4Mult( T_Tra , T_Pan , tmp1 );
    //    p3d_mat4Mult( tmp1, T_Til, tmp2 );
    //    p3d_mat4Mult( tmp2, mKinectToOrigin, tmp1);
    //    p3d_mat4Copy( tmp1, mKinectToOrigin );
}

//! @ingroup KINECT
//! Computes a condfiguration
//! from a set of points
void SkeletonListener::setHumanConfiguration(int id)
{
    if( human_ == NULL)
        return;

    if( print_ )
        cout << "Set Human Configuration" << endl;

    //setEigenPositions(id);

    // Set pelvis
    OpenRAVE::KinBody::JointPtr joint = human_->GetJoint("PelvisTransX");
    int index_dof = joint->GetDOFIndex();

    std::vector<double> q;
    human_->GetDOFValues(q);

    Eigen::Vector3d pos;

    if( print_ )
        cout << "Index dof is : " << index_dof << endl;

    //    if( data.TORSO.confidence > 0 && data.HIP_LEFT.confidence > 0 && data.HIP_RIGHT.confidence > 0 )
    //    {
    q[index_dof+0] = pos_[TORSO][0];
    q[index_dof+1] = pos_[TORSO][1];
    q[index_dof+2] = pos_[TORSO][2] - 0.20 ; // Hack 1 meter

    // calcul de l'orientation du pelvis
    Eigen::Vector3d sum, midP;
    sum = pos_[HIP_LEFT] + pos_[HIP_RIGHT];
    midP = 0.5*sum;

    q[index_dof+5] = atan2( pos_[HIP_LEFT][1]-midP[1] , pos_[HIP_LEFT][0]-midP[0]  );
    q[index_dof+5] -= M_PI/2; // Hack +  Pi / 2
    q[index_dof+5] = angle_limit_PI( q[index_dof+5] );

    cout << "HIP_LEFT : " << pos_[HIP_LEFT] << endl;
    cout << "HIP_RIGHT : " << pos_[HIP_RIGHT] << endl;

    //    cout << " q[0] : " << q[0] << endl;
    //    cout << " q[1] : " << q[1] << endl;
    //    cout << " q[2] : " << q[2] << endl;
    //    cout << " q[3] : " << q[3] << endl;
    //    cout << " q[4] : " << q[4] << endl;
    //    cout << " q[5] : " << q[5] << endl;

    human_->SetJointValues(q);
    //return;
    //    }

    //--------------------------------------------------------------
    Eigen::Affine3d Tinv,Tpelv;
    Eigen::Vector3d shoulder;
    //Eigen::Vector3d Xaxis; Xaxis << 1 , 0 , 0 ;
    Eigen::Affine3d TrotX,TrotXtmp;
    //Eigen::Vector3d Yaxis; Yaxis << 0 , 1 , 0 ;
    Eigen::Affine3d TrotY,TrotYtmp;

    joint = human_->GetJoint("TorsoX");

    Tpelv = get_joint_transform( joint );
    drawFrame( Tpelv );
    //m_absPos = get_joint_transform( joint );

    Tinv = Tpelv.inverse();
    pos = Tinv*pos_[NECK];

    double TorsoX = atan2( -pos[1] , pos[2] );
    index_dof = human_->GetJoint("TorsoX")->GetDOFIndex();
    q[index_dof] = TorsoX;

    TrotX.linear() = Eigen::Matrix3d( Eigen::AngleAxisd( TorsoX, Eigen::Vector3d::UnitX() ));
    TrotX.translation() = Eigen::Vector3d::Zero();
    TrotXtmp = Tpelv * TrotX;
    Tinv = TrotXtmp.inverse();
    pos = Tinv*pos_[NECK];

    double TorsoY = atan2( pos[0] , pos[2] );
    index_dof = human_->GetJoint("TorsoY")->GetDOFIndex();
    q[index_dof] = TorsoY;

    TrotY.linear() = Eigen::Matrix3d( Eigen::AngleAxisd( TorsoY, Eigen::Vector3d::UnitY() ));
    TrotY.translation() = Eigen::Vector3d::Zero();
    TrotYtmp = TrotXtmp * TrotY;
    Tinv = TrotYtmp.inverse();
    pos = Tinv*pos_[SHOULDER_LEFT];

    double TorsoZ = atan2( -pos[0] , pos[1] );
    index_dof = human_->GetJoint("TorsoZ")->GetDOFIndex();
    q[index_dof] = TorsoZ;

    human_->SetJointValues(q);
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
    human_->SetJointValues(q);

    if( print_ )
        cout << "Set human torso configuration" << endl;

    joint = human_->GetJoint("rShoulderX");
    Eigen::Affine3d T = get_joint_transform( joint );
    pos = T.translation();
    shoulder = T.translation();

    index_dof = human_->GetJoint("rArmTrans")->GetDOFIndex();
    q[index_dof] = ( pos_[ELBOW_RIGHT] - pos ).norm() - 0.2066 ;

    joint = human_->GetJoint("TorsoZ");

    Trot = get_joint_transform( joint );
    Trot.translation() = pos;

    Tinv = Trot.inverse();
    pos = Tinv*pos_[ELBOW_RIGHT];

    // calcul de la direction pour le bras droit
    Eigen::Vector3d dir,sub;
    dir = pos / pos.norm();

    // JP
    // selon x
    double alpha1r = atan2( -pos[2] , -pos[1] );

    if( print_ )
        cout << "get joint rShoulderX" << endl;
    index_dof = human_->GetJoint("rShoulderX")->GetDOFIndex();
    q[index_dof] = alpha1r;

    if( print_ )
        cout << "alpha1r : " << alpha1r << endl;

    Trot2.linear() = Eigen::Matrix3d( Eigen::AngleAxisd( alpha1r, Eigen::Vector3d::UnitX() ));
    Trot2.translation() = Eigen::Vector3d::Zero();

    Trot3 = Trot*Trot2;
    Tinv = Trot3.inverse();
    pos = Tinv*pos_[ELBOW_RIGHT];

    // selon z
    double alpha2r = atan2( pos[0] , -pos[1] );

    if( print_ )
        cout << "get joint rShoulderZ" << endl;
    index_dof =  human_->GetJoint("rShoulderZ")->GetDOFIndex();
    q[index_dof] = alpha2r;

    if( print_ )
        cout << "alpha2r : " << alpha2r << endl;

    Trot2.linear() = Eigen::Matrix3d( Eigen::AngleAxisd( alpha2r, Eigen::Vector3d::UnitZ() ));
    Trot2.translation() = Eigen::Vector3d::Zero();
    Trot4 = Trot3*Trot2;
    Tinv = Trot4.inverse();
    pos = Tinv*pos_[HAND_RIGHT];

    // selon y
    double alpha3r = atan2( pos[2], pos[0] );

    if( print_ )
        cout << "get joint rShoulderY" << endl;
    index_dof = human_->GetJoint("rShoulderY")->GetDOFIndex();
    q[index_dof] = alpha3r;

    if( print_ )
        cout << "alpha3r : " << alpha3r << endl;

    vect1 = shoulder - pos_[ELBOW_RIGHT];
    vect1.normalize();

    vect2 = pos_[HAND_RIGHT] - pos_[ELBOW_RIGHT];
    vect2.normalize();

    // Elbow
    double alpha4r = M_PI - acos( vect1.dot(vect2) ) ;

    if( print_ )
        cout << "get joint rElbowZ" << endl;
    index_dof = human_->GetJoint("rElbowZ")->GetDOFIndex();
    q[index_dof] = alpha4r;

    if( print_ )
        cout << "alpha4r : " << alpha4r << endl;

    if( print_ )
        print_config(q);

    human_->SetJointValues(q);

    if( print_ )
        cout << "Set human right arm configuration" << endl;
    //        /return;
    //    }
    //---------------------------------------------------------------------------
    //---------------------------------------------------------------------------


    joint = human_->GetJoint("lShoulderX");
    T = get_joint_transform( joint );
    pos = T.translation();
    shoulder = T.translation();

    index_dof = human_->GetJoint("lArmTrans")->GetDOFIndex();
    q[index_dof] = ( pos_[ELBOW_LEFT] - pos ).norm() - 0.2066 ;

    joint = human_->GetJoint("TorsoZ");

    Trot = get_joint_transform( joint );
    Trot.translation() = pos;

    //  p3d_mat4Copy( Trot , m_absPos );

    Tinv = Trot.inverse();
    pos = Tinv*pos_[ELBOW_LEFT];

    // calcul de la direction pour le bras droit
    //Eigen::Vector3d dir,sub;
    dir = pos / pos.norm();

    // JP //********************************************
    // selon x
    double alpha1l = atan2( pos[2] , pos[1] );

    index_dof = human_->GetJoint("lShoulderX")->GetDOFIndex();
    q[index_dof] = alpha1l;

    Trot2.linear() = Eigen::Matrix3d(Eigen::AngleAxisd( alpha1l, Eigen::Vector3d::UnitX() ));
    Trot2.translation() = Eigen::Vector3d::Zero();
    Trot3 = Trot*Trot2;
    Tinv = Trot3.inverse();
    pos = Tinv*pos_[ELBOW_LEFT];

    // selon z
    double alpha2l = atan2( -pos[0] , pos[1] );

    index_dof = human_->GetJoint("lShoulderZ")->GetDOFIndex();
    q[index_dof] = alpha2l;

    Trot2.linear() = Eigen::Matrix3d(Eigen::AngleAxisd( alpha2l, Eigen::Vector3d::UnitZ() ));
    Trot2.translation() = Eigen::Vector3d::Zero();
    Trot4 = Trot3*Trot2;
    Tinv = Trot4.inverse();
    pos = Tinv*pos_[HAND_LEFT];

    // selon y
    double alpha3l = -atan2( -pos[2], pos[0] );

    index_dof = human_->GetJoint("lShoulderY")->GetDOFIndex();
    q[index_dof] = alpha3l;

    vect1 = shoulder - pos_[ELBOW_LEFT];
    vect1.normalize();

    vect2 = pos_[HAND_LEFT] - pos_[ELBOW_LEFT];
    vect2.normalize();

    // Elbow
    double alpha4l = -M_PI + acos( vect1.dot(vect2) ) ;

    index_dof = human_->GetJoint("lElbowZ")->GetDOFIndex();
    q[index_dof] = alpha4l;

    //    }

    human_->SetJointValues(q);
}
