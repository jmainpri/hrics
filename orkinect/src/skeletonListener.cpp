#include "skeletonListener.hpp"

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

Eigen::Affine3d get_joint_transform(OpenRAVE::KinBody::JointPtr joint)
{
//    m[0] = 1; m[1] = 0; m[2] = 0;
//    m[4] = 0; m[5] = 1; m[6] = 0;
//    m[8] = 0; m[9] = 0; m[10] = 1;

//    right.x = m[0]; up.x = m[1]; dir.x = m[2];
//    right.y = m[4]; up.y = m[5]; dir.y = m[6];
//    right.z = m[8]; up.z = m[9]; dir.z = m[10];

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
    return T;
}

SkeletonListener::SkeletonListener(OpenRAVE::EnvironmentBasePtr penv)
{
    env_ = penv;

    int argc = 0;
    char** argv;
    ros::init( argc, argv, "orkinect" );
    node_ = new ros::NodeHandle;

    max_num_skel_ = 10;

    user_is_tracked_.resize(max_num_skel_);
    transforms_.resize(max_num_skel_);

    for(int i=0;i<max_num_skel_;i++) {
        user_is_tracked_[i] = false;
        transforms_[i].resize(15);
    }

    human_ = env_->GetRobot( "human_model" );

    cout << "Set Human Robot : " << human_->GetName() << endl;

    OpenRAVE::RaveTransform<float> T;
    T.rot.x = 0.262839;
    T.rot.y = -0.733602;
    T.rot.z = -0.623389;
    T.rot.w = 0.0642694;
    T.trans.x = 2.99336;
    T.trans.y = -0.755646;
    T.trans.z = 2.81558;
    env_->GetViewer()->SetCamera( T );
    //env_->GetViewer()->setCamera( 0.262839, -0.733602, -0.623389, 0.0642694, 2.99336, -0.755646, 2.81558 );

    print_ = true;
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
                listener.lookupTransform("/openni_depth_frame", "/head_" + num_to_string(i) , ros::Time(0), transforms_[i][0]);
                listener.lookupTransform("/openni_depth_frame", "/neck_" + num_to_string(i) , ros::Time(0), transforms_[i][1]);
                listener.lookupTransform("/openni_depth_frame", "/torso_" + num_to_string(i) , ros::Time(0), transforms_[i][2]);

                listener.lookupTransform("/openni_depth_frame", "/left_shoulder_" + num_to_string(i) , ros::Time(0), transforms_[i][3]);
                listener.lookupTransform("/openni_depth_frame", "/left_elbow_" + num_to_string(i), ros::Time(0), transforms_[i][4]);
                listener.lookupTransform("/openni_depth_frame", "/left_hand_" + num_to_string(i), ros::Time(0), transforms_[i][5]);

                listener.lookupTransform("/openni_depth_frame", "/right_shoulder_" + num_to_string(i), ros::Time(0), transforms_[i][6]);
                listener.lookupTransform("/openni_depth_frame", "/right_elbow_" + num_to_string(i), ros::Time(0), transforms_[i][7]);
                listener.lookupTransform("/openni_depth_frame", "/right_hand_" + num_to_string(i), ros::Time(0), transforms_[i][8]);

                listener.lookupTransform("/openni_depth_frame", "/left_hip_" + num_to_string(i), ros::Time(0), transforms_[i][9]);
                listener.lookupTransform("/openni_depth_frame", "/left_knee_" + num_to_string(i), ros::Time(0), transforms_[i][10]);
                listener.lookupTransform("/openni_depth_frame", "/left_foot_" + num_to_string(i), ros::Time(0), transforms_[i][11]);

                listener.lookupTransform("/openni_depth_frame", "/right_hip_" + num_to_string(i), ros::Time(0), transforms_[i][12]);
                listener.lookupTransform("/openni_depth_frame", "/right_knee_" + num_to_string(i), ros::Time(0), transforms_[i][13]);
                listener.lookupTransform("/openni_depth_frame", "/right_foot_" + num_to_string(i), ros::Time(0), transforms_[i][14]);

                user_is_tracked_[i] =  true;
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

        for(int i=0; i<int(tracked_user_id_.size()); i++)
        {
            setHumanConfiguration( tracked_user_id_[i] );
        }

        draw();
        rate.sleep();
    }
}

void SkeletonListener::draw()
{
    graphptrs_.clear();

    if( tracked_user_id_.empty() )
        return;

    OpenRAVE::GraphHandlePtr figure;
    std::vector<OpenRAVE::RaveVector<float> > vpoints;
    std::vector<float> vcolors;

    for(int i=0; i<int(tracked_user_id_.size()); i++)
    {
        if( !user_is_tracked_[i] )
            continue;

        for(int j = 0; j<15; j++)
        {
            float x = transforms_[ tracked_user_id_[i] ][j].getOrigin().getX();
            float y = transforms_[ tracked_user_id_[i] ][j].getOrigin().getY();
            float z = transforms_[ tracked_user_id_[i] ][j].getOrigin().getZ();

            OpenRAVE::RaveVector<float> pnt(x,y,z);
            vpoints.push_back(pnt);
            vcolors.push_back(1);
            vcolors.push_back(0);
            vcolors.push_back(0);
        }
    }

    figure = env_->plot3( &vpoints[0].x, vpoints.size(), sizeof(vpoints[0]), 20.0, &vcolors[0], 0 );
    graphptrs_.push_back( figure );

    //cout << env_->GetViewer()->GetCameraTransform() << endl;
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
    }
}

//! @ingroup KINECT
//! Computes a condfiguration
//! from a set of points
void SkeletonListener::setHumanConfiguration(int id)
{
    cout << "Set Human Configuration" << endl;

    setEigenPositions(id);

    // Set pelvis
    OpenRAVE::KinBody::JointPtr joint; // = human_->GetJoint("Pelvis");
    int index_dof; //= joint->GetDOFIndex();

    std::vector<double> q;
    human_->GetDOFValues(q);

    Eigen::Vector3d pos;

////    if( data.TORSO.confidence > 0 && data.HIP_LEFT.confidence > 0 && data.HIP_RIGHT.confidence > 0 )
////    {
//        q[index_dof+0] = pos_[TORSO][0];
//        q[index_dof+1] = pos_[TORSO][1];
//        q[index_dof+2] = pos_[TORSO][2] - 0.20 ; // Hack 1 meter

//        // calcul de l'orientation du pelvis
//        Eigen::Vector3d sum, midP;
//        sum = pos_[HIP_LEFT] + pos_[HIP_RIGHT];
//        midP = 05*sum;

//        q[index_dof+5] = atan2( pos_[HIP_LEFT][1]-midP[1] , pos_[HIP_LEFT][0]-midP[0]  );
//        q[index_dof+5] += -M_PI/2; // Hack +  Pi / 2
////    }

    //--------------------------------------------------------------
      Eigen::Affine3d Tinv,Tpelv;
      Eigen::Vector3d shoulder;
      //Eigen::Vector3d Xaxis; Xaxis << 1 , 0 , 0 ;
      Eigen::Affine3d TrotX,TrotXtmp;
      //Eigen::Vector3d Yaxis; Yaxis << 0 , 1 , 0 ;
      Eigen::Affine3d TrotY,TrotYtmp;

//    p3d_matrix4 Tinv,Tpelv;
//    p3d_vector3 shoulder;
//    p3d_vector3 Xaxis = { 1 , 0 , 0 };
//    p3d_matrix4 TrotX,TrotXtmp;
//    p3d_vector3 Yaxis = { 0 , 1 , 0 };
//    p3d_matrix4 TrotY,TrotYtmp;

//    if( data.NECK.confidence > 0 && data.SHOULDER_LEFT.confidence > 0 )
//    {
        human_->SetJointValues(q);

        joint = human_->GetJoint("TorsoX");

        Tpelv = get_joint_transform( joint );
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

        TrotY.linear() = Eigen::Matrix3d( Eigen::AngleAxisd( TorsoX, Eigen::Vector3d::UnitX() ));
        TrotY.translation() = Eigen::Vector3d::Zero();
        TrotYtmp = TrotXtmp * TrotY;
        Tinv = TrotYtmp.inverse();
        pos = Tinv*pos_[SHOULDER_LEFT];

        double TorsoZ = atan2( -pos[0] , pos[1] );
        index_dof = human_->GetJoint("TorsoZ")->GetDOFIndex();
        q[index_dof] = TorsoZ;
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

        joint = human_->GetJoint("rShoulderX");
        Eigen::Affine3d T = get_joint_transform( joint );
        pos = T.translation();
        shoulder = T.translation();

        index_dof = human_->GetJoint("rArmTrans")->GetDOFIndex();
        q[index_dof] = ( pos_[ELBOW_RIGHT] - pos ).norm() - 0.2066 ;

        joint = human_->GetJoint("TorsoZ");

        Trot = get_joint_transform( joint );
        Trot.translation() = pos;

        //  p3d_mat4Copy( Trot , m_absPos );

        Tinv = Trot.inverse();
        pos = Tinv*pos_[ELBOW_RIGHT];

        // calcul de la direction pour le bras droit
        Eigen::Vector3d dir,sub;
        dir = pos / pos.norm();

        // JP
        // selon x
        double alpha1r = atan2( -pos[2] , -pos[1] );

        TrotX.linear() = Eigen::Matrix3d( Eigen::AngleAxisd( alpha1r, Eigen::Vector3d::UnitX() ));
        TrotX.translation() = Eigen::Vector3d::Zero();
        Trot3 = Trot*Trot2;
        Tinv = Trot3.inverse();
        pos = Tinv*pos_[ELBOW_RIGHT];

        index_dof = human_->GetJoint("rShoulderX")->GetDOFIndex();
        q[index_dof] = alpha1r;

        // selon z
        double alpha2r = atan2( pos[0] , -pos[1] );

        Trot2.linear() = Eigen::Matrix3d( Eigen::AngleAxisd( alpha2r, Eigen::Vector3d::UnitZ() ));
        Trot2.translation() = Eigen::Vector3d::Zero();
        Trot4 = Trot3*Trot2;
        Tinv = Trot4.inverse();
        pos = Tinv*pos_[HAND_RIGHT];

        index_dof =  human_->GetJoint("rShoulderZ")->GetDOFIndex();
        q[index_dof] = alpha2r;

        // selon y
        double alpha3r = atan2( pos[2], pos[0] );

        index_dof = human_->GetJoint("rShoulderY")->GetDOFIndex();
        q[index_dof] = alpha3r;

        vect1 = shoulder - pos_[ELBOW_RIGHT];
        vect1.normalize();

        vect2 = pos_[HAND_RIGHT] - pos_[ELBOW_RIGHT];
        vect2.normalize();

        // Elbow
        double alpha4r = M_PI - acos( vect1.dot(vect2) ) ;

        index_dof = human_->GetJoint("rElbowZ")->GetDOFIndex();
        q[index_dof] = alpha4r;
//    }

    //---------------------------------------------------------------------------
    //---------------------------------------------------------------------------
//    if( data.ELBOW_LEFT.confidence > 0 && data.HAND_LEFT.confidence > 0 )
//    {
        joint = human_->GetJoint("lShoulderX");
        T = get_joint_transform( joint );
        pos = T.translation();
        shoulder = T.translation();

        index_dof = human_->GetJoint("rArmTrans")->GetDOFIndex();
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
        double alpha1l = atan2( -pos[2] , -pos[1] );

        index_dof = human_->GetJoint("lShoulderX")->GetDOFIndex();
        q[index_dof] = alpha1l;

        Trot2.linear() = Eigen::Matrix3d(Eigen::AngleAxisd( alpha1l, Eigen::Vector3d::UnitX() ));
        Trot2.translation() = Eigen::Vector3d::Zero();
        Trot3 = Trot*Trot2;
        Tinv = Trot3.inverse();
        pos = Tinv*pos_[ELBOW_RIGHT];

        // selon z
        double alpha2l = atan2( pos[0] , -pos[1] );

        index_dof = human_->GetJoint("lShoulderZ")->GetDOFIndex();
        q[index_dof] = alpha2l;

        Trot2.linear() = Eigen::Matrix3d(Eigen::AngleAxisd( alpha2l, Eigen::Vector3d::UnitZ() ));
        Trot2.translation() = Eigen::Vector3d::Zero();
        Trot4 = Trot3*Trot2;
        Tinv = Trot4.inverse();
        pos = Tinv*pos_[HAND_LEFT];

        // selon y
        double alpha3l = atan2( pos[2], pos[0] );

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
