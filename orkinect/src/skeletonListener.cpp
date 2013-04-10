#include "skeletonListener.hpp"

using std::cout;
using std::endl;

template <typename T>
std::string num_to_string ( T Number )
{
    std::ostringstream ss;
    ss << Number;
    return ss.str();
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
}

void SkeletonListener::listen()
{
    ros::Rate rate(10.0);

    tf::TransformListener listener;

    while (node_->ok())
    {
        for(int i=0;i<10;i++)
        {
            user_is_tracked_[i] = false;

            if( !listener.frameExists("/head_" + num_to_string(i)))
                continue;

            cout << "listening to user " << i << endl;

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

                user_is_tracked_[i] = true;
            }
            catch (tf::TransformException ex){
                //ROS_ERROR("%s",ex.what());
            }

            if( user_is_tracked_[i] ){
                cout << "tracking id : " << i << endl;
            }

            //listener_.lookupTransform("/openni_depth_frame", "/turtle1", ros::Time(0), transform);
        }

        draw();
        rate.sleep();
    }
}

void SkeletonListener::draw()
{
    OpenRAVE::GraphHandlePtr figure;
    std::vector<OpenRAVE::RaveVector<float> > vpoints;
    std::vector<float> vcolors;

    graphptrs_.clear();

    for(int i = 0; i<max_num_skel_; i++)
    {
        if( !user_is_tracked_[i] )
            continue;

        for(int j = 0; j<15; j++)
        {
            float x = transforms_[i][j].getOrigin().getX();
            float y = transforms_[i][j].getOrigin().getY();
            float z = transforms_[i][j].getOrigin().getZ();

            OpenRAVE::RaveVector<float> pnt(x,y,z);
            vpoints.push_back(pnt);
            vcolors.push_back(1);
            vcolors.push_back(0);
            vcolors.push_back(0);
        }
    }

    figure = env_->plot3( &vpoints[0].x,vpoints.size(), sizeof(vpoints[0]),20.0,&vcolors[0], 0 );
    graphptrs_.push_back(figure);
}
