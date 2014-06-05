#include "playMotion.hpp"

using std::cout;
using std::endl;

PlayMotion::PlayMotion( OpenRAVE::EnvironmentBasePtr env, const std::vector<HRICS::RecordMotion*>& recorders )
{
    env_ = env;
    _motion_recorders = recorders;
    _StopRun = false;

    usingMove3D = false;
}


//void PlayMotion::setDirection(const bool dir) { _play_dir = dir; }
void PlayMotion::setStep(const int step) { _step_size = step; }
void PlayMotion::setPlayType(const int playType) { _play_type = playType; }
void PlayMotion::setRecentInput(const bool input) { _recent_input = input;}
bool PlayMotion::getRecentInput() { return _recent_input;}
int PlayMotion::getCurrentFrame() { return _current_frame; }


void PlayMotion::play( const std::vector<std::string>& filepaths )
{
    if (    _motion_recorders.size() == 2  && usingMove3D )
    {
        features_ = new HRICS::FeaturesOpenRAVE();
        features_->init_dist("human_model", "human_model_blue");
        features_->init_vel("human_model");
    }

    if( filepaths.size() > _motion_recorders.size() )
    {
        cout << "The number of motion recorder is reater than te number of motion to play" << endl;
        return;
    }

    for (int i=0; i<int(filepaths.size()); i++)
    {
        //TODO This is supposed to queue motions, but it wont' work that way.  Probably need to use a modulo to select correct human/file
        _motion_recorders[i]->storeMotion( _motion_recorders[i]->loadFromCSV(filepaths[i]), true );
    }

    _StopRun = false;
    cout << "Play type is: " << _play_type << endl;
    switch (_play_type) {
    case 0:
        runRealTime();
        break;
    case 1:
        runControlled();
        break;
    case 2:
        runStatistics();
    default:
        runRealTime();
    }

}

void PlayMotion::play_folder( std::string &folder )
{
    _motion_recorders[0]->loadFolder(folder);
    runControlled_folder();
}

void PlayMotion::runRealTime()
{
    if( _motion_recorders.empty() ) {
        return;
    }

    _current_frame=0;
    int last_frame = _motion_recorders[0]->getStoredMotions()[0].size();  //Assumes all motion recorders have the same # of frames.  Should probably take the highest
    double tu_last = 0.0;
    double dt = 0.0;

    for (_current_frame; _current_frame < last_frame; _current_frame++)
    {
//        timeval tim;
//        gettimeofday(&tim, NULL);
//        double tu = tim.tv_sec+(tim.tv_usec/1000000.0);
//        dt += ( tu - tu_last );
//        tu_last = tu;
        dt = _motion_recorders[0]->get_dt(0, _current_frame);
        usleep(dt*1000000.0);

        for (int j=0; j<int(_motion_recorders.size()); j++)
        {
//                cout << "configuration " << i << endl;
            _motion_recorders[j]->setRobotToStoredMotionConfig(0,_current_frame);
        }
//        graphptrs_.clear();

//        std::vector<OpenRAVE::RaveVector<float> > vpoints;
//        std::vector<float> vcolors;

//        OpenRAVE::RaveVector<float> pnt(0,0,0);
//        vpoints.push_back(pnt);
//        vcolors.push_back(1);
//        vcolors.push_back(0);
//        vcolors.push_back(0);

//        graphptrs_.push_back( env_->plot3( &vpoints[0].x, vpoints.size(), sizeof(vpoints[0]), 0.05, &vcolors[0], 1 ) );
    }

    cout << "End play motion" << endl;
    return;
}

void PlayMotion::runStatistics()
{
    cout << "Running statistics on supplied trajectories" << endl;

    if( _motion_recorders.empty() || !usingMove3D ) {
        return;
    }

    _current_frame=0;
    int nb_frames = _motion_recorders[0]->getStoredMotions()[0].size();  //Assumes all motion recorders have the same # of frames.  Should probably take the highest
    Eigen::VectorXd times(nb_frames);
    float total_time = 0.0;

    Move3D::Robot* human1 = Move3D::global_Project->getActiveScene()->getRobotByName("human_model");
    Move3D::Trajectory traj( human1 );

    std::vector<double> dt_tmp;

    for (_current_frame; _current_frame < nb_frames; _current_frame++)
    {
        for (int j=0; j<int(_motion_recorders.size()); j++)
        {
            _motion_recorders[j]->setRobotToStoredMotionConfig(0,_current_frame);
        }

        traj.push_back( human1->getCurrentPos() );

        //Get time stamp
        double dt = _motion_recorders[0]->get_dt(0, _current_frame);

        dt_tmp.push_back( dt );

//        cout << "dt : " << dt << endl;
        total_time += dt;
        times(_current_frame) = total_time;
//        cout << "times(_current_frame) : " << times(_current_frame) << endl;

        features_->bufferDistance();
        features_->bufferVelocity( dt );
    }

//    double s = 0.0;
//    double s_max = traj.getParamMax();
//    int n_step = nb_frames;
//    double step = s_max / nb_frames;

//    for ( int i=0; i<n_step; i++ )
//    {
//        human1->setAndUpdate( *traj.configAtParam(s) );

//        features_->bufferDistance();
//        features_->bufferVelocity( dt_tmp[i] );

//        s += step;
//    }

    cout << "End computing features, saving to file" << endl;

    std::vector<Eigen::VectorXd> features_dist = features_->getDistBuffer();
    std::vector< std::vector<Eigen::Vector3d> > features_vel = features_->getVelBuffer();

    if( (!features_dist.empty()) || (!features_vel.empty()) || nb_frames < 1 )
    {
        int nb_features_dist = features_dist[0].size();
        Eigen::MatrixXd mat_dist( 1 + nb_features_dist, nb_frames );

        mat_dist.row(0) = times;

        for (int i=0; i < nb_features_dist; i++)
            for (int j=0; j < nb_frames; j++)
                mat_dist(i+1,j) = features_dist[j](i);

        cout << "rows : " << mat_dist.rows() << " , cols " << mat_dist.cols() << endl;

        const char* home = getenv("HOME_MOVE3D");
        if( home ){
            move3d_save_matrix_to_file( mat_dist, std::string(home) + "/../move3d-launch/matlab/hrics_feature_dist.csv" );
        }

        int nb_features_vel = features_vel[0].size();
        Eigen::MatrixXd mat_vel( 1 + 3*nb_features_vel, nb_frames );

        mat_vel.row(0) = times;

        for (int i=0; i < nb_features_vel; i++)
            for (int j=0; j < nb_frames; j++)
                for (int k=0; k < 3; k++)
                    mat_vel(1+3*i+k,j) = features_vel[j][i](k);

        cout << "rows : " << mat_vel.rows() << " , cols " << mat_vel.cols() << endl;

        if( home ){
            move3d_save_matrix_to_file( mat_vel, std::string(home) + "/../move3d-launch/matlab/hrics_feature_vel.csv" );
        }
    }

    cout << "End play motion" << endl;
    return;
}

void PlayMotion::runControlled() //TODO weird implementation. should be fixed at some point, but definitely working.
{
    int numFrames = int(_motion_recorders[0]->getStoredMotions()[0].size());

    if( _motion_recorders.empty() )
    {
        return;
    }

    _current_frame = 0;
    _recent_input = false;

    while ( !_StopRun )
    {

        for (int j=0; j<int(_motion_recorders.size()); j++)
        {
            _motion_recorders[j]->setRobotToStoredMotionConfig(0,_current_frame);
        }

        if (_recent_input) _current_frame+=_step_size; //I don't it working like this.  Shouldn't constantly be updatin to current frame.

        if (_current_frame >= numFrames) _current_frame = numFrames; //Bounds check
        if (_current_frame < 0) _current_frame = 0; //Bounds check

        if ( _current_frame >= numFrames)
        {
            _StopRun = true;
        }

        _recent_input = false;

    }

    cout << "End play motion" << endl;
    return;
}

void PlayMotion::runControlled_folder() //TODO weird implementation. should be fixed at some point, but definitely working.
{
    std::vector<motion_t> motion_cache = _motion_recorders[0]->getStoredMotions();

    for (int i = 0; i < int(motion_cache.size()); i++)
    {
        int numFrames = int(motion_cache[i].size());

        if( _motion_recorders.empty() )
        {
            return;
        }

        _current_frame = 0;
        _recent_input = false;

        while ( !_StopRun )
        {

            for (int j=0; j<int(_motion_recorders.size()); j++)
            {
                _motion_recorders[j]->setRobotToStoredMotionConfig(i,_current_frame);
            }

            if (_recent_input) _current_frame+=_step_size; //I don't it working like this.  Shouldn't constantly be updatin to current frame.

            if (_current_frame >= numFrames) _current_frame = numFrames; //Bounds check
            if (_current_frame < 0) _current_frame = 0; //Bounds check

            if ( _current_frame >= numFrames)
            {
                _StopRun = true;
            }

            _recent_input = false;

        }

        cout << "End of motion:" << i << endl;
    }
    return;
}

void PlayMotion::reset_recorders()
{
    _StopRun = true;

    for (int j=0; j<int(_motion_recorders.size()); j++)
    {
        _motion_recorders[j]->reset();
    }
}
