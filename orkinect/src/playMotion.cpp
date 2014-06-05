#include "playMotion.hpp"

using std::cout;
using std::endl;

PlayMotion::PlayMotion( OpenRAVE::EnvironmentBasePtr env, const std::vector<HRICS::RecordMotion*>& recorders )
{
    env_ = env;
    _motion_recorders = recorders;
    _StopRun = false;
}


//void PlayMotion::setDirection(const bool dir) { _play_dir = dir; }
void PlayMotion::setStep(const int step) { _step_size = step; }
void PlayMotion::setControlled(const bool controlled) { _play_controlled = controlled; }
void PlayMotion::setRecentInput(const bool input) { _recent_input = input;}
bool PlayMotion::getRecentInput() { return _recent_input;}
int PlayMotion::getCurrentFrame() { return _current_frame; }


void PlayMotion::play( const std::vector<std::string>& filepaths )
{
    if (    _motion_recorders.size() == 2 )
    {
        features_ = new HRICS::FeaturesOpenRAVE();
        features_->init_dist("human_model", "human_model_blue");
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
    if (_play_controlled) runControlled();
    else runRealTime();

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
    double tu_last = 0.0;
    double dt = 0.0;

    while ( !_StopRun )
    {
//        timeval tim;
//        gettimeofday(&tim, NULL);
//        double tu = tim.tv_sec+(tim.tv_usec/1000000.0);
//        dt += ( tu - tu_last );
//        tu_last = tu;
        dt = _motion_recorders[0]->get_dt(0, _current_frame);
        usleep(dt*1000000.0);

        features_->bufferDistance();

        for (int j=0; j<int(_motion_recorders.size()); j++)
        {
//                cout << "configuration " << i << endl;
            _motion_recorders[j]->setRobotToStoredMotionConfig(0,_current_frame);
        }

            _current_frame++;

        if ( _current_frame >= int(_motion_recorders[0]->getStoredMotions()[0].size())) {
            _StopRun = true;
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
