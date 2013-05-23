#include "playMotion.hpp"


using std::cout;
using std::endl;

PlayMotion::PlayMotion( OpenRAVE::EnvironmentBasePtr env, const std::vector<HRICS::RecordMotion*>& recorders )
{
    env_ = env;
    _motion_recorders = recorders;
}

void PlayMotion::play( const std::vector<std::string>& filepaths )
{
    if( filepaths.size() > _motion_recorders.size() )
    {
        cout << "The number of motion recorder is reater than te number of motion to play" << endl;
        return;
    }

    for (int i=0; i<int(filepaths.size()); i++)
    {
        _motion_recorders[i]->storeMotion( _motion_recorders[i]->loadFromCSV(filepaths[i]), true );
    }

    run();
}

void PlayMotion::run()
{
    if( _motion_recorders.empty() ) {
        return;
    }

    int StopRun = false;
    int i=0;
    double tu_last = 0.0;
    double dt = 0.0;

    while ( !StopRun )
    {
        timeval tim;
        gettimeofday(&tim, NULL);
        double tu = tim.tv_sec+(tim.tv_usec/1000000.0);
        dt += ( tu - tu_last );
        tu_last = tu;

        if ( dt>=0.025 )
        {
            for (int j=0; j<int(_motion_recorders.size()); j++)
            {
//                cout << "configuration " << i << endl;
                _motion_recorders[j]->setRobotToStoredMotionConfig(0,i);
            }

            dt = 0.0;
            i++;
        }

        if ( i >= int(_motion_recorders[0]->getStoredMotions()[0].size())) {
            StopRun = true;
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
}
