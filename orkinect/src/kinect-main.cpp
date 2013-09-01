#include "kinect-main.hpp"
#include "skeletonListener.hpp"
#include <boost/thread.hpp>

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); (it)++)

KinectProblem::KinectProblem(EnvironmentBasePtr penv) : ProblemInstance(penv)
{
    __description = "A very simple plugin.";
    cout << __description << endl;
    RegisterCommand("numbodies",boost::bind(&KinectProblem::NumBodies,this,_1,_2),"returns bodies");
    RegisterCommand("startlistening",boost::bind(&KinectProblem::StartListening, this,_1,_2),"starts listening to the tf frames");
    RegisterCommand("setkinectframe",boost::bind(&KinectProblem::SetKinectFrame, this,_1,_2),"sets the kinect frame");
    RegisterCommand("setbuttonstate",boost::bind(&KinectProblem::SetButtonState, this,_1,_2),"sets the button state");
    RegisterCommand("loadtrajectoryfile",boost::bind(&KinectProblem::LoadTrajectoryFile, this,_1,_2),"Loads the trajectory file from given path");
    RegisterCommand("playtrajectoryfiles",boost::bind(&KinectProblem::PlayTrajectoryFiles, this,_1,_2),"Plays the trajectory files");
    RegisterCommand("settrajectorycontrol",boost::bind(&KinectProblem::SetTrajectoryControl, this,_1,_2),"Set control of playback flag");
    RegisterCommand("controltrajectoryplayback",boost::bind(&KinectProblem::ControlTrajectoryPlayback, this,_1,_2),"Control playback of trajectory files");
    RegisterCommand("getplaybackframe",boost::bind(&KinectProblem::GetPlaybackFrame, this,_1,_2),"return the current playback frame");
    RegisterCommand("setnumkinect",boost::bind(&KinectProblem::SetNumKinect, this,_1,_2),"set the number of kinects that are being used");
    RegisterCommand("setcustomtracker",boost::bind(&KinectProblem::SetCustomTracker, this,_1,_2),"set which openni tracker is being used");
    RegisterCommand("resamplefiles",boost::bind(&KinectProblem::ResampleFiles, this,_1,_2),"resample all files in _filepaths");
    RegisterCommand("enablecamera",boost::bind(&KinectProblem::EnableCamera, this,_1,_2),"Enable the camera to take live snapshots");

    _skel_listen = new SkeletonListener(penv);
    RobotBasePtr human1 = penv->GetRobot( "human_model" );
    RobotBasePtr human2 = penv->GetRobot( "human_model_blue" );

    if (human1 == NULL && human2 == NULL){
//      _motion_recorders = NULL;
      cout << "No Human in Scene" << endl;
    }
    if(human1 != NULL){
        HRICS::RecordMotion* temp = new HRICS::RecordMotion(human1);
        HRICS::CameraListener* tempCam = new HRICS::CameraListener(0);
        temp->setRobotId(0);
        temp->_camera = tempCam;

        _motion_recorders.push_back( temp );
    }
    if(human2 != NULL){
        HRICS::RecordMotion* temp = new HRICS::RecordMotion(human2);
        HRICS::CameraListener* tempCam = new HRICS::CameraListener(1); //TODO if not custom tracker but human2 is used, set robotid to 0
        temp->setRobotId(1);
        temp->_camera = tempCam;

        _motion_recorders.push_back( temp );
    }

    if( !_motion_recorders.empty() )
    {
        _motion_player = new PlayMotion( penv, _motion_recorders );
    }
    else {
        _motion_player = NULL;
    }
}

void KinectProblem::Destroy()
{
    RAVELOG_INFO("module unloaded from environment\n");
}

KinectProblem::~KinectProblem()
{

}

/**
void KinectProblem::SetActiveRobots(const std::vector<RobotBasePtr >& robots)
{
    if( robots.size() == 0 ) {
        RAVELOG_WARNA("No robots to plan for\n");
        return;
    }

    vector<RobotBasePtr >::const_iterator itrobot;
    FORIT(itrobot, robots) {
        if( strcmp((*itrobot)->GetName().c_str(), _strRobotName.c_str() ) == 0  ) {
            robot = *itrobot;
            break;
        }
    }

    if( robot == NULL ) {
        RAVELOG_ERRORA("Failed to find %S\n", _strRobotName.c_str());
        return;
    }
}
**/

bool KinectProblem::SendCommand(std::ostream& sout, std::istream& sinput)
{
    ProblemInstance::SendCommand(sout,sinput);
    return true;
}

int KinectProblem::main(const std::string& cmd)
{
    RAVELOG_DEBUG("env: %s\n", cmd.c_str());

    const char* delim = " \r\n\t";
    string mycmd = cmd;
    char* p = strtok(&mycmd[0], delim);
    if( p != NULL )
        _strRobotName = p;
    cout << "_strRobotName: " << _strRobotName << endl;

    //std::vector<RobotBasePtr> robots;
    //GetEnv()->GetRobots(robots);
    //SetActiveRobots(robots);
    return 0;
}

bool KinectProblem::NumBodies(ostream& sout, istream& sinput)
{
    vector<KinBodyPtr> vbodies;
    GetEnv()->GetBodies(vbodies);
    sout << vbodies.size();     // publish the results
    return true;
}

bool KinectProblem::StartListening(ostream& sout, istream& sinput)
{
    //_skel_listen->listen();
    _skel_listen->setMotionRecorder(_motion_recorders);
    boost::thread( &SkeletonListener::listen, _skel_listen );
    return true;
}

bool KinectProblem::SetKinectFrame(ostream& sout, istream& sinput)
{
    int KinID = 0;
    double TX = 0;
    double TY = 0;
    double TZ = 0;
    double RotZ = 0;
    double RotY = 0;

    sinput >> KinID;
    sinput >> TX;
    sinput >> TY;
    sinput >> TZ;
    sinput >> RotZ;
    sinput >> RotY;

    _skel_listen->setKinectFrame(KinID,TX,TY,TZ,RotZ,RotY);

    return true;
}

bool KinectProblem::SetNumKinect(ostream& sout, istream& sinput)
{
    int num_kinect;
    sinput >> num_kinect;
    _skel_listen->setNumKinect(num_kinect);

    return true;
}

bool KinectProblem::SetCustomTracker(ostream& sout, istream& sinput)
{
    bool cust_tracker;
    sinput >> cust_tracker;
    _skel_listen->setTracker(cust_tracker);

    return true;
}

bool KinectProblem::SetButtonState(ostream &sout, istream &sinput)
{
    bool temp;
    sinput >> temp;
    _skel_listen->setRecord(temp);

//    cout << "set buton to " << temp << endl;

    return true;
}

bool KinectProblem::LoadTrajectoryFile(ostream& sout, istream& sinput)
{
    cout << "file loaded" << endl;
    string path;
    sinput >> path;
    _filepaths.push_back( path );

    return true;
}

bool KinectProblem::ResampleFiles(ostream& sout, istream& sinput) //TODO fix output name with proper regular expressions.
{
    int sampleSize;
    sinput >> sampleSize;

    HRICS::RecordMotion* recorder = _motion_recorders[0];

    for (int f = 0; f < int(_filepaths.size()); f++)
    {
        //convert int to string
        std::ostringstream f_num;
        f_num << f;

        motion_t a_motion = recorder->loadFromCSV(_filepaths[f]);
        a_motion = recorder->resample( a_motion, sampleSize);
        recorder->saveToCSVJoints( "/home/rafihayne/workspace/statFiles/recorded_motion/resampled_"+f_num.str()+".csv" , a_motion);
    }

    return true;
}

bool KinectProblem::EnableCamera(ostream& sout, istream& sinput)
{

    bool useCamera;
    sinput >> useCamera;

    for (int i = 0; i < int(_motion_recorders.size()); i++ ) {
        _motion_recorders[i]->use_camera_ = useCamera;
        //TODO fix this.  I think this can be implemented better.
        if(_skel_listen->getTracker() == false && i >= 1) //If we're not using the custom tracker (only 1 kinect used), but have multiple humans in the scene, we only need 1 camera feed.
        {
            cout << "More humans than kinects.  Disabling camera for human: " << i << endl;
            _motion_recorders[i]->use_camera_ = false;
        }
    }

    return true;
}

bool KinectProblem::PlayTrajectoryFiles(ostream& sout, istream& sinput)
{
    if( _motion_player == NULL )
        cout << "motion player is not initialized" << endl;

    boost::thread( &PlayMotion::play, _motion_player, _filepaths );
//    _motion_player->play( _filepaths );

    return true;
}

bool KinectProblem::SetTrajectoryControl(ostream& sout, istream& sinput)
{
    bool isControlled;
    sinput >> isControlled;
    _motion_player->setControlled(isControlled);

    return true;
}

bool KinectProblem::ControlTrajectoryPlayback(ostream& sout, istream& sinput)
{
    int step;
    sinput >> step;
    _motion_player->setStep(step);
    _motion_player->setRecentInput(true);

    return true;
}

bool KinectProblem::GetPlaybackFrame(ostream& sout, istream& sinput)
{
    int frame = _motion_player->getCurrentFrame();
    sout << frame;

    return true;
}



//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_ProblemInstance && interfacename == "kinect" ) {
        return InterfaceBasePtr(new KinectProblem(penv));
    }
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_ProblemInstance].push_back("Kinect");
}

RAVE_PLUGIN_API void DestroyPlugin()
{
    RAVELOG_INFO("destroying plugin\n");
}

