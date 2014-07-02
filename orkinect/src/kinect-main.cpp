#include "kinect-main.hpp"
#include <boost/thread.hpp>

#include "skeletonListener.hpp"
#include "playMotion.hpp"
#include "recordMotion.hpp"
#include "cameraListener.hpp"

#include <libmove3d/planners/API/Device/robot.hpp>
#include <libmove3d/planners/API/Device/joint.hpp>
#include <libmove3d/planners/API/project.hpp>
#include <libmove3d/planners/API/scene.hpp>

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
    RegisterCommand("resettrajectoryfiles",boost::bind(&KinectProblem::ResetTrajectoryFiles, this,_1,_2),"Resets the loaded trajectory files");
    RegisterCommand("replaytrajectoryfiles",boost::bind(&KinectProblem::ReplayTrajectoryFiles, this,_1,_2),"Replays the loaded trajectory files");
    RegisterCommand("playtrajectoryfiles",boost::bind(&KinectProblem::PlayTrajectoryFiles, this,_1,_2),"Plays the trajectory files");
    RegisterCommand("playtrajectoryfolder",boost::bind(&KinectProblem::PlayTrajectoryFolder, this,_1,_2),"Plays the trajectory folder");
    RegisterCommand("setplaytype",boost::bind(&KinectProblem::SetPlayType, this,_1,_2),"Set type of playback");
    RegisterCommand("controltrajectoryplayback",boost::bind(&KinectProblem::ControlTrajectoryPlayback, this,_1,_2),"Control playback of trajectory files");
    RegisterCommand("getplaybackframe",boost::bind(&KinectProblem::GetPlaybackFrame, this,_1,_2),"return the current playback frame");
    RegisterCommand("setnumkinect",boost::bind(&KinectProblem::SetNumKinect, this,_1,_2),"set the number of kinects that are being used");
    RegisterCommand("setcustomtracker",boost::bind(&KinectProblem::SetCustomTracker, this,_1,_2),"set which openni tracker is being used");
    RegisterCommand("resamplefiles",boost::bind(&KinectProblem::ResampleFiles, this,_1,_2),"resample all files in _filepaths");
    RegisterCommand("decrementfile",boost::bind(&KinectProblem::DecrementFile, this,_1,_2),"decrement the file id.  allows you to overwrite a saved motion");
    RegisterCommand("enablecamera",boost::bind(&KinectProblem::EnableCamera, this,_1,_2),"Enable the camera to take live snapshots");
    RegisterCommand("usepr2frame",boost::bind(&KinectProblem::UsePR2Frame, this,_1,_2),"Enable use of the pr2 headframe as a transform");
    RegisterCommand("initmove3d",boost::bind(&KinectProblem::InitMove3D, this,_1,_2),"initializes move3d data structures");
    RegisterCommand("drawmocapfile",boost::bind(&KinectProblem::DrawMocapFile, this,_1,_2),"draws a recorded mocap marker file");

    int argc = 0;
    char** argv;
    ros::init(argc, argv, "orkinect", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    _skel_listen = new SkeletonListener(penv, nh);
    RobotBasePtr human1 = penv->GetRobot( "human_model" );
    RobotBasePtr human2 = penv->GetRobot( "human_model_blue" );

    if (human1 == NULL && human2 == NULL){
//      _motion_recorders = NULL;
      cout << "No Human in Scene" << endl;
    }
    if(human1 != NULL){
        HRICS::RecordMotion* temp = new HRICS::RecordMotion(human1);
        HRICS::CameraListener* tempCam = new HRICS::CameraListener(0, nh);
        temp->setRobotId(0);
        temp->_camera = tempCam;

        _motion_recorders.push_back( temp );
    }
    if(human2 != NULL){
        HRICS::RecordMotion* temp = new HRICS::RecordMotion(human2);
        HRICS::CameraListener* tempCam = new HRICS::CameraListener(1, nh); //TODO if not custom tracker but human2 is used, set robotid to 0
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

bool KinectProblem::DrawMocapFile(ostream& sout, istream& sinput)
{
    cout << "file loaded" << endl;
    string path;
    sinput >> path;
    _motion_player->play_mocap(path);

    return true;
}

bool KinectProblem::ResetTrajectoryFiles(ostream& sout, istream& sinput)
{
    cout << "Resetting Loaded Trajectories" << endl;
    _filepaths.clear();
    _motion_player->reset_recorders();

    return true;
}

bool KinectProblem::ReplayTrajectoryFiles(ostream& sout, istream& sinput)
{
    cout << "Replaying Loaded Trajectories" << endl;
    _motion_player->replay_trajectory();

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
        string home(getenv("HOME"));
        recorder->saveToCSVJoints( home + "/Desktop/oct_lib/resampled_"+f_num.str()+".csv" , a_motion);
    }

    return true;
}

bool KinectProblem::EnableCamera(ostream& sout, istream& sinput)
{
    bool useCamera;
    std::string folder;

    sinput >> useCamera;
    sinput >> folder;

    if( folder != "" )
        cout << "Camera Folder : " << folder << endl;

    for (int i = 0; i < int(_motion_recorders.size()); i++ ) {
        _motion_recorders[i]->use_camera_ = useCamera;
        if( folder != "" )
            _motion_recorders[i]->_camera->setFolder( folder );
        //TODO fix this.  I think this can be implemented better.
        if(_skel_listen->getTracker() == false && i >= 1) //If we're not using the custom tracker (only 1 kinect used), but have multiple humans in the scene, we only need 1 camera feed.
        {
            cout << "More humans than kinects.  Disabling camera for human: " << i << endl;
            _motion_recorders[i]->use_camera_ = false;
        }
    }

    return true;
}

bool KinectProblem::DecrementFile(ostream& sout, istream& sinput)
{

    for (int i = 0; i < int(_motion_recorders.size()); i++ ) {
        _motion_recorders[i]->decrement_file();
    }

    return true;
}

bool KinectProblem::UsePR2Frame(ostream& sout, istream& sinput)
{

    bool usePR2;
    sinput >> usePR2;
    _skel_listen->setPR2(usePR2);

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

bool KinectProblem::PlayTrajectoryFolder(ostream& sout, istream& sinput)
{
    std::string folder;
    sinput >> folder;

    boost::thread( &PlayMotion::play_folder, _motion_player, folder );

    return true;
}

bool KinectProblem::SetPlayType(ostream& sout, istream& sinput)
{
    int playType;
    sinput >> playType;
    _motion_player->setPlayType(playType);

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

bool KinectProblem::SetPlaybackFrame(ostream& sout, istream& sinput)
{
    int frame;
    sinput >> frame;
    _motion_player->setCurrentFrame(frame);

    return true;
}


bool KinectProblem::InitMove3D(ostream& sout, istream& sinput)
{
    RobotBasePtr human = GetEnv()->GetRobot("human_model"); // Getting transform
//    TransformMatrix T = human->GetTransform();
//    T.trans.x=-1;  T.trans.y=1;  T.trans.z=0;
//    human->SetTransform(T);

    std::ostringstream out;
    std::istringstream iss;

    std::string coll_checker_name = "VoxelColChecker" ;
    CollisionCheckerBasePtr pchecker = RaveCreateCollisionChecker( GetEnv(), coll_checker_name.c_str() ); // create the module
    if( !pchecker ) {
        RAVELOG_ERROR( "Failed to create checker %s\n", coll_checker_name.c_str() );
        return false;
    }

    // VOXEL COLLISON CHECKER
    iss.clear(); iss.str("SetDimension robotcentered extent 2.0 2.0 2.0 offset 1.0 1.0 -1.0"); pchecker->SendCommand( out, iss );
    iss.clear(); iss.str("SetCollisionPointsRadii radii 6 0.20 .14 .10 .08 .07 .05 activation 6 0 1 1 1 1 0"); pchecker->SendCommand( out, iss );
    iss.clear(); iss.str("Initialize"); pchecker->SendCommand( out, iss );
    iss.clear(); iss.str("SetDrawing off"); pchecker->SendCommand( out, iss );
    GetEnv()->SetCollisionChecker( pchecker );

    std::string coll_move3d_name = "Move3d" ;
    ModuleBasePtr pmove3d = RaveCreateModule( GetEnv(), coll_move3d_name.c_str() ); // create the module
    if( !pmove3d ) {
        RAVELOG_ERROR( "Failed to create checker %s\n", coll_move3d_name.c_str() );
        return false;
    }

    // MOVE3D
    iss.clear(); iss.str("InitMove3dEnv"); pmove3d->SendCommand( out, iss );
    iss.clear(); iss.str("LoadConfigFile " + std::string(getenv("HOME")) + "/workspace/move3d/move3d-launch/parameters/params_collaboration_planning"); pmove3d->SendCommand( out, iss );
    iss.clear(); iss.str("SetParameter drawTraj 1"); pmove3d->SendCommand( out, iss );
    iss.clear(); iss.str("SetParameter jntToDraw 5"); pmove3d->SendCommand( out, iss );
    iss.clear(); iss.str("SetParameter trajStompTimeLimit 2.5"); pmove3d->SendCommand( out, iss );

    cout << "Move3D::global_Project->getActiveScene()->getActiveRobot()->getName() : " << Move3D::global_Project->getActiveScene()->getActiveRobot()->getName() << endl;

    //Let the motion player know that it can use move3d
    _motion_player->setUsingMove3D(true);

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

