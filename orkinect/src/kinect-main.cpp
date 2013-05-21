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
    RegisterCommand("startrecording",boost::bind(&KinectProblem::StartRecording, this,_1,_2),"start recording motion");
    RegisterCommand("savetofile",boost::bind(&KinectProblem::SaveToFile, this, _1, _2), "save recorded motion to file");

    _skel_listen = new SkeletonListener(penv);
    RobotBasePtr human=penv->GetRobot( "human_model" );
    if (human == NULL){
      _motion_recorder = NULL;
      cout << "No Human in Scene" << endl;
    }
    else {
         _motion_recorder = new HRICS::RecordMotion(human);
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
    _skel_listen->setMotionRecorder(_motion_recorder);
    boost::thread( &SkeletonListener::listen, _skel_listen );
    return true;
}

bool KinectProblem::SetKinectFrame(ostream& sout, istream& sinput)
{
    double TX = 0;
    double TY = 0;
    double TZ = 0;
    double RotZ = 0;
    double RotY = 0;

    sinput >> TX;
    sinput >> TY;
    sinput >> TZ;
    sinput >> RotZ;
    sinput >> RotY;

    _skel_listen->setKinectFrame(TX,TY,TZ,RotZ,RotY);

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

bool KinectProblem::StartRecording(ostream& sout, istream &sinput)
{
    _motion_recorder->setRobot(_strRobotName);
    return true;
}

bool KinectProblem::SaveToFile(ostream& sout, istream &sinput)
{
    _motion_recorder->saveCurrentConfig();
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

