#include "prediction-main.hpp"
#include "jointListener.hpp"
#include <boost/thread.hpp>
#include <ros/ros.h>

using namespace std;
using namespace OpenRAVE;
using namespace HRICS;

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); (it)++)

PredictionProblem::PredictionProblem(EnvironmentBasePtr penv) : ProblemInstance(penv)
{
    __description = "A very simple plugin.";
    cout << __description << endl;
    RegisterCommand("numbodies",boost::bind(&PredictionProblem::NumBodies,this,_1,_2),"returns bodies");
    RegisterCommand("startlistening",boost::bind(&PredictionProblem::StartListening, this,_1,_2),"starts listening to the tf frames");

    int argc = 0;
    char** argv;
    ros::init(argc, argv, "orprediction", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh;
    _joint_listen = new JointListener(penv, nh);

    RobotBasePtr human1 = penv->GetRobot( "human_model" );


//    if (human1 == NULL && human2 == NULL){
////      _motion_recorders = NULL;
//      cout << "No Human in Scene" << endl;
//    }
//    if(human1 != NULL){
//        HRICS::RecordMotion* temp = new HRICS::RecordMotion(human1);
//        HRICS::CameraListener* tempCam = new HRICS::CameraListener(0, nh);
//        temp->setRobotId(0);
//        temp->_camera = tempCam;

//        _motion_recorders.push_back( temp );
//    }
//    if(human2 != NULL){
//        HRICS::RecordMotion* temp = new HRICS::RecordMotion(human2);
//        HRICS::CameraListener* tempCam = new HRICS::CameraListener(1, nh); //TODO if not custom tracker but human2 is used, set robotid to 0
//        temp->setRobotId(1);
//        temp->_camera = tempCam;

//        _motion_recorders.push_back( temp );
//    }

//    if( !_motion_recorders.empty() )
//    {
//        _motion_player = new PlayMotion( penv, _motion_recorders );
//    }
//    else {
//        _motion_player = NULL;
//    }
}

void PredictionProblem::Destroy()
{
    RAVELOG_INFO("module unloaded from environment\n");
}

PredictionProblem::~PredictionProblem()
{

}

/**
void PredictionProblem::SetActiveRobots(const std::vector<RobotBasePtr >& robots)
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

bool PredictionProblem::SendCommand(std::ostream& sout, std::istream& sinput)
{
    ProblemInstance::SendCommand(sout,sinput);
    return true;
}

int PredictionProblem::main(const std::string& cmd)
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

bool PredictionProblem::StartListening(ostream& sout, istream& sinput)
{
    //_joint_listen->listen();
    boost::thread( &JointListener::listen, _joint_listen );
    return true;
}

bool PredictionProblem::NumBodies(ostream& sout, istream& sinput)
{
    vector<KinBodyPtr> vbodies;
    GetEnv()->GetBodies(vbodies);
    sout << vbodies.size();     // publish the results
    return true;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_ProblemInstance && interfacename == "prediction" ) {
        return InterfaceBasePtr(new PredictionProblem(penv));
    }
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_ProblemInstance].push_back("Prediction");
}

RAVE_PLUGIN_API void DestroyPlugin()
{
    RAVELOG_INFO("destroying plugin\n");
}

