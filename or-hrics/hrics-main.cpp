#include "hrics-main.hpp"

HRICSProblem::HRICSProblem(EnvironmentBasePtr penv) : ProblemInstance(penv)
{
    __description = "A very simple plugin.";
    cout << __description << endl;
    RegisterCommand("numbodies",boost::bind(&HRICSProblem::NumBodies,this,_1,_2),"returns bodies");
    RegisterCommand("load",boost::bind(&HRICSProblem::Load, this,_1,_2),"loads a given file");
}

void HRICSProblem::Destroy() {
    RAVELOG_INFO("module unloaded from environment\n");
}


HRICSProblem::~HRICSProblem()
{

}

void HRICSProblem::SetActiveRobots(const std::vector<RobotBasePtr >& robots)
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

bool HRICSProblem::SendCommand(std::ostream& sout, std::istream& sinput)
{
    ProblemInstance::SendCommand(sout,sinput);
    return true;
}

int HRICSProblem::main(const std::string& cmd)
{
    RAVELOG_DEBUG("env: %s\n", cmd.c_str());

    const char* delim = " \r\n\t";
    string mycmd = cmd;
    char* p = strtok(&mycmd[0], delim);
    if( p != NULL )
        _strRobotName = p;

    std::vector<RobotBasePtr> robots;
    GetEnv()->GetRobots(robots);
    SetActiveRobots(robots);
    return 0;
}

bool HRICSProblem::NumBodies(ostream& sout, istream& sinput)
{
    vector<KinBodyPtr> vbodies;
    GetEnv()->GetBodies(vbodies);
    sout << vbodies.size();     // publish the results
    return true;
}

bool HRICSProblem::Load(ostream& sout, istream& sinput)
{
    string filename;
    sinput >> filename;
    bool bSuccess = GetEnv()->Load(filename.c_str());     // load the file
    return bSuccess;
}

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_ProblemInstance && interfacename == "hrics" ) {
        return InterfaceBasePtr(new HRICSProblem(penv));
    }
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_ProblemInstance].push_back("HRICS");
}

RAVE_PLUGIN_API void DestroyPlugin()
{
    RAVELOG_INFO("destroying plugin\n");
}

