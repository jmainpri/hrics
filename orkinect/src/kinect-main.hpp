#ifndef KINECTMAIN_HPP
#define KINECTMAIN_HPP

#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <iostream>

using namespace std;
using namespace OpenRAVE;

class SkeletonListener;
class PlayMotion;

namespace HRICS
{
class RecordMotion;
}

class KinectProblem : public ModuleBase
{
public:
    KinectProblem(EnvironmentBasePtr penv);
    virtual ~KinectProblem();
    void Destroy();

    virtual int main(const std::string& args);
    virtual bool SendCommand(std::ostream& sout, std::istream& sinput);

    bool NumBodies(ostream& sout, istream& sinput);
    bool StartListening(ostream& sout, istream& sinput);
    bool SetKinectFrame(ostream& sout, istream& sinput);
    bool SetButtonState(ostream& sout, istream& sinput);
    bool StartRecording(ostream& sout, istream& sinput);
    bool SaveToFile(ostream& sout, istream& sinput);
    bool LoadTrajectoryFile(ostream& sout, istream& sinput);
    bool ResetTrajectoryFiles(ostream& sout, istream& sinput);
    bool ReplayTrajectoryFiles(ostream& sout, istream& sinput);
    bool ResampleFiles(ostream& sout, istream& sinput);
    bool DecrementFile(ostream& sout, istream& sinput);
    bool PlayTrajectoryFiles(ostream& sout, istream& sinput);
    bool PlayTrajectoryFolder(ostream& sout, istream& sinput);
    bool SetPlayType(ostream& sout, istream& sinput);
    bool ControlTrajectoryPlayback(ostream& sout, istream& sinput);
    bool GetPlaybackFrame(ostream& sout, istream& sinput);
    bool SetPlaybackFrame(ostream& sout, istream& sinput);
    bool SetNumKinect(ostream& sout, istream& sinput);
    bool SetCustomTracker(ostream& sout, istream& sinput);
    bool EnableCamera(ostream& sout, istream& sinput);
    bool UsePR2Frame(ostream& sout, istream& sinput);
    bool InitMove3D(ostream& sout, istream& sinput);
    bool DrawMocapFile(ostream& sout, istream& sinput);

private:
    string _strRobotName; ///< name of the active robot
    RobotBasePtr robot;
    SkeletonListener* _skel_listen;
    std::vector<HRICS::RecordMotion*> _motion_recorders;
//    std::vector<HRICS::CameraListener*> _camera_listeners;
    PlayMotion* _motion_player;
    std::vector<std::string> _filepaths;
};

#endif // HRICSMAIN_HPP
