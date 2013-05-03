#ifndef KINECTMAIN_HPP
#define KINECTMAIN_HPP

#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <iostream>

using namespace std;
using namespace OpenRAVE;

class SkeletonListener;

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

private:
    string _strRobotName; ///< name of the active robot
    RobotBasePtr robot;
    SkeletonListener* _skel_listen;
};

#endif // HRICSMAIN_HPP
