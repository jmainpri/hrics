#ifndef KINECTMAIN_HPP
#define KINECTMAIN_HPP

#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <iostream>

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); (it)++)

using namespace std;
using namespace OpenRAVE;

class SkeletonLinstener;

class KinectProblem : public ProblemInstance
{
public:
    KinectProblem(EnvironmentBasePtr penv);
    virtual ~KinectProblem();
    void Destroy();

    virtual int main(const std::string& args);
    virtual void SetActiveRobots(const std::vector<RobotBasePtr>& robots);
    virtual bool SendCommand(std::ostream& sout, std::istream& sinput);

    bool NumBodies(ostream& sout, istream& sinput);
    bool Load(ostream& sout, istream& sinput);

private:
    string _strRobotName; ///< name of the active robot
    RobotBasePtr robot;
    SkeletonLinstener* _skel_listen;
};

#endif // HRICSMAIN_HPP
