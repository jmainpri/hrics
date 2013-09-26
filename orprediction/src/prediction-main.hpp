#ifndef PREDICTIONMAIN_HPP
#define PREDICTIONMAIN_HPP

#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <iostream>

namespace HRICS
{

class JointListener;

class PredictionProblem : public OpenRAVE::ModuleBase
{
public:
    PredictionProblem(OpenRAVE::EnvironmentBasePtr penv);
    virtual ~PredictionProblem();
    void Destroy();

    virtual int main(const std::string& args);
    virtual bool SendCommand(std::ostream& sout, std::istream& sinput);

private:

    bool NumBodies(std::ostream& sout, std::istream& sinput);
    bool StartListening(std::ostream& sout, std::istream& sinput);

    std::string _strRobotName; ///< name of the active robot
    OpenRAVE::RobotBasePtr robot;
    JointListener* _joint_listen;
};

}

#endif // PREDICTIONMAIN_HPP
