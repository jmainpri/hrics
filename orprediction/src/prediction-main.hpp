#ifndef PREDICTIONMAIN_HPP
#define PREDICTIONMAIN_HPP

#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <iostream>

using namespace OpenRAVE;

class PredictionProblem : public ModuleBase
{
public:
    PredictionProblem(EnvironmentBasePtr penv);
    virtual ~PredictionProblem();
    void Destroy();

    virtual int main(const std::string& args);
    virtual bool SendCommand(std::ostream& sout, std::istream& sinput);

private:

    bool NumBodies(std::ostream& sout, std::istream& sinput);

    std::string _strRobotName; ///< name of the active robot
    RobotBasePtr robot;
};

#endif // PREDICTIONMAIN_HPP
