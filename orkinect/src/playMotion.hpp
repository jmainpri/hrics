#ifndef PLAYMOTION_HPP
#define PLAYMOTION_HPP

#include <openrave/openrave.h>
#include "recordMotion.hpp"

class PlayMotion {

public:
    PlayMotion(OpenRAVE::EnvironmentBasePtr env, const std::vector<HRICS::RecordMotion*>& recorders);

    void play(const std::vector<std::string>& filepaths);

private:

    void run();

    std::vector<HRICS::RecordMotion*> _motion_recorders;
    OpenRAVE::EnvironmentBasePtr env_;
    std::vector<boost::shared_ptr<void> > graphptrs_;

};
#endif //PLAYMOION_HPP
