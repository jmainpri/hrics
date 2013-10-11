#ifndef PLAYMOTION_HPP
#define PLAYMOTION_HPP

#include <openrave/openrave.h>
#include "recordMotion.hpp"

class PlayMotion {

public:
    PlayMotion(OpenRAVE::EnvironmentBasePtr env, const std::vector<HRICS::RecordMotion*>& recorders);

    void play(const std::vector<std::string>& filepaths);

//    void setDirection(const bool dir);
    void setStep(const int step);
    void setControlled(const bool controlled);
    void setRecentInput(const bool input);
    bool getRecentInput();
    int getCurrentFrame();

    void play_folder( std::string &folder );

private:

    void runRealTime();
    void runControlled();
    void runControlled_folder();


    std::vector<HRICS::RecordMotion*> _motion_recorders;
    OpenRAVE::EnvironmentBasePtr env_;
    std::vector<boost::shared_ptr<void> > graphptrs_;
    int _current_frame;
//    bool _play_dir; //true = forward, false = backwards
    int _step_size;
    bool _play_controlled;
    bool _recent_input;

};
#endif //PLAYMOION_HPP
