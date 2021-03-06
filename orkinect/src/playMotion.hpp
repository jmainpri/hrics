#ifndef PLAYMOTION_HPP
#define PLAYMOTION_HPP

#include <openrave/openrave.h>
#include "recordMotion.hpp"
#include "features/orFeatures.hpp"
#include <libmove3d/planners/API/project.hpp>
#include <libmove3d/planners/API/scene.hpp>
#include <libmove3d/planners/API/Graphic/drawModule.hpp>

#include "utils/misc_functions.hpp"
class FeaturesOpenRAVE;

class PlayMotion {

public:
    PlayMotion(OpenRAVE::EnvironmentBasePtr env, const std::vector<HRICS::RecordMotion*>& recorders);

    void play(const std::vector<std::string>& filepaths);

//    void setDirection(const bool dir);
    void setStep(const int step);
    void setPlayType(const int playType);
    void setRecentInput(const bool input);
    bool getRecentInput();
    int getCurrentFrame();
    int setCurrentFrame(int frame);
    void play_folder( std::string &folder );
    void reset_recorders();
    void replay_trajectory();
    void saveStatsToCSV();
    void play_mocap( std::string &m_filename, std::string &o_filename );

    void setUsingMove3D(bool val) {usingMove3D = val; }

private:

    void runRealTime();
    void runControlled();
    void runControlled_folder();
    void runStatistics();
    std::vector< std::vector<std::string> > load_csv_to_matrix( std::string &filename );
    void drawFrame(OpenRAVE::RaveTransformMatrix<double> T);

    HRICS::FeaturesOpenRAVE* features_;
    std::vector<HRICS::RecordMotion*> _motion_recorders;
    OpenRAVE::EnvironmentBasePtr env_;
    std::vector<boost::shared_ptr<void> > graphptrs_;
    std::vector<std::string> filepaths_;
    int _current_frame;
//    bool _play_dir; //true = forward, false = backwards
    int _step_size;
    int _play_type;
    bool _recent_input;
    bool _StopRun;
    bool usingMove3D;

};










#endif //PLAYMOION_HPP
