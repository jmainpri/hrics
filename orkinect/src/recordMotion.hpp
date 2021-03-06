#ifndef RECORDMOTION_HPP
#define RECORDMOTION_HPP

#include <vector>
#include <string.h>
#include <iostream>
#include <sys/time.h>

#include <openrave/openrave.h>
#include "cameraListener.hpp"

//#include "API/ConfigSpace/configuration.hpp"

typedef std::vector< double > confPtr_t;
typedef std::vector< std::pair<double,confPtr_t> > motion_t;

namespace HRICS
{
class RecordMotion {

public:
    RecordMotion();
    RecordMotion( OpenRAVE::RobotBasePtr robot );
    ~RecordMotion();

    void setRobot(const std::string& robotname);
    void saveCurrentConfig();
    void reset();
    void clearCurrentMotion() { m_motion.clear(); }
    void saveCurrentToFile();
    void saveCurrentToCSV();
    void saveToXml( const std::string& filename );
    void saveToXml( const std::string& filename, const motion_t& motion );

    void decrement_file();

    motion_t loadFromXml( const std::string &filename );
    void loadMotionFromMultipleFiles( const std::string& baseFilename, int number_of_files );
    bool loadRegressedFromCSV();
    void translateStoredMotions();
    motion_t invertTranslation( const motion_t& motion );
    motion_t loadFromCSV( const std::string& filename );
    void loadFolder(std::string& folder);

    void storeMotion( const motion_t& motion, bool new_motion = true);
    void addToCurrentMotion( const motion_t& motion );
    void saveToCSV( const std::string &filename, const motion_t& motion);
    void saveStoredToCSV( const std::string &filename );
    void saveToCSVJoints( const std::string &filename, const motion_t& motion );
    motion_t resample( const motion_t& motion, int nb_sample );
    motion_t getArmTorsoMotion( const motion_t& motion, confPtr_t q );

    void showStoredMotion();
    void showCurrentMotion();
    void showMotion( const motion_t& motion );
    bool setRobotToStoredMotionConfig(int motion_id, int config_id);
    bool setRobotToConfiguration(int ith);
    bool setShowMotion(int ith);

    void drawMotion( const motion_t& motion );
    //void dawColorSkinedCylinder( const Eigen::Vector3d& p1, const Eigen::Vector3d& p2);
    void drawHeraklesArms();
    void draw();

    motion_t extractSubpart(int init, int end );
    motion_t extractSubpart(int init, int end, const motion_t& motion);

    void incrementMotionId() { m_id_motion++; }
    void setRobotId(int id) { m_id_human = id; }

    const std::vector<motion_t>& getStoredMotions() { return m_stored_motions; }
    double get_dt(int motion_id, int config_id ) { return m_stored_motions[motion_id][config_id].first; }

    void saveImageToFile(timeval time);

    bool m_is_recording;
    HRICS::CameraListener* _camera;
    bool use_camera_;
    std::vector<timeval> m_times; //Parallel array to m_motion to save timestamps.  kinda messy

private:
    OpenRAVE::RobotBasePtr m_robot;
    confPtr_t m_init_q;
    double m_time_last_saved;
    double m_time_to_record;
    double m_time_last_record;
    int m_id_file;
    int m_id_motion;
    int m_id_human;
    motion_t m_motion;

    std::vector<motion_t> m_stored_motions;
    int m_ith_shown_motion;

    bool m_save_image;
};
}

extern HRICS::RecordMotion* global_motionRecorder;

#endif // RECORDMOTION_HPP
