#ifndef RECORDMOTION_HPP
#define RECORDMOTION_HPP

#include <vector>
#include <string.h>
#include <iostream>
#include <sys/time.h>

#include <openrave/openrave.h>

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
    void bufferCurrentConfig(double curOffset);
    motion_t fixPelvisFrame();
    motion_t fixPelvisFrame(motion_t motion);
    void reset();
    void clearCurrentMotion() { m_motion.clear(); }
    void saveCurrentToFile();
    void saveCurrentToCSV();
    void saveToXml( const std::string& filename );
    void saveToXml( const std::string& filename, const motion_t& motion );

    motion_t loadFromXml( const std::string &filename );
    void loadMotionFromMultipleFiles( const std::string& baseFilename, int number_of_files );
    bool loadRegressedFromCSV();
    void translateStoredMotions();
    motion_t invertTranslation( const motion_t& motion );
    motion_t loadFromCSV( const std::string& filename );
    void loadFolder();

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
    void setBuffSize(int size) { buff_max_conf = size; }
    void setNumKeep(int num) { buff_num_keep = num; }

    const std::vector<motion_t>& getStoredMotions() { return m_stored_motions; }
    const motion_t& getCurrentMotion() { return m_motion; }

    void findTrueStart();
    void load_classes();
    bool m_is_recording;
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
    int buff_max_conf; // max # of configurations to keep in the buffer at a given time
    int buff_num_keep; //# of configurations to be kept when buffer is resized

    std::vector<motion_t> m_stored_motions;
    std::vector<motion_t> m_classes_;
    int m_ith_shown_motion;

};
}

extern HRICS::RecordMotion* global_motionRecorder;

#endif // RECORDMOTION_HPP
