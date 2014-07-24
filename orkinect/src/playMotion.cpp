#include "playMotion.hpp"
#include <libmove3d/include/P3d-pkg.h>

using std::cout;
using std::endl;


template <class T>
bool convert_text_to_num(T& t,
                 const std::string& s,
                 std::ios_base& (*f)(std::ios_base&))
{
  std::istringstream iss(s);
  return !(iss >> f >> t).fail();
}


PlayMotion::PlayMotion( OpenRAVE::EnvironmentBasePtr env, const std::vector<HRICS::RecordMotion*>& recorders )
{
    env_ = env;
    _motion_recorders = recorders;
    _StopRun = false;

    usingMove3D = false;
}


//void PlayMotion::setDirection(const bool dir) { _play_dir = dir; }
void PlayMotion::setStep(const int step) { _step_size = step; }
void PlayMotion::setPlayType(const int playType) { _play_type = playType; }
void PlayMotion::setRecentInput(const bool input) { _recent_input = input;}
bool PlayMotion::getRecentInput() { return _recent_input;}
int PlayMotion::getCurrentFrame() { return _current_frame; }
int PlayMotion::setCurrentFrame(int frame) { _current_frame = frame; }


void PlayMotion::play( const std::vector<std::string>& filepaths )
{
    filepaths_ = filepaths;

    if (    _motion_recorders.size() == 2  && usingMove3D )
    {
        features_ = new HRICS::FeaturesOpenRAVE("human_model", "human_model_blue");
        features_->init_dist("human_model", "human_model_blue");
        features_->init_vel("human_model");
        features_->init_pos("human_model");
        features_->init_col("human_model");
    }

    if( filepaths_.size() > _motion_recorders.size() )
    {
        cout << "The number of motion recorder is reater than te number of motion to play" << endl;
        return;
    }

    for (int i=0; i<int(filepaths_.size()); i++)
    {
        //TODO This is supposed to queue motions, but it wont' work that way.  Probably need to use a modulo to select correct human/file
        _motion_recorders[i]->storeMotion( _motion_recorders[i]->loadFromCSV(filepaths_[i]), true );
    }

    _StopRun = false;
    cout << "Play type is: " << _play_type << endl;
    switch (_play_type) {
    case 0:
        runRealTime();
        break;
    case 1:
        runControlled();
        break;
    case 2:
        runStatistics();
    }

    return;

}

void PlayMotion::play_folder( std::string &folder )
{
    _motion_recorders[0]->loadFolder(folder);
    runControlled_folder();
}

std::vector< std::vector<std::string> > PlayMotion::load_csv_to_matrix( std::string &filename )
{
    cout << "Loading Data from CSV" << endl;
    cout << "file: " << filename.c_str() << endl;

    std::ifstream       file( filename.c_str() );
    std::vector< std::vector<std::string> >   matrix;
    std::vector< std::string >   row;
    std::string                 line;
    std::string                  cell;

    while( file )           //Create matrix from csv
    {
        std::getline(file,line);
        std::stringstream lineStream(line);
        row.clear();

        while(std::getline( lineStream, cell, ',' ))
        {
            row.push_back( cell );
        }

        if( !row.empty() )
            matrix.push_back( row );
    }

    if( matrix.empty() ) {
        cout << "no data has been loaded" << endl;
        return matrix;
    }

    cout << "File fully loaded!" << endl;
    return matrix;

}

void PlayMotion::play_mocap( std::string &m_filename, std::string &o_filename )
{
    std::vector< std::vector<std::string> >   m_matrix;
    std::vector< std::vector<std::string> >   o_matrix;
    timeval tim;
    double last_config_time = 0.0;

    // Load marker data
    m_matrix = load_csv_to_matrix(m_filename);
    if( m_matrix.empty() ) {
        cout << "no marker data has been loaded" << endl;
        return;
    }

    // Load object data
    o_matrix = load_csv_to_matrix(o_filename);
    if( o_matrix.empty() ) {
        cout << "no object data has been loaded" << endl;
        return;
    }

    cout << "File fully loaded!" << endl;

    int o_size = o_matrix.size();
    int m_size = m_matrix.size();

    // time sec, time nsec, # of markers, marker id, x, y, z,...
    for ( int row = 0; row < std::min(o_size, m_size); ++row)
    {
        double temp;
        int nb_markers;
        int nb_objects;

        convert_text_to_num<time_t>( tim.tv_sec, m_matrix[row][0], std::dec );
        convert_text_to_num<double>( temp, m_matrix[row][1], std::dec );
        tim.tv_usec = temp / 1000; //Convert from nsec to usec

        convert_text_to_num<int>( nb_markers, m_matrix[row][2], std::dec );
        convert_text_to_num<int>( nb_objects, o_matrix[row][2], std::dec );

        double tu = tim.tv_sec+(tim.tv_usec/1000000.0);
        double dt = 0.0;
        if( last_config_time != 0.0 )
            dt = tu - last_config_time;
        last_config_time = tu;

        // Load and draw markers
        for ( int col = 3; col <= nb_markers*4; col+=4  )
        {

            int id;
            std::string name;
            double x;
            double y;
            double z;
            double color [4];

            convert_text_to_num<std::string>( name, m_matrix[row][col], std::dec );
            convert_text_to_num<double>( x, m_matrix[row][col+1], std::dec );
            convert_text_to_num<double>( y, m_matrix[row][col+2], std::dec );
            convert_text_to_num<double>( z, m_matrix[row][col+3], std::dec );


            id = 0;
            if (name == "ChestFront")
                id = 0;
            if (name ==  "ChestBack")
                id = 1;
            if (name == "SternumFront")
                id = 2;
            if (name == "SternumBack")
                id = 3;
            if (name == "rShoulderFront")
                id = 4;
            if (name == "rShoulderBack")
                id = 5;
            if (name == "rElbowOuter")
                id = 6;
            if (name == "rElbowInner")
                id = 7;
            if (name == "rWristOuter")
                id = 8;
            if (name == "rWristInner")
                id = 9;
            if (name == "rPalm")
                id = 10;
            if (name == "lShoulderFront")
                id = 11;
            if (name == "lShoulderBack")
                id = 12;
            if (name == "lElbowOuter")
                id = 13;
            if (name == "lElbowInner")
                id = 14;
            if (name == "lWristOuter")
                id = 15;
            if (name ==  "lWristInner")
                id = 16;
            if (name == "lPalm")
                id = 17;

            color[3] = 1;
            double scaled = double (id) / double(18);

//            cout << "id : " << id << endl;

            GroundColorMixGreenToRed( color, scaled );
            move3d_draw_sphere(x, y, z, 0.01875, color );

        }





        // Load and draw objects + frame
        for ( int col = 3; col <= nb_objects*9; col+=9  )
        {
            double color [4];
            std::string id;
            bool occluded;
            double x;
            double y;
            double z;
            double r_x;
            double r_y;
            double r_z;
            double r_w;

            convert_text_to_num<std::string>( id, o_matrix[row][col], std::dec );
            convert_text_to_num<bool>( occluded, o_matrix[row+1][col], std::dec );
            convert_text_to_num<double>( x, o_matrix[row][col+2], std::dec );
            convert_text_to_num<double>( y, o_matrix[row][col+3], std::dec );
            convert_text_to_num<double>( z, o_matrix[row][col+4], std::dec );
            convert_text_to_num<double>( r_x, o_matrix[row][col+5], std::dec );
            convert_text_to_num<double>( r_y, o_matrix[row][col+6], std::dec );
            convert_text_to_num<double>( r_z, o_matrix[row][col+7], std::dec );
            convert_text_to_num<double>( r_w, o_matrix[row][col+8], std::dec );

            if (occluded)
                continue;


            OpenRAVE::RaveTransform<double> tf;
            tf.trans.x = x;
            tf.trans.y = y;
            tf.trans.z = z;
            tf.rot.x = r_x;
            tf.rot.y = r_y;
            tf.rot.z = r_z;
            tf.rot.w = r_w;

            OpenRAVE::RaveTransformMatrix<double> T(tf);

//            if ( id == "TouchTomorrow3" || id == "ArchieLeftHand")
            if ( id == "TouchTomorrow3")
            {

//                for (int i = 0; i < 12; i++)
//                    cout << T.m[i] << " ";
//                cout << endl;

                OpenRAVE::RaveVector<double> x_dir;
                OpenRAVE::RaveVector<double> y_dir;
                OpenRAVE::RaveVector<double> z_dir;

                x_dir.x = T.m[0]; y_dir.x = T.m[1]; z_dir.x = T.m[2];
                x_dir.y = T.m[4]; y_dir.y = T.m[5]; z_dir.y = T.m[6];
                x_dir.z = T.m[8]; y_dir.z = T.m[9]; z_dir.z = T.m[10];


//                T.m[0] = - z_dir.x; T.m[1]  = 0; T.m[2] = 0; T.m[3] = x;
//                T.m[4] = - z_dir.y; T.m[5]  = 0; T.m[6] = 0; T.m[7] = y;
//                T.m[8] = - z_dir.z; T.m[9]  = 0; T.m[10] = 1; T.m[11] = z;
//                T.m[12] =        0; T.m[13] = 0; T.m[14] = 0; T.m[15] = 1;
                //Fix y
                OpenRAVE::RaveVector<double> new_x = -z_dir;
                new_x.z = 0;
                new_x /= sqrt(new_x.lengthsqr3());
                OpenRAVE::RaveVector<double> new_z(0,0,1);
                OpenRAVE::RaveVector<double> new_y = new_x.cross(new_z);
                new_y /= sqrt(new_y.lengthsqr3());

                new_y = -new_y;

                T.m[0]  = new_x.x; T.m[1]  = new_y.x; T.m[2]  = new_z.x;
                T.m[4]  = new_x.y; T.m[5]  = new_y.y; T.m[6]  = new_z.y;
                T.m[8]  = new_x.z; T.m[9]  = new_y.z; T.m[10] = new_z.z;

//                for (int k = 0; k < 12; k+=4)
//                {
//                    cout << T.m[k] << ", " << T.m[k+1] << ", " << T.m[k+2] << endl;
//                }
//                cout << T.trans << endl;


                drawFrame(T);
            }

            if ( id == "ArchieLeftHand")
            {

//                for (int i = 0; i < 12; i++)
//                    cout << T.m[i] << " ";
//                cout << endl;

                OpenRAVE::RaveVector<double> x_dir;
                OpenRAVE::RaveVector<double> y_dir;
                OpenRAVE::RaveVector<double> z_dir;

                x_dir.x = T.m[0]; y_dir.x = T.m[1]; z_dir.x = T.m[2];
                x_dir.y = T.m[4]; y_dir.y = T.m[5]; z_dir.y = T.m[6];
                x_dir.z = T.m[8]; y_dir.z = T.m[9]; z_dir.z = T.m[10];


//                T.m[0] = - z_dir.x; T.m[1]  = 0; T.m[2] = 0; T.m[3] = x;
//                T.m[4] = - z_dir.y; T.m[5]  = 0; T.m[6] = 0; T.m[7] = y;
//                T.m[8] = - z_dir.z; T.m[9]  = 0; T.m[10] = 1; T.m[11] = z;
//                T.m[12] =        0; T.m[13] = 0; T.m[14] = 0; T.m[15] = 1;

                OpenRAVE::RaveVector<double> new_x = -z_dir;
                new_x.z = 0;
                new_x /= sqrt(new_x.lengthsqr3());
                OpenRAVE::RaveVector<double> new_z(0,0,1);
                OpenRAVE::RaveVector<double> new_y = new_x.cross(new_z);
                new_y /= sqrt(new_y.lengthsqr3());

                new_y = -new_y;

                T.m[0]  = new_x.x; T.m[1]  = new_y.x; T.m[2]  = new_z.x;
                T.m[4]  = new_x.y; T.m[5]  = new_y.y; T.m[6]  = new_z.y;
                T.m[8]  = new_x.z; T.m[9]  = new_y.z; T.m[10] = new_z.z;

//                for (int k = 0; k < 12; k+=4)
//                {
//                    cout << T.m[k] << ", " << T.m[k+1] << ", " << T.m[k+2] << endl;
//                }
//                cout << T.trans << endl;

                drawFrame(T);
            }

            color[0] = 1;
            color[1] = 0;
            color[2] = 0;
            color[3] = 1;

            move3d_draw_sphere(x, y, z, 0.01875, color );
        }









//        row = 0;
//        sleep(2);
        usleep(dt*1000000.0);
        move3d_draw_clear_handles();
        graphptrs_.clear();
    }
}

void PlayMotion::drawFrame(OpenRAVE::RaveTransformMatrix<double> T)
{
    OpenRAVE::GraphHandlePtr fig1,fig2,fig3;
    OpenRAVE::Vector right,up,dir, pos;

    T.Extract( right, up, dir, pos );

    fig1 = env_->drawarrow( pos, pos+0.5*right, 0.01, OpenRAVE::Vector(1,0,0,1));
    fig2 = env_->drawarrow( pos, pos+0.5*up,    0.01, OpenRAVE::Vector(0,1,0,1));
    fig3 = env_->drawarrow( pos, pos+0.5*dir,   0.01, OpenRAVE::Vector(0,0,1,1));

    graphptrs_.push_back( fig1 );
    graphptrs_.push_back( fig2 );
    graphptrs_.push_back( fig3 );
}

void PlayMotion::runRealTime()
{
    if( _motion_recorders.empty() ) {
        return;
    }

    _current_frame=0;
    int last_frame = _motion_recorders[0]->getStoredMotions()[0].size();  //Assumes all motion recorders have the same # of frames.  Should probably take the highest
    double tu_last = 0.0;
    double dt = 0.0;

    for (_current_frame; _current_frame < last_frame; _current_frame++)
    {
//        timeval tim;
//        gettimeofday(&tim, NULL);
//        double tu = tim.tv_sec+(tim.tv_usec/1000000.0);
//        dt += ( tu - tu_last );
//        tu_last = tu;
        dt = _motion_recorders[0]->get_dt(0, _current_frame);
        usleep(dt*1000000.0);

        for (int j=0; j<int(_motion_recorders.size()); j++)
        {
//                cout << "configuration " << i << endl;
            _motion_recorders[j]->setRobotToStoredMotionConfig(0,_current_frame);
            //features_->printPosition();
        }
//        graphptrs_.clear();

//        std::vector<OpenRAVE::RaveVector<float> > vpoints;
//        std::vector<float> vcolors;

//        OpenRAVE::RaveVector<float> pnt(0,0,0);
//        vpoints.push_back(pnt);
//        vcolors.push_back(1);
//        vcolors.push_back(0);
//        vcolors.push_back(0);

//        graphptrs_.push_back( env_->plot3( &vpoints[0].x, vpoints.size(), sizeof(vpoints[0]), 0.05, &vcolors[0], 1 ) );

        bool draw = true;
        if (draw)
        {
            features_->bufferVelocity( dt );
            features_->bufferPosition();
            if (_current_frame > 0)
            {
                move3d_draw_clear_handles();
                Eigen::Vector3d anchor = features_->getPosBuffer()[_current_frame][0];
                Eigen::Vector3d vel = features_->getVelBuffer()[_current_frame][0];
                Eigen::Vector3d p2 = anchor+vel;

                double color[4] = {1,0,0,1};

                move3d_draw_one_line( anchor[0], anchor[1], anchor[2], p2[0], p2[1], p2[2], Any, color );
            }
        }


    }

    _StopRun = true;

    cout << "End play motion" << endl;
    return;
}

void PlayMotion::runStatistics()
{
    cout << "Running statistics on supplied trajectories" << endl;

    if( _motion_recorders.empty() || !usingMove3D ) {
        return;
    }

    _current_frame=0;
    int nb_frames = _motion_recorders[0]->getStoredMotions()[0].size();  //Assumes all motion recorders have the same # of frames.  Should probably take the highest
    Eigen::VectorXd times(nb_frames);
    float total_time = 0.0;

    Move3D::Robot* human1 = Move3D::global_Project->getActiveScene()->getRobotByName("human_model");
    Move3D::Trajectory traj( human1 );

    std::vector<double> dt_tmp;

    //Run over entire motion buffering velocity/pos data at each frame
    for (_current_frame; _current_frame < nb_frames; _current_frame++)
    {
        for (int j=0; j<int(_motion_recorders.size()); j++)
        {
            _motion_recorders[j]->setRobotToStoredMotionConfig(0,_current_frame);
        }

        traj.push_back( human1->getCurrentPos() );

        //Get time stamp
        double dt = _motion_recorders[0]->get_dt(0, _current_frame);

        dt_tmp.push_back( dt );

//        cout << "dt : " << dt << endl;
        total_time += dt;
        times(_current_frame) = total_time;
//        cout << "times(_current_frame) : " << times(_current_frame) << endl;

        features_->bufferDistance();
        features_->bufferVelocity( dt );
        features_->bufferPosition();
        //features_->bufferCollision();
    }

//    double s = 0.0;
//    double s_max = traj.getParamMax();
//    int n_step = nb_frames;
//    double step = s_max / nb_frames;

//    for ( int i=0; i<n_step; i++ )
//    {
//        human1->setAndUpdate( *traj.configAtParam(s) );

//        features_->bufferDistance();
//        features_->bufferVelocity( dt_tmp[i] );

//        s += step;
//    }

    cout << "End computing features, saving to file" << endl;
//    cout << "Pos size: " << features_->getPosBuffer()[0][0].size() << endl;
//    return;

    std::vector<Eigen::VectorXd> features_dist = features_->getDistBuffer();
    std::vector< std::vector<Eigen::Vector3d> > features_vel = features_->getVelBuffer();
    std::vector< std::vector<Eigen::Vector3d> > features_pos = features_->getPosBuffer();
    vel_t smoothed = features_->getSmoothedVelBuffer();
    std::vector< std::vector<double> > features_curv = features_->getCurviture(smoothed);
    std::vector< std::vector<double> > features_speed = features_->getSpeed(smoothed);

    if( (!features_dist.empty()) || (!features_vel.empty()) || (!features_pos.empty()) || (!features_curv.empty()) || (!features_speed.empty()) || nb_frames < 1 )
    {
        const char* home = getenv("HOME_MOVE3D");
        static int fileNum = 0;

        /*--------------------DIST--------------------*/
        int nb_features_dist = features_dist[0].size();
        Eigen::MatrixXd mat_dist( 1 + nb_features_dist, nb_frames );

        mat_dist.row(0) = times;

        for (int i=0; i < nb_features_dist; i++)
            for (int j=0; j < nb_frames; j++)
                mat_dist(i+1,j) = features_dist[j](i);

        cout << "rows : " << mat_dist.rows() << " , cols " << mat_dist.cols() << endl;

        if( home ){
            std::ostringstream file_name;
            file_name << std::string(home);
            file_name << "/../move3d-launch/matlab/quan_motion/hrics_feature_dist_";
            file_name << fileNum;
            file_name << ".csv";


            move3d_save_matrix_to_csv_file( mat_dist, file_name.str() );
        }

        /*--------------------VEL--------------------*/
        int nb_features_vel = features_vel[0].size();
        Eigen::MatrixXd mat_vel( 1 + 3*nb_features_vel, nb_frames );

        mat_vel.row(0) = times;

        for (int i=0; i < nb_features_vel; i++)
            for (int j=0; j < nb_frames; j++)
                for (int k=0; k < 3; k++)
                    mat_vel(1+3*i+k,j) = features_vel[j][i](k);

        cout << "rows : " << mat_vel.rows() << " , cols " << mat_vel.cols() << endl;

        if( home ){
            std::ostringstream file_name;
            file_name << std::string(home);
            file_name << "/../move3d-launch/matlab/quan_motion/hrics_feature_vel_";
            file_name << fileNum;
            file_name << ".csv";

            move3d_save_matrix_to_csv_file( mat_vel, file_name.str() );
        }

        /*--------------------CURV--------------------*/
        int nb_features_curv = features_curv[0].size();
        Eigen::MatrixXd mat_curv( 1 + nb_features_curv, nb_frames );

        mat_curv.row(0) = times;

        for (int i=0; i < nb_features_curv; i++)
            for (int j=0; j < nb_frames; j++)
                    mat_curv(i+1,j) = features_curv[j][i];

        cout << "rows : " << mat_curv.rows() << " , cols " << mat_curv.cols() << endl;

        if( home ){
            std::ostringstream file_name;
            file_name << std::string(home);
            file_name << "/../move3d-launch/matlab/quan_motion/hrics_feature_curv_";
            file_name << fileNum;
            file_name << ".csv";

            move3d_save_matrix_to_csv_file( mat_curv, file_name.str() );
        }

        /*--------------------SPEED--------------------*/
        int nb_features_speed = features_speed[0].size();
        Eigen::MatrixXd mat_speed( 1 + nb_features_speed, nb_frames );

        mat_speed.row(0) = times;

        for (int i=0; i < nb_features_speed; i++)
            for (int j=0; j < nb_frames; j++)
                    mat_speed(i+1,j) = features_speed[j][i];

        cout << "rows : " << mat_speed.rows() << " , cols " << mat_speed.cols() << endl;

        if( home ){
            std::ostringstream file_name;
            file_name << std::string(home);
            file_name << "/../move3d-launch/matlab/quan_motion/hrics_feature_speed_";
            file_name << fileNum;
            file_name << ".csv";

            move3d_save_matrix_to_csv_file( mat_speed, file_name.str() );
        }

        /*--------------------POS--------------------*/
        int nb_features_pos = features_pos[0].size();
        Eigen::MatrixXd mat_pos( 1 + 3*nb_features_pos, nb_frames );

        mat_pos.row(0) = times;

        for (int i=0; i < nb_features_pos; i++)
            for (int j=0; j < nb_frames; j++)
                for (int k=0; k < 3; k++)
                    mat_pos(1+3*i+k,j) = features_pos[j][i](k);

        cout << "rows : " << mat_pos.rows() << " , cols " << mat_pos.cols() << endl;

        if( home ){
            std::ostringstream file_name;
            file_name << std::string(home);
            file_name << "/../move3d-launch/matlab/quan_motion/hrics_feature_pos_";
            file_name << fileNum;
            file_name << ".csv";

            move3d_save_matrix_to_csv_file( mat_pos, file_name.str() );
        }
        fileNum++;
    }

    cout << "End play motion" << endl;
    return;
}

void PlayMotion::runControlled() //TODO weird implementation. should be fixed at some point, but definitely working.
{
    int numFrames = int(_motion_recorders[0]->getStoredMotions()[0].size());

    if( _motion_recorders.empty() )
    {
        return;
    }

    _current_frame = 0;
    _recent_input = false;

    while ( !_StopRun )
    {

        for (int j=0; j<int(_motion_recorders.size()); j++)
        {
            _motion_recorders[j]->setRobotToStoredMotionConfig(0,_current_frame);
        }

        if (_recent_input) _current_frame+=_step_size; //I don't it working like this.  Shouldn't constantly be updatin to current frame.

        if (_current_frame >= numFrames) _current_frame = numFrames; //Bounds check
        if (_current_frame < 0) _current_frame = 0; //Bounds check

        if ( _current_frame >= numFrames)
        {
            _StopRun = true;
        }

        _recent_input = false;

    }

    cout << "End play motion" << endl;
    return;
}

void PlayMotion::runControlled_folder() //TODO weird implementation. should be fixed at some point, but definitely working.
{
    std::vector<motion_t> motion_cache = _motion_recorders[0]->getStoredMotions();

    for (int i = 0; i < int(motion_cache.size()); i++)
    {
        int numFrames = int(motion_cache[i].size());

        if( _motion_recorders.empty() )
        {
            return;
        }

        _current_frame = 0;
        _recent_input = false;

        while ( !_StopRun )
        {

            for (int j=0; j<int(_motion_recorders.size()); j++)
            {
                _motion_recorders[j]->setRobotToStoredMotionConfig(i,_current_frame);
            }

            if (_recent_input) _current_frame+=_step_size; //I don't it working like this.  Shouldn't constantly be updatin to current frame.

            if (_current_frame >= numFrames) _current_frame = numFrames; //Bounds check
            if (_current_frame < 0) _current_frame = 0; //Bounds check

            if ( _current_frame >= numFrames)
            {
                _StopRun = true;
            }

            _recent_input = false;

        }

        cout << "End of motion:" << i << endl;
    }
    return;
}

void PlayMotion::reset_recorders()
{
    _StopRun = true;

    for (int i=0; i<int(_motion_recorders.size()); i++)
    {
        _motion_recorders[i]->reset();
    }
}

void PlayMotion::replay_trajectory()
{
    _current_frame = 0;
    if ( _play_type == 0 && _StopRun)
        runRealTime();
    if ( _play_type == 1 && _StopRun)
    {
        _StopRun = false;
        runControlled();
    }
}
