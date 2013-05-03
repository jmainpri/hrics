#include "skeletonDrawing.hpp"

//! @ingroup KINECT
//! Draw line segment
void SkeletonDrawing::drawLineSegmentModel( OpenRAVE::EnvironmentBasePtr env, std::vector<boost::shared_ptr<void> >& graph, const std::vector<Eigen::Vector3d>& pos )
{
//    HEAD = 0,
//    NECK = 1,
//    TORSO = 2,

//    SHOULDER_LEFT = 3,
//    ELBOW_LEFT = 4,
//    HAND_LEFT = 5,

//    SHOULDER_RIGHT = 6,
//    ELBOW_RIGHT = 7,
//    HAND_RIGHT = 8,

//    KNEE_RIGHT = 9,
//    HIP_LEFT = 10,
//    FOOT_LEFT = 11,

//    KNEE_LEFT = 12,
//    HIP_RIGHT = 13,
//    FOOT_RIGHT = 14

    float colors[] = {1,0,0};


    int nb_points = 28;

    float* ppoints = new float[3*nb_points];

    int i=0;

    ppoints[i+0] = pos[HEAD][0];
    ppoints[i+1] = pos[HEAD][1];
    ppoints[i+2] = pos[HEAD][2];

    i += 3;

    ppoints[i+0] = pos[NECK][0];
    ppoints[i+1] = pos[NECK][1];
    ppoints[i+2] = pos[NECK][2];

    i += 3;

    ppoints[i+0] = pos[SHOULDER_LEFT][0];
    ppoints[i+1] = pos[SHOULDER_LEFT][1];
    ppoints[i+2] = pos[SHOULDER_LEFT][2];

    i += 3;

    ppoints[i+0] = pos[NECK][0];
    ppoints[i+1] = pos[NECK][1];
    ppoints[i+2] = pos[NECK][2];

    i += 3;

    ppoints[i+0] = pos[SHOULDER_RIGHT][0];
    ppoints[i+1] = pos[SHOULDER_RIGHT][1];
    ppoints[i+2] = pos[SHOULDER_RIGHT][2];

    i += 3;

    ppoints[i+0] = pos[NECK][0];
    ppoints[i+1] = pos[NECK][1];
    ppoints[i+2] = pos[NECK][2];


    i += 3;

    ppoints[i+0] = pos[SHOULDER_LEFT][0];
    ppoints[i+1] = pos[SHOULDER_LEFT][1];
    ppoints[i+2] = pos[SHOULDER_LEFT][2];

    i += 3;

    ppoints[i+0] = pos[ELBOW_LEFT][0];
    ppoints[i+1] = pos[ELBOW_LEFT][1];
    ppoints[i+2] = pos[ELBOW_LEFT][2];

    i += 3;

    ppoints[i+0] = pos[ELBOW_LEFT][0];
    ppoints[i+1] = pos[ELBOW_LEFT][1];
    ppoints[i+2] = pos[ELBOW_LEFT][2];

    i += 3;

    ppoints[i+0] = pos[HAND_LEFT][0];
    ppoints[i+1] = pos[HAND_LEFT][1];
    ppoints[i+2] = pos[HAND_LEFT][2];

    i += 3;

    ppoints[i+0] = pos[SHOULDER_RIGHT][0];
    ppoints[i+1] = pos[SHOULDER_RIGHT][1];
    ppoints[i+2] = pos[SHOULDER_RIGHT][2];

    i += 3;

    ppoints[i+0] = pos[ELBOW_RIGHT][0];
    ppoints[i+1] = pos[ELBOW_RIGHT][1];
    ppoints[i+2] = pos[ELBOW_RIGHT][2];

    i += 3;

    ppoints[i+0] = pos[ELBOW_RIGHT][0];
    ppoints[i+1] = pos[ELBOW_RIGHT][1];
    ppoints[i+2] = pos[ELBOW_RIGHT][2];

    i += 3;

    ppoints[i+0] = pos[HAND_RIGHT][0];
    ppoints[i+1] = pos[HAND_RIGHT][1];
    ppoints[i+2] = pos[HAND_RIGHT][2];

    i += 3;

    ppoints[i+0] = pos[NECK][0];
    ppoints[i+1] = pos[NECK][1];
    ppoints[i+2] = pos[NECK][2];

    i += 3;

    ppoints[i+0] = pos[TORSO][0];
    ppoints[i+1] = pos[TORSO][1];
    ppoints[i+2] = pos[TORSO][2];

    i += 3;

    ppoints[i+0] = pos[TORSO][0];
    ppoints[i+1] = pos[TORSO][1];
    ppoints[i+2] = pos[TORSO][2];

    i += 3;

    ppoints[i+0] = pos[HIP_LEFT][0];
    ppoints[i+1] = pos[HIP_LEFT][1];
    ppoints[i+2] = pos[HIP_LEFT][2];

    i += 3;

    ppoints[i+0] = pos[TORSO][0];
    ppoints[i+1] = pos[TORSO][1];
    ppoints[i+2] = pos[TORSO][2];

    i += 3;

    ppoints[i+0] = pos[HIP_RIGHT][0];
    ppoints[i+1] = pos[HIP_RIGHT][1];
    ppoints[i+2] = pos[HIP_RIGHT][2];

    i += 3;

    ppoints[i+0] = pos[HIP_RIGHT][0];
    ppoints[i+1] = pos[HIP_RIGHT][1];
    ppoints[i+2] = pos[HIP_RIGHT][2];

    i += 3;

    ppoints[i+0] = pos[KNEE_RIGHT][0];
    ppoints[i+1] = pos[KNEE_RIGHT][1];
    ppoints[i+2] = pos[KNEE_RIGHT][2];

    i += 3;

    ppoints[i+0] = pos[KNEE_RIGHT][0];
    ppoints[i+1] = pos[KNEE_RIGHT][1];
    ppoints[i+2] = pos[KNEE_RIGHT][2];

    i += 3;

    ppoints[i+0] = pos[FOOT_RIGHT][0];
    ppoints[i+1] = pos[FOOT_RIGHT][1];
    ppoints[i+2] = pos[FOOT_RIGHT][2];

    i += 3;

    ppoints[i+0] = pos[HIP_LEFT][0];
    ppoints[i+1] = pos[HIP_LEFT][1];
    ppoints[i+2] = pos[HIP_LEFT][2];

    i += 3;

    ppoints[i+0] = pos[KNEE_LEFT][0];
    ppoints[i+1] = pos[KNEE_LEFT][1];
    ppoints[i+2] = pos[KNEE_LEFT][2];

    i += 3;

    ppoints[i+0] = pos[KNEE_LEFT][0];
    ppoints[i+1] = pos[KNEE_LEFT][1];
    ppoints[i+2] = pos[KNEE_LEFT][2];

    i += 3;

    ppoints[i+0] = pos[FOOT_LEFT][0];
    ppoints[i+1] = pos[FOOT_LEFT][1];
    ppoints[i+2] = pos[FOOT_LEFT][2];

    OpenRAVE::GraphHandlePtr fig = env->drawlinelist( ppoints, nb_points, 3*sizeof(float), 3.0, colors);

    delete ppoints;

    graph.push_back( fig );

    //cout << "sizeof(float) : " << sizeof(float) << endl;
}
