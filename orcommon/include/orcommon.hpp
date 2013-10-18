#ifndef ORCOMMON_HPP
#define ORCOMMON_HPP

#include <openrave/openrave.h>
#include <Eigen/Dense>


typedef std::vector< double > confPtr_t;
typedef std::vector< std::pair<double,confPtr_t> > motion_t;


namespace HRICS {

    template <typename T>
std::string num_to_string ( T Number )
{
    std::ostringstream ss;
    ss << Number;
    return ss.str();
}

template <class T>
bool string_to_num(T& t,
                   const std::string& s,
                   std::ios_base& (*f)(std::ios_base&))
{
    std::istringstream iss(s);
    return !(iss >> f >> t).fail();
}

Eigen::MatrixXd motiont_to_eigen(const motion_t motion)
{
    Eigen::MatrixXd result;
    if (motion.empty())
        return result;

    result.setZero(motion.size(), motion[0].second.size()+1);

    for (int i=0; i < motion.size(); i++)
    {
        result(i, 0) = i; //Make sure there is an index
        for(int j=1; j < motion[i].second.size(); j++)
        {
            result(i,j) = motion[i].second[j];
        }

    }

    return result;
}

Eigen::Vector3d or_vector_to_eigen(const OpenRAVE::Vector& pos)
{
    Eigen::Vector3d p;
    p[0] = pos.x;
    p[1] = pos.y;
    p[2] = pos.z;
    return p;
}

Eigen::Vector3d or_vector_to_eigen(const std::vector<double>& pos)
{
    Eigen::Vector3d p;
    p[0] = pos[0];
    p[1] = pos[1];
    p[2] = pos[2];
    return p;
}

std::vector<double> eigen_vector_to_or(const Eigen::Vector3d& pos)
{
    std::vector<double> p( 3 );
    p[0] = pos[0];
    p[1] = pos[1];
    p[2] = pos[2];
    return p;
}

Eigen::Affine3d get_joint_transform(OpenRAVE::KinBody::JointPtr joint)
{
    OpenRAVE::RaveTransformMatrix<double> t( joint->GetFirstAttached()->GetTransform() );
    OpenRAVE::Vector right,up,dir,pos;
    t.Extract( right, up, dir, pos);
    Eigen::Matrix3d rot;
    rot(0,0) = right.x; rot(0,1) = up.x; rot(0,2) = dir.x;
    rot(1,0) = right.y; rot(1,1) = up.y; rot(1,2) = dir.y;
    rot(2,0) = right.z; rot(2,1) = up.z; rot(2,2) = dir.z;

    Eigen::Affine3d T;
    T.linear() = rot;
    T.translation() = or_vector_to_eigen( joint->GetAnchor() );

    //cout << "T : " << endl << T.matrix() << endl;
    return T;
}

double angle_limit_PI(double angle){

    while (angle < -M_PI){
        angle += 2*M_PI;
    }
    while (angle > M_PI){
        angle -= 2*M_PI;
    }
    return angle;
}

void print_config(const std::vector<double>& q)
{
    for( int i=0;i<int(q.size());i++)
    {
        std::cout << "q[" << i << "] = " << q[i] << std::endl;
    }
}
};

#endif // SKELETONLISTENER_HPP

