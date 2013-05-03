#include "skeletonListener.hpp"

class SkeletonDrawing
{
public :
    static void drawLineSegmentModel( OpenRAVE::EnvironmentBasePtr env, std::vector<boost::shared_ptr<void> >& graph, const std::vector<Eigen::Vector3d>& pos );
};

