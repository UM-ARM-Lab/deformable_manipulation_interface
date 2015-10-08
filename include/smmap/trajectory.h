#ifndef trajectory_h
#define trajectory_h

#include <arc_utilities/eigen_helpers.hpp>
#include <memory>

namespace smmap
{
    typedef Eigen::Matrix3Xd ObjectPointSet;
    typedef std::shared_ptr< ObjectPointSet > ObjectPointSetPtr;

    typedef std::vector< ObjectPointSet > ObjectTrajectory;
    typedef std::shared_ptr< ObjectTrajectory > ObjectTrajectoryPtr;

    typedef EigenHelpers::VectorAffine3d GripperTrajectory;
    typedef std::shared_ptr< GripperTrajectory > GripperTrajectoryPtr;

}

#endif
