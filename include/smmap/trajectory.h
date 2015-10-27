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

    inline double distance( const ObjectTrajectory& traj1, const ObjectTrajectory& traj2 )
    {
        assert( traj1.size() == traj2.size() );
        double dist = 0;
        for ( size_t ind = 0; ind < traj1.size(); ind++ )
        {
            dist += (traj1[ind] - traj2[ind]).squaredNorm();
        }

        return sqrt(dist);
    }
}

#endif
