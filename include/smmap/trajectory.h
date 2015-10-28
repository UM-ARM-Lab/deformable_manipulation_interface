#ifndef trajectory_h
#define trajectory_h

#include <arc_utilities/eigen_helpers.hpp>
#include <memory>

namespace smmap
{
    typedef Eigen::Matrix3Xd ObjectPointSet;
    typedef std::vector< ObjectPointSet, Eigen::aligned_allocator<Eigen::Matrix3Xd> > ObjectTrajectory;
    typedef EigenHelpers::VectorAffine3d GripperTrajectory;

    inline double distance( const ObjectTrajectory& traj1, const ObjectTrajectory& traj2 )
    {
        assert( traj1.size() == traj2.size() );
        double dist = 0;
        for ( size_t ind = 0; ind < traj1.size(); ind++ )
        {
            dist += (traj1[ind] - traj2[ind]).squaredNorm();
        }

        return std::sqrt(dist);
    }
}

#endif
