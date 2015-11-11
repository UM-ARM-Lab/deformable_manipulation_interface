#ifndef trajectory_h
#define trajectory_h

#include <arc_utilities/eigen_helpers.hpp>

namespace smmap
{
    typedef Eigen::Matrix3Xd ObjectPointSet;
    typedef std::vector< ObjectPointSet, Eigen::aligned_allocator< Eigen::Matrix3Xd > > ObjectTrajectory;

    typedef EigenHelpers::VectorAffine3d SingleGripperTrajectory;
    typedef std::vector< SingleGripperTrajectory, Eigen::aligned_allocator< SingleGripperTrajectory > > AllGrippersTrajectory;

    inline EigenHelpers::VectorAffine3d getLastGrippersPose( const AllGrippersTrajectory& grippers_trajectory )
    {
        EigenHelpers::VectorAffine3d last_poses( grippers_trajectory.size() );
        for ( size_t gripper_ind = 0; gripper_ind < grippers_trajectory.size(); gripper_ind++ )
        {
            last_poses[gripper_ind] = grippers_trajectory[gripper_ind].back();
        }
        return last_poses;
    }

    inline double distance( const ObjectPointSet& set1, const ObjectPointSet& set2 )
    {
        return ( set1 - set2 ).norm();
    }

    inline double distanceSquared( const ObjectPointSet& set1, const ObjectPointSet& set2 )
    {
        return ( set1 - set2 ).squaredNorm();
    }

    inline double distanceRMS( const ObjectTrajectory& traj1, const ObjectTrajectory& traj2 )
    {
        assert( traj1.size() == traj2.size() );
        double dist_squared = 0;
        for ( size_t ind = 0; ind < traj1.size(); ind++ )
        {
            dist_squared += distanceSquared( traj1[ind], traj2[ind]);
        }

        return std::sqrt( dist_squared / traj1.size() );
    }
}

#endif // trajectory_h
