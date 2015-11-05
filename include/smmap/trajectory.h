#ifndef trajectory_h
#define trajectory_h

#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <limits>
#include <memory>
#include <tuple>
#include <exception>

namespace smmap
{
    typedef Eigen::Matrix3Xd ObjectPointSet;
    typedef std::vector< ObjectPointSet, Eigen::aligned_allocator<Eigen::Matrix3Xd> > ObjectTrajectory;
    typedef EigenHelpers::VectorAffine3d GripperTrajectory;

    // TODO: move this somewhere else
    struct GripperData
    {
        GripperData( const Eigen::Affine3d& tf, const std::vector< size_t >& inds, const std::string& n )
            : transform( tf )
            , node_indices( inds )
            , name( n )
        {}

        friend std::ostream& operator<< ( std::ostream& out, const GripperData& data )
        {
            out << data.name << " " << PrettyPrint::PrettyPrint( data.node_indices ) << std::endl
                << data.transform.matrix() << std::endl;
            return out;
        }

        Eigen::Affine3d transform;
        std::vector< size_t > node_indices;
        std::string name;
    };
    typedef std::vector< GripperData > GrippersDataVector;

    // TODO what about normalizing for the trajectory length?
    // Currently returns the RMS distance
    inline double distance( const ObjectTrajectory& traj1, const ObjectTrajectory& traj2 )
    {
        assert( traj1.size() == traj2.size() );
        double dist = 0;
        for ( size_t ind = 0; ind < traj1.size(); ind++ )
        {
            dist += (traj1[ind] - traj2[ind]).squaredNorm();
        }

        return std::sqrt( dist / traj1.size() );
    }

    // TODO: move this somewhere else
    inline std::pair< size_t, double >getMinimumDistanceToGripper(
            const std::vector< size_t >& gripper_indices, size_t node_index,
            const Eigen::MatrixXd& object_initial_node_distance )
    {
        double min_dist = std::numeric_limits< double >::infinity();
        size_t min_ind = 0;

        for ( size_t ind: gripper_indices )
        {
            if ( object_initial_node_distance( ind, node_index ) < min_dist )
            {
                min_dist = object_initial_node_distance( ind, node_index );
                min_ind = ind;
            }
        }

        return std::pair< size_t, double>( min_ind, min_dist );
    }

}

#endif
