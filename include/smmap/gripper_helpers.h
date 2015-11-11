#ifndef gripper_helpers_h
#define gripper_helpers_h

#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <limits>
#include <memory>

namespace smmap
{
    struct GripperData
    {
        GripperData( const Eigen::Affine3d& pose, const std::vector< size_t >& node_indices, const std::string& name )
            : pose( pose )
            , node_indices( node_indices )
            , name( name )
        {}

        friend std::ostream& operator<< ( std::ostream& out, const GripperData& data )
        {
            out << data.name << " Num Indices: " << PrettyPrint::PrettyPrint( data.node_indices )
                << " " << PrettyPrint::PrettyPrint( data.pose );
            return out;
        }

        Eigen::Affine3d pose;
        std::vector< size_t > node_indices;
        std::string name;
    };
    typedef std::vector< GripperData, Eigen::aligned_allocator<Eigen::Matrix4d> > VectorGrippersData;


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

#endif // gripper_helpers_h
