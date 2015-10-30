#include "smmap/diminishing_rigidity_model.h"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <ros/ros.h>

using namespace smmap;

////////////////////////////////////////////////////////////////////////////////
// Constructors and Destructor
////////////////////////////////////////////////////////////////////////////////

DiminishingRigidityModel::DiminishingRigidityModel(
        const GrippersDataVector& grippers_data,
        const ObjectPointSet& object_initial_configuration, double k )
    : DiminishingRigidityModel( grippers_data, object_initial_configuration, k, k )
{}

DiminishingRigidityModel::DiminishingRigidityModel(
        const GrippersDataVector& grippers_data,
        const ObjectPointSet& object_initial_configuration, double k_translation, double k_rotation )
    : grippers_data_( grippers_data )
    , object_initial_configuration_( object_initial_configuration )
    , k_translation_( k_translation )
    , k_rotation_( k_rotation )
{
    if ( k_translation <= 0 )
    {
        throw new std::invalid_argument("k_translation must be greater than 0");
    }
    if ( k_rotation <= 0 )
    {
        throw new std::invalid_argument("k_rotation must be greater than 0");
    }

    computeObjectNodeDistanceMatrix();
    computeJacobian();
}

////////////////////////////////////////////////////////////////////////////////
// Constructor helpers
////////////////////////////////////////////////////////////////////////////////

void DiminishingRigidityModel::computeObjectNodeDistanceMatrix()
{
    // TODO: replace this ugly hack
    if ( object_initial_node_distance_.size() == 0 )
    {
        ROS_INFO( "Computing object initial distance matrix" );

        const size_t num_nodes = object_initial_configuration_.cols();
        object_initial_node_distance_.resize( num_nodes, num_nodes );
        for ( size_t i = 0; i < num_nodes; i++ )
        {
            for ( size_t j = i; j < num_nodes; j++ )
            {
                object_initial_node_distance_( i, j ) =
                    ( object_initial_configuration_.block< 3, 1>( 0, i )
                    - object_initial_configuration_.block< 3, 1>( 0, j ) ).norm();
                object_initial_node_distance_( j, i ) = object_initial_node_distance_( i, j );
            }
        }
    }
}

void DiminishingRigidityModel::computeJacobian()
{
    ROS_INFO( "Computing object Jacobian" );

    const size_t num_grippers = grippers_data_.size();
    const size_t num_Jcols = 6*num_grippers;

    const size_t num_nodes = object_initial_configuration_.cols();
    const size_t num_Jrows = 3*object_initial_configuration_.cols();

    J_.resize( num_Jrows, num_Jcols );

    // for each gripper
    for ( size_t gripper_ind = 0; gripper_ind < num_grippers; gripper_ind++ )
    {
        const std::vector< size_t >& gripper_node_indices = grippers_data_[gripper_ind].second;
        for ( size_t node_ind = 0; node_ind < num_nodes; node_ind++ )
        {
            const std::pair< size_t, double > dist_to_gripper
                = getMinimumDistanceToGripper( gripper_node_indices, node_ind,
                        object_initial_configuration_ );

            Eigen::Matrix3d J_trans = Eigen::Matrix3d::Identity();
            J_trans = std::exp( -k_translation_ * dist_to_gripper.second ) * J_trans;

            // TODO: do
            Eigen::Matrix3d J_rot = Eigen::Matrix3d::Zero();

            J_.block< 3, 3 >( node_ind * 3, gripper_ind * 6 ) = J_trans;
            J_.block< 3, 3 >( node_ind * 3, gripper_ind * 6 + 3 ) = J_rot;
        }
    }

}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
// Virtual function overrides
////////////////////////////////////////////////////////////////////////////////

void DiminishingRigidityModel::doUpdateModel(
        const GrippersDataVector& gripper_data,
        const std::vector< GripperTrajectory >& gripper_trajectories,
        const std::vector< kinematics::VectorVector6d >& gripper_velocities,
        const ObjectTrajectory& object_trajectory,
        const kinematics::VectorMatrix3Xd& object_velocities )
{
}

ObjectTrajectory DiminishingRigidityModel::doGetPrediction(
        const ObjectPointSet& object_configuration,
        const std::vector< GripperTrajectory>& gripper_trajectories,
        const std::vector< kinematics::VectorVector6d >& gripper_velocities ) const
{
    assert( gripper_trajectories.size() > 0 );
    assert( gripper_velocities.size() == gripper_trajectories.size() );

    ObjectTrajectory object_traj( gripper_trajectories[0].size(), object_configuration );

    for ( size_t vel_ind = 0; vel_ind < gripper_velocities[0].size(); vel_ind++ )
    {
        Eigen::MatrixXd combined_gripper_vel( gripper_velocities.size()*6, 1 );
        for ( size_t gripper_ind = 0; gripper_ind < gripper_velocities.size(); gripper_ind++ )
        {
            combined_gripper_vel.block< 6, 1 >( gripper_ind * 6, 0 ) =
                gripper_velocities[gripper_ind][vel_ind];
        }

        Eigen::MatrixXd delta_obj = J_*combined_gripper_vel;
        delta_obj.conservativeResize( 3, object_configuration.cols() );

        object_traj[vel_ind + 1] = object_traj[vel_ind] + delta_obj;
    }

    return object_traj;
}

void DiminishingRigidityModel::doPerturbModel( std::mt19937_64& generator )
{
    k_translation_ += perturbation_distribution( generator );
    k_rotation_ += perturbation_distribution( generator );

    if ( k_translation_ <= 0 )
    {
        k_translation_ = std::numeric_limits< double >::epsilon();
    }
    if ( k_rotation_ <= 0 )
    {
        k_rotation_ = std::numeric_limits< double >::epsilon();
    }
}

// Static member initialization
////////////////////////////////////////////////////////////////////////////////

/// TODO: remove this magic number for the noise generated by this distribution
std::normal_distribution< double > DiminishingRigidityModel::perturbation_distribution =
    std::normal_distribution< double >( 0, 0.1 );

Eigen::MatrixXd DiminishingRigidityModel::object_initial_node_distance_;
