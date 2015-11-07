#include "smmap/diminishing_rigidity_model.h"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <ros/ros.h>

using namespace smmap;

// TODO: find a way to accept dynamic/online gripper re-grasping

////////////////////////////////////////////////////////////////////////////////
// Constructors and Destructor
////////////////////////////////////////////////////////////////////////////////

DiminishingRigidityModel::DiminishingRigidityModel(
        const VectorGrippersData& grippers_data,
        const ObjectPointSet& object_initial_configuration, double k )
    : DiminishingRigidityModel( grippers_data, object_initial_configuration, k, k )
{}

DiminishingRigidityModel::DiminishingRigidityModel(
        const VectorGrippersData& grippers_data,
        const ObjectPointSet& object_initial_configuration, double k_translation, double k_rotation )
    : grippers_data_( grippers_data )
    , object_initial_configuration_( object_initial_configuration )
    , k_translation_( k_translation )
    , k_rotation_( k_rotation )
{
    if ( k_translation < 0 )
    {
        throw new std::invalid_argument("k_translation must be positive");
    }
    if ( k_rotation < 0 )
    {
        throw new std::invalid_argument("k_rotation must be positive");
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
    ROS_INFO( "Computing object Jacobian: Diminishing rigidity k_trans: %f k_rot: %f", k_translation_, k_rotation_ );

    const size_t num_grippers = grippers_data_.size();
    const size_t num_Jcols = 6*num_grippers;

    const size_t num_nodes = object_initial_configuration_.cols();
    const size_t num_Jrows = 3*object_initial_configuration_.cols();

    J_.resize( num_Jrows, num_Jcols );

    // for each gripper
    for ( size_t gripper_ind = 0; gripper_ind < num_grippers; gripper_ind++ )
    {
        // Get all the data we need for a given gripper
        const std::vector< size_t >& gripper_node_indices = grippers_data_[gripper_ind].node_indices;
        const Eigen::Matrix3d gripper_rot = grippers_data_[gripper_ind].pose.rotation();

        for ( size_t node_ind = 0; node_ind < num_nodes; node_ind++ )
        {
            const std::pair< size_t, double > dist_to_gripper
                = getMinimumDistanceToGripper( gripper_node_indices, node_ind,
                        object_initial_node_distance_ );

            Eigen::Matrix3d J_trans = Eigen::Matrix3d::Identity();
            Eigen::Matrix3d J_rot = Eigen::Matrix3d::Zero();
            // Vector from gripper to node
            const Eigen::Vector3d gripper_to_node =
                    object_initial_configuration_.block< 3, 1 >( 0, node_ind ) -
                    grippers_data_[gripper_ind].pose.translation();
            J_rot.block< 3, 1 >( 0, 0 ) = gripper_rot.block< 3, 1 >( 0, 0 ).cross( gripper_to_node );
            J_rot.block< 3, 1 >( 0, 1 ) = gripper_rot.block< 3, 1 >( 0, 1 ).cross( gripper_to_node );
            J_rot.block< 3, 1 >( 0, 2 ) = gripper_rot.block< 3, 1 >( 0, 2 ).cross( gripper_to_node );

            J_.block< 3, 3 >( node_ind * 3, gripper_ind * 6 ) =
                    std::exp( -k_translation_ * dist_to_gripper.second ) * J_trans;
            J_.block< 3, 3 >( node_ind * 3, gripper_ind * 6 + 3 ) =
                    std::exp( -k_rotation_ * dist_to_gripper.second ) * J_rot;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
// Virtual function overrides
////////////////////////////////////////////////////////////////////////////////

void DiminishingRigidityModel::doUpdateModel(
        const VectorGrippersData& grippers_data,
        const AllGrippersTrajectory& grippers_trajectory,
        const std::vector< kinematics::VectorVector6d >& grippers_velocities,
        const ObjectTrajectory& object_trajectory,
        const kinematics::VectorMatrix3Xd& object_velocities )
{
    // This model doesn't do any updates, so tell the compiler that it's okay
    // that these values are unused.
    (void)grippers_data;
    (void)grippers_trajectory;
    (void)grippers_velocities;
    (void)object_trajectory;
    (void)object_velocities;
}

ObjectTrajectory DiminishingRigidityModel::doGetPrediction(
        const ObjectPointSet& object_configuration,
        const AllGrippersTrajectory& grippers_trajectory,
        const std::vector< kinematics::VectorVector6d >& grippers_velocities ) const
{
    assert( grippers_trajectory.size() > 0 );
    assert( grippers_velocities.size() == grippers_trajectory.size() );

    ObjectTrajectory object_traj( grippers_trajectory[0].size(), object_configuration );

    for ( size_t vel_ind = 0; vel_ind < grippers_velocities[0].size(); vel_ind++ )
    {
        // create a 6g by 1 vector that contains all the gripper velocities at this time step
        Eigen::MatrixXd combined_gripper_vel( grippers_velocities.size()*6, 1 );
        for ( size_t gripper_ind = 0; gripper_ind < grippers_velocities.size(); gripper_ind++ )
        {
            combined_gripper_vel.block< 6, 1 >( gripper_ind * 6, 0 ) =
                grippers_velocities[gripper_ind][vel_ind];
        }

        // calculate the velocity of the object given the gripper velocity
        Eigen::MatrixXd delta_obj = J_*combined_gripper_vel;
        delta_obj.conservativeResize( 3, object_configuration.cols() );

        object_traj[vel_ind + 1] = object_traj[vel_ind] + delta_obj;
    }

    return object_traj;
}

AllGrippersTrajectory DiminishingRigidityModel::doGetDesiredGrippersTrajectory(
        const ObjectPointSet& object_current_configuration,
        const ObjectPointSet& object_desired_configuration,
        EigenHelpers::VectorAffine3d grippers_pose,
        double max_step, size_t num_steps ) const
{
    assert( grippers_pose.size() == grippers_data_.size() );
    // Initialize the starting point of the trajectory with the current gripper
    // poses
    AllGrippersTrajectory traj( grippers_pose.size() );
    for ( size_t gripper_ind = 0; gripper_ind < grippers_pose.size(); gripper_ind++ )
    {
        traj[gripper_ind].push_back( grippers_pose[gripper_ind] );
    }

    // TODO: get rid of this uglyness
    Eigen::MatrixXd tmp_current = object_current_configuration;
    Eigen::MatrixXd tmp_desired = object_desired_configuration;
    const Eigen::VectorXd desired = Eigen::Map< Eigen::VectorXd >( tmp_current.data(), object_desired_configuration.cols() * object_desired_configuration.rows() );
    const Eigen::VectorXd current = Eigen::Map< Eigen::VectorXd >( tmp_desired.data(), object_current_configuration.cols() * object_current_configuration.rows() );

//    std::cout << "Model with k = " << k_translation_ << std::endl;
//    std::cout << "Current Config:\n" << object_current_configuration << std::endl;
//    std::cout << "Current transformed:\n" << current.transpose() << std::endl;
//    std::cout << "Desired Config:\n" << object_desired_configuration << std::endl;
//    std::cout << "Desired transformed:\n" << desired.transpose() << std::endl;

    const Eigen::MatrixXd J_inv = EigenHelpers::Pinv( J_, EigenHelpers::SuggestedRcond() );

    for ( size_t traj_step = 1; traj_step <= num_steps; traj_step++ )
    {
        const Eigen::VectorXd object_delta = -(desired - current);
        Eigen::VectorXd combined_grippers_velocity = J_inv * object_delta;

//        std::cout << "object_delta: " << object_delta.transpose() << std::endl;
        if ( combined_grippers_velocity.norm() > max_step )
        {
            combined_grippers_velocity = combined_grippers_velocity / combined_grippers_velocity.norm() * max_step;
        }

        for ( size_t gripper_ind = 0; gripper_ind < grippers_data_.size(); gripper_ind++ )
        {
            kinematics::Vector6d gripper_velocity = combined_grippers_velocity.block< 6, 1 >( gripper_ind * 6, 0 );

            traj[gripper_ind].push_back( traj[gripper_ind][traj_step - 1] *
                    kinematics::expTwistAffine3d( gripper_velocity, 1 ) );
        }
    }

    return traj;
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
