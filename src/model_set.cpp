#include "smmap/model_set.h"

#include <cmath>
#include <chrono>
#include <assert.h>

#include <kinematics_toolbox/kinematics.h>

#include "smmap/diminishing_rigidity_model.h"

using namespace smmap;

ModelSet::ModelSet( const GrippersDataVector& grippers_data,
        const ObjectPointSet& object_initial_configuration )
    : grippers_data_( grippers_data )
    , object_initial_configuration_( object_initial_configuration )
    , rnd_generator_( std::chrono::system_clock::now().time_since_epoch().count() )
{
    // 0 is totally rigid (weight is 1), 2 is loose (weight is e^-2*dist)
    for ( double k = 0; k <= 2; k += 0.1 )
    {
        addModel( DeformableModel::Ptr( new DiminishingRigidityModel(
                        grippers_data, object_initial_configuration_, k ) ) );
    }
}

ModelSet::~ModelSet()
{}

void ModelSet::makePredictions(
        const std::vector< GripperTrajectory >& gripper_trajectories,
        const ObjectPointSet& object_configuration ) const
{
    //TODO: do
    assert("THIS FUNCTION IS NOT YET IMPLEMENTED!" && false);
}

void ModelSet::updateModels(
        const std::vector< GripperTrajectory >& gripper_trajectories,
        const ObjectTrajectory& object_trajectory )
{
    assert( gripper_trajectories.size() > 0 );
    assert( object_trajectory.size() == gripper_trajectories[0].size() );

    // Do some math to calculate velocities for the grippers and the object
    std::vector< kinematics::VectorVector6d > gripper_velocities =
        calculateGripperVelocities( gripper_trajectories );

    kinematics::VectorMatrix3Xd object_velocities =
            calculateObjectVelocities( object_trajectory );

    // Evaluate our confidence in each model
    evaluateConfidence( gripper_trajectories, gripper_velocities, object_trajectory );

    // Allow each model to update itself based on the new data
    for ( auto& model: model_list_ )
    {
        model->updateModel( grippers_data_,
                gripper_trajectories,
                gripper_velocities,
                object_trajectory,
                object_velocities );
    }
}

const std::vector< double >& ModelSet::getModelConfidence() const
{
    return model_confidence_;
}

////////////////////////////////////////////////////////////////////////////////
// Private helpers
////////////////////////////////////////////////////////////////////////////////

void ModelSet::evaluateConfidence(
        const std::vector< GripperTrajectory >& gripper_trajectories,
        const std::vector< kinematics::VectorVector6d >& gripper_velocities,
        const ObjectTrajectory& object_trajectory )
{
    // TODO: deal with the object/gripers not moving at all
    for ( size_t ind = 0; ind < model_list_.size(); ind++ )
    {
        const ObjectTrajectory model_prediction = model_list_[ind]->getPrediction(
                object_trajectory[0], gripper_trajectories, gripper_velocities );

        const double dist = distance(  object_trajectory, model_prediction );

        model_confidence_[ind] = 1.0 / ( 1 + 5*dist );
    }
}

std::vector< kinematics::VectorVector6d > ModelSet::calculateGripperVelocities(
        const std::vector< GripperTrajectory >& gripper_trajectories ) const
{
    std::vector< kinematics::VectorVector6d > gripper_velocities( gripper_trajectories.size() );
    for ( size_t ind = 0; ind < gripper_trajectories.size(); ind++ )
    {
        gripper_velocities[ind] = kinematics::calculateVelocities( gripper_trajectories[ind] );
    }
    return gripper_velocities;
}

kinematics::VectorMatrix3Xd ModelSet::calculateObjectVelocities(
        const ObjectTrajectory& object_trajectory ) const
{
    kinematics::VectorMatrix3Xd object_velocities( object_trajectory.size() - 1, object_trajectory[0] );

    for ( size_t ind = 0; ind < object_trajectory.size() - 1; ind ++ )
    {
        object_velocities[ind] = object_trajectory[ind + 1] - object_trajectory[ind];
    }

    return object_velocities;
}

void ModelSet::addModel( DeformableModel::Ptr model )
{
    assert( model_list_.size() == model_confidence_.size() );

    model_list_.push_back( model );
    model_confidence_.push_back( 0 );
}
