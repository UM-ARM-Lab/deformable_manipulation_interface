#include "smmap/model_set.h"

#include <cmath>
#include <chrono>
#include <assert.h>

#include <kinematics_toolbox/kinematics.h>

#include "smmap/diminishing_rigidity_model.h"

using namespace smmap;

ModelSet::ModelSet( const VectorGrippersData& grippers_data,
        const ObjectPointSet& object_initial_configuration )
    : grippers_data_( grippers_data )
    , object_initial_configuration_( object_initial_configuration )
    , rnd_generator_( std::chrono::system_clock::now().time_since_epoch().count() )
{
    // 0 is totally rigid (weight is 1), 2 is loose (weight is e^-2*dist)
    for ( double k = 0; k <= 5; k += 0.1 )
    {
        addModel( DeformableModel::Ptr( new DiminishingRigidityModel(
                        grippers_data, object_initial_configuration_, k ) ) );
    }
}

ModelSet::~ModelSet()
{}

void ModelSet::makePredictions(
        const AllGrippersTrajectory& grippers_trajectory,
        const ObjectPointSet& object_configuration ) const
{
    //TODO: do
    assert("THIS FUNCTION IS NOT YET IMPLEMENTED!" && false);
}

void ModelSet::updateModels(
        const AllGrippersTrajectory& grippers_trajectory,
        const ObjectTrajectory& object_trajectory )
{
    assert( grippers_trajectory.size() > 0 );
    assert( object_trajectory.size() == grippers_trajectory[0].size() );

    // Do some math to calculate velocities for the grippers and the object
    std::vector< kinematics::VectorVector6d > grippers_velocities =
        calculateGrippersVelocities( grippers_trajectory );

    kinematics::VectorMatrix3Xd object_velocities =
            calculateObjectVelocities( object_trajectory );

    // Evaluate our confidence in each model
    evaluateConfidence( grippers_trajectory, grippers_velocities, object_trajectory );

    // Allow each model to update itself based on the new data
    for ( auto& model: model_list_ )
    {
        model->updateModel( grippers_data_,
                grippers_trajectory,
                grippers_velocities,
                object_trajectory,
                object_velocities );
    }
}

std::vector< std::pair< AllGrippersTrajectory, double > > ModelSet::getDesiredGrippersTrajectories(
        const ObjectPointSet& object_current_configuration,
        const ObjectPointSet& object_desired_configuration,
        EigenHelpers::VectorAffine3d grippers_pose,
        double max_step, size_t num_steps )
{
    std::vector< std::pair< AllGrippersTrajectory, double > > grippers_trajectories;

    for ( size_t ind = 0; ind < model_list_.size(); ind++ )
    {
        AllGrippersTrajectory grippers_trajectory = model_list_[ind]->getDesiredGrippersTrajectory(
                    object_current_configuration, object_desired_configuration,
                    grippers_pose, max_step, num_steps );

        grippers_trajectories.push_back( std::pair< AllGrippersTrajectory, double >( grippers_trajectory, model_confidence_[ind] ) );
    }

    return grippers_trajectories;
}

const std::vector< double >& ModelSet::getModelConfidence() const
{
    return model_confidence_;
}

////////////////////////////////////////////////////////////////////////////////
// Private helpers
////////////////////////////////////////////////////////////////////////////////

void ModelSet::evaluateConfidence(
        const AllGrippersTrajectory& grippers_trajectory,
        const std::vector< kinematics::VectorVector6d >& grippers_velocities,
        const ObjectTrajectory& object_trajectory )
{
    // TODO: deal with the object/gripers not moving at all
    for ( size_t ind = 0; ind < model_list_.size(); ind++ )
    {
        const ObjectTrajectory model_prediction = model_list_[ind]->getPrediction(
                object_trajectory[0], grippers_trajectory, grippers_velocities );

        const double dist = distance(  object_trajectory, model_prediction );

        model_confidence_[ind] = 1.0 / ( 1 + 5*dist );
    }
}

std::vector< kinematics::VectorVector6d > ModelSet::calculateGrippersVelocities(
        const AllGrippersTrajectory& grippers_trajectory ) const
{
    std::vector< kinematics::VectorVector6d > grippers_velocities( grippers_trajectory.size() );
    for ( size_t ind = 0; ind < grippers_trajectory.size(); ind++ )
    {
        grippers_velocities[ind] = kinematics::calculateVelocities( grippers_trajectory[ind] );
    }
    return grippers_velocities;
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
