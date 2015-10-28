#include "smmap/model_set.h"

#include <chrono>
#include <assert.h>

#include <kinematics_toolbox/kinematics.h>

#include "smmap/diminishing_rigidity_model.h"

using namespace smmap;

ModelSet::ModelSet( const ObjectPointSet& object_initial_configuration )
    : object_initial_configuration_( object_initial_configuration )
    , rnd_generator_( std::chrono::system_clock::now().time_since_epoch().count() )
{
    for ( double k = 0.1; k <= 1; k += 0.1 )
    {
        addModel( DeformableModel::Ptr( new DiminishingRigidityModel(
                        object_initial_configuration_, k ) ) );
    }
}

ModelSet::~ModelSet()
{}

void ModelSet::evaluateConfidence(
        const std::vector< GripperTrajectory >& gripper_trajectories,
        const ObjectTrajectory& object_trajectory )
{
    assert( gripper_trajectories.size() > 0 );
    assert( object_trajectory.size() == gripper_trajectories[0].size() );

    // First step is to convert gripper_trajectories into velocities, as many
    // models may need this
    std::vector< kinematics::VectorVector6d > gripper_velocities( gripper_trajectories.size() );
    for ( size_t ind = 0; ind < gripper_trajectories.size(); ind++ )
    {
        gripper_velocities[ind] = kinematics::calculateVelocities( gripper_trajectories[ind] );
    }

    for ( size_t ind = 0; ind < model_list_.size(); ind++ )
    {
        model_confidence_[ind] = 1.0 / ( 1 + 50*distance(  object_trajectory,
                    model_list_[ind]->getPrediction( object_trajectory[0],
                        gripper_trajectories, gripper_velocities ) ) );
    }

    std::cout << "Confidences: " << PrettyPrint::PrettyPrint( model_confidence_ ) << std::endl;
}

void ModelSet::updateModels(
        const std::vector< GripperTrajectory >& gripper_trajectories,
        const ObjectTrajectory& object_trajectory )
{
    assert( gripper_trajectories.size() > 0 );
    assert( object_trajectory.size() == gripper_trajectories[0].size() );
}

void ModelSet::addModel( DeformableModel::Ptr model )
{
    assert( model_list_.size() == model_confidence_.size() );

    model_list_.push_back( model );
    model_confidence_.push_back( 0 );
}
