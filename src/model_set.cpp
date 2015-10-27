#include "smmap/model_set.h"

#include <chrono>
#include <assert.h>

using namespace smmap;

ModelSet::ModelSet()
    : rnd_generator( new std::mt19937_64( std::chrono::system_clock::now().time_since_epoch().count() ) )
{}

ModelSet::~ModelSet()
{
}

void ModelSet::addModel( const DeformableModel::Ptr& m )
{
    model_list.push_front( m );
}

void ModelSet::evaluateAccuracy(
        const std::vector< GripperTrajectory >& gripper_trajectories,
        const ObjectTrajectory& object_trajectory )
{
}

void ModelSet::updateModels(
        const std::vector< GripperTrajectory >& gripper_trajectories,
        const ObjectTrajectory& object_trajectory )
{
}
