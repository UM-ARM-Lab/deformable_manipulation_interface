#ifndef model_set_h
#define model_set_h

#include <list>
#include <memory>
#include <random>

#include "smmap/deformable_model.h"

namespace smmap
{
    class ModelSet
    {
        public:
            ModelSet();
            ~ModelSet();

            void addModel( const DeformableModel::Ptr& m );
            void makePredictions( const GripperTrajectory& gripper_trajectory );
            void evaluateAccuracy(
                    const std::vector< GripperTrajectory >& gripper_trajectories,
                    const ObjectTrajectory& object_trajectory );
            void updateModels(
                    const std::vector< GripperTrajectory >& gripper_trajectories,
                    const ObjectTrajectory& object_trajectory );

        private:
            std::list< DeformableModel::Ptr > model_list;

            std::shared_ptr< std::mt19937_64 > rnd_generator;
    };
}

#endif
