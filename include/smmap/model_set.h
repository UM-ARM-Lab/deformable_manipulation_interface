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
            ModelSet( const ObjectPointSet& object_initial_configuration );
            ~ModelSet();

            void makePredictions(
                    const GripperTrajectory& gripper_trajectory );
            void evaluateConfidence(
                    const std::vector< GripperTrajectory >& gripper_trajectories,
                    const ObjectTrajectory& object_trajectory );
            void updateModels(
                    const std::vector< GripperTrajectory >& gripper_trajectories,
                    const ObjectTrajectory& object_trajectory );

        private:
            void addModel( DeformableModel::Ptr model );

            const ObjectPointSet object_initial_configuration_;
            std::vector< DeformableModel::Ptr > model_list_;
            std::vector< double > model_confidence_;

            std::mt19937_64 rnd_generator_;
    };
}

#endif
