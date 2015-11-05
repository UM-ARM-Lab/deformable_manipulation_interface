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
            ////////////////////////////////////////////////////////////////////
            // Constructor and destructor
            ////////////////////////////////////////////////////////////////////

            ModelSet( const VectorGrippersData& grippers_data,
                    const ObjectPointSet& object_initial_configuration );
            ~ModelSet();

            void updateModels(
                    const AllGrippersTrajectory& grippers_trajectory,
                    const ObjectTrajectory& object_trajectory );

            void makePredictions(
                    const AllGrippersTrajectory& grippers_trajectory,
                    const ObjectPointSet& object_configuration ) const;

            std::vector< std::pair< AllGrippersTrajectory, double > > getDesiredGrippersTrajectories(
                    const ObjectPointSet& object_current_configuration,
                    const ObjectPointSet& object_desired_configuration,
                    EigenHelpers::VectorAffine3d grippers_pose,
                    double max_step, size_t num_steps );

            const std::vector< double >& getModelConfidence() const;

        private:
            ////////////////////////////////////////////////////////////////////
            // Private helpers
            ////////////////////////////////////////////////////////////////////

            void addModel( DeformableModel::Ptr model );

            std::vector< kinematics::VectorVector6d > calculateGrippersVelocities(
                    const AllGrippersTrajectory& grippers_trajectory ) const;

            kinematics::VectorMatrix3Xd calculateObjectVelocities(
                    const ObjectTrajectory& object_trajectory ) const;

            void evaluateConfidence(
                    const AllGrippersTrajectory& grippers_trajectory,
                    const std::vector< kinematics::VectorVector6d >& grippers_velocities,
                    const ObjectTrajectory& object_trajectory );

            // TODO: move this to *somewhere* else
            const VectorGrippersData grippers_data_;
            const ObjectPointSet object_initial_configuration_;
            std::vector< DeformableModel::Ptr > model_list_;
            std::vector< double > model_confidence_;

            std::mt19937_64 rnd_generator_;
    };
}

#endif
