#ifndef deformable_model_h
#define deformable_model_h

#include <memory>
#include <random>

#include <kinematics_toolbox/kinematics.h>

#include "smmap/trajectory.h"

namespace smmap
{
    class DeformableModel
    {
        public:
            typedef std::shared_ptr< DeformableModel > Ptr;

            ////////////////////////////////////////////////////////////////////
            /// Wrappers for virtual functions
            ////////////////////////////////////////////////////////////////////

            void updateModel(
                    const VectorGrippersData& gripper_data,
                    const AllGrippersTrajectory& grippers_trajectory,
                    const std::vector< kinematics::VectorVector6d >& grippers_velocities,
                    const ObjectTrajectory& object_trajectory,
                    const kinematics::VectorMatrix3Xd& object_velocities )
            {
                doUpdateModel( gripper_data, grippers_trajectory,
                        grippers_velocities, object_trajectory, object_velocities );
            }

            ObjectTrajectory getPrediction(
                    const ObjectPointSet& object_configuration,
                    const AllGrippersTrajectory& grippers_trajectory,
                    const std::vector< kinematics::VectorVector6d >& grippers_velocities ) const
            {
                return doGetPrediction( object_configuration, grippers_trajectory, grippers_velocities );
            }

            AllGrippersTrajectory getDesiredGrippersTrajectory(
                    const ObjectPointSet& object_current_configuration,
                    const ObjectPointSet& object_desired_configuration,
                    EigenHelpers::VectorAffine3d grippers_pose,
                    double max_step, size_t num_steps ) const
            {
                return doGetDesiredGrippersTrajectory( object_current_configuration,
                        object_desired_configuration, grippers_pose, max_step, num_steps );
            }

            void perturbModel( std::mt19937_64& generator )
            {
                doPerturbModel( generator );
            }

        protected:

            ////////////////////////////////////////////////////////////////////
            /// Destructor that prevents "delete pointer to base object"
            ////////////////////////////////////////////////////////////////////

            ~DeformableModel() {}

        private:

            ////////////////////////////////////////////////////////////////////
            /// Virtual functions that need to be overridden by derived classes
            ////////////////////////////////////////////////////////////////////

            virtual void doUpdateModel(
                    const VectorGrippersData& gripper_data,
                    const AllGrippersTrajectory& grippers_trajectory,
                    const std::vector< kinematics::VectorVector6d >& grippers_velocities,
                    const ObjectTrajectory& object_trajectory,
                    const kinematics::VectorMatrix3Xd& object_velocities ) = 0;

            virtual ObjectTrajectory doGetPrediction(
                    const ObjectPointSet& object_configuration,
                    const AllGrippersTrajectory& grippers_trajectory,
                    const std::vector< kinematics::VectorVector6d >& grippers_velocities ) const = 0;

            virtual AllGrippersTrajectory doGetDesiredGrippersTrajectory(
                    const ObjectPointSet& object_current_configuration,
                    const ObjectPointSet& object_desired_configuration,
                    EigenHelpers::VectorAffine3d grippers_pose,
                    double max_step, size_t num_steps ) const = 0;

            virtual void doPerturbModel( std::mt19937_64& generator ) = 0;
    };

}

#endif
