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
                    const GrippersDataVector& gripper_data,
                    const std::vector< GripperTrajectory >& gripper_trajectories,
                    const std::vector< kinematics::VectorVector6d >& gripper_velocities,
                    const ObjectTrajectory& object_trajectory,
                    const kinematics::VectorMatrix3Xd& object_velocities )
            {
                doUpdateModel( gripper_data,
                        gripper_trajectories,
                        gripper_velocities,
                        object_trajectory,
                        object_velocities );
            }

            ObjectTrajectory getPrediction(
                    const ObjectPointSet& object_configuration,
                    const std::vector< GripperTrajectory >& gripper_trajectories,
                    const std::vector< kinematics::VectorVector6d >& gripper_velocities ) const
            {
                return doGetPrediction( object_configuration, gripper_trajectories, gripper_velocities );
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
                    const GrippersDataVector& gripper_data,
                    const std::vector< GripperTrajectory >& gripper_trajectories,
                    const std::vector< kinematics::VectorVector6d >& gripper_velocities,
                    const ObjectTrajectory& object_trajectory,
                    const kinematics::VectorMatrix3Xd& object_velocities ) = 0;

            virtual ObjectTrajectory doGetPrediction(
                    const ObjectPointSet& object_configuration,
                    const std::vector< GripperTrajectory > & gripper_trajectories,
                    const std::vector< kinematics::VectorVector6d >& gripper_velocities ) const = 0;

            virtual void doPerturbModel( std::mt19937_64& generator ) = 0;
    };

}

#endif
