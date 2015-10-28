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

            ObjectTrajectory getPrediction(
                    const ObjectPointSet& object_configuration,
                    const std::vector< GripperTrajectory >& gripper_trajectories,
                    const std::vector< kinematics::VectorVector6d >& gripper_velocities ) const
            {
                return doGetPrediction( object_configuration, gripper_trajectories, gripper_velocities );
            }

            void perturbModel( std::mt19937_64& generator )
            {
                return doPerturbModel( generator );
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

            virtual ObjectTrajectory doGetPrediction(
                    const ObjectPointSet& object_configuration,
                    const std::vector< GripperTrajectory > & gripper_trajectories,
                    const std::vector< kinematics::VectorVector6d >& gripper_velocities ) const = 0;

            virtual void doPerturbModel( std::mt19937_64& generator ) = 0;
    };

}

#endif
