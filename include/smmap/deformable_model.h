#ifndef deformable_model_h
#define deformable_model_h

#include <memory>
#include <random>

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
                    const GripperTrajectory& gripper_traj_ ) const
            {
                return doGetPrediction( gripper_traj_ );
            }

            void perturbModel( const std::shared_ptr< std::mt19937_64 >& generator )
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
                    const GripperTrajectory & gripper_traj_ ) const = 0;

            virtual void doPerturbModel( const std::shared_ptr< std::mt19937_64 >& generator ) = 0;
    };

}

#endif
