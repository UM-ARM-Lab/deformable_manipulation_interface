#ifndef diminishing_rigidity_model_h
#define diminishing_rigidity_model_h

#include "smmap/deformable_model.h"

namespace smmap
{
    class DiminishingRigidityModel : public DeformableModel
    {
        typedef std::shared_ptr< DiminishingRigidityModel > Ptr;

        public:
            ////////////////////////////////////////////////////////////////////
            /// Constructors and Destructor
            ////////////////////////////////////////////////////////////////////

            DiminishingRigidityModel( const ObjectPointSet& object_initial_configuration, double k = 0.5 );
            DiminishingRigidityModel( const ObjectPointSet& object_initial_configuration, double k_translation, double k_rotation );

        private:

            ////////////////////////////////////////////////////////////////////
            /// Virtual function overrides
            ////////////////////////////////////////////////////////////////////

            ObjectTrajectory doGetPrediction(
                    const ObjectPointSet& object_configuration,
                    const std::vector< GripperTrajectory >& gripper_trajectories,
                    const std::vector< kinematics::VectorVector6d >& gripper_velocities ) const;

            void doPerturbModel( std::mt19937_64& generator );

            ////////////////////////////////////////////////////////////////////
            /// Static members
            ////////////////////////////////////////////////////////////////////

            static std::normal_distribution< double > perturbation_distribution;

            ////////////////////////////////////////////////////////////////////
            /// Private members
            ////////////////////////////////////////////////////////////////////

            const ObjectPointSet object_initial_configuration_;
            double k_translation_;
            double k_rotation_;

    };
}

#endif
