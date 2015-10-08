#ifndef diminishing_rigidity_model_h
#define diminishing_rigidity_model_h

#include "smmap/deformable_model.h"

namespace smmap
{
    class DiminishingRigidityModel : public DeformableModel
    {
        public:
            typedef std::shared_ptr< DiminishingRigidityModel > Ptr;

            ////////////////////////////////////////////////////////////////////
            /// Constructors and Destructor
            ////////////////////////////////////////////////////////////////////

            DiminishingRigidityModel( ObjectPointSetPtr starting_points_,
                    double k_translation = 0.5 );
            DiminishingRigidityModel( ObjectPointSetPtr starting_points_,
                    double k_translation, double k_rotation );

        private:

            ////////////////////////////////////////////////////////////////////
            /// Virtual function overrides
            ////////////////////////////////////////////////////////////////////

            ObjectTrajectory doGetPrediction(
                    const GripperTrajectory& gripper_traj_ ) const;

            void doPerturbModel( const std::shared_ptr< std::mt19937_64 >& generator );

            ////////////////////////////////////////////////////////////////////
            /// Static members
            ////////////////////////////////////////////////////////////////////

            static std::normal_distribution< double > perturbation_distribution;

            ////////////////////////////////////////////////////////////////////
            /// Private members
            ////////////////////////////////////////////////////////////////////

            const ObjectPointSetPtr starting_points;
            double k_translation;
            double k_rotation;

    };
}

#endif
