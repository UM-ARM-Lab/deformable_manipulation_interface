#ifndef diminishing_rigidity_model_h
#define diminishing_rigidity_model_h

#include "smmap/deformable_model.h"

namespace smmap
{
    class DiminishingRigidityModel : public DeformableModel
    {
//        typedef std::shared_ptr< DiminishingRigidityModel > Ptr;

        public:
            ////////////////////////////////////////////////////////////////////
            // Constructors and Destructor
            ////////////////////////////////////////////////////////////////////

            DiminishingRigidityModel( const VectorGrippersData& grippers_data,
                    const ObjectPointSet& object_initial_configuration,
                    double k = 0.5 );

            DiminishingRigidityModel( const VectorGrippersData& grippers_data,
                    const ObjectPointSet& object_initial_configuration,
                    double k_translation, double k_rotation );

        private:

            ////////////////////////////////////////////////////////////////////
            // Constructor helpers
            ////////////////////////////////////////////////////////////////////

            void computeObjectNodeDistanceMatrix();

            ////////////////////////////////////////////////////////////////////
            // Virtual function overrides
            ////////////////////////////////////////////////////////////////////

            void doUpdateModel(
                    const VectorGrippersData& gripper_data,
                    const AllGrippersTrajectory& grippers_trajectory,
                    const std::vector< kinematics::VectorVector6d >& grippers_velocities,
                    const ObjectTrajectory& object_trajectory,
                    const kinematics::VectorMatrix3Xd& object_velocities );

            ObjectTrajectory doGetPrediction(
                    const ObjectPointSet& object_configuration,
                    const AllGrippersTrajectory& grippers_trajectory,
                    const std::vector< kinematics::VectorVector6d >& gripper_velocities ) const;

            AllGrippersTrajectory doGetDesiredGrippersTrajectory(
                    const ObjectPointSet &object_current_configuration,
                    const ObjectPointSet &object_desired_configuration,
                    EigenHelpers::VectorAffine3d grippers_pose,
                    double max_step, size_t num_steps ) const;

            void doPerturbModel( std::mt19937_64& generator );

            ////////////////////////////////////////////////////////////////////
            // Model update parameters
            ////////////////////////////////////////////////////////////////////

            void computeJacobian();

            ////////////////////////////////////////////////////////////////////
            // Static members
            ////////////////////////////////////////////////////////////////////

            static std::normal_distribution< double > perturbation_distribution;
            static Eigen::MatrixXd object_initial_node_distance_;

            ////////////////////////////////////////////////////////////////////
            // Private members
            ////////////////////////////////////////////////////////////////////

            const VectorGrippersData grippers_data_;
            const ObjectPointSet object_initial_configuration_;
            double k_translation_;
            double k_rotation_;
            Eigen::MatrixXd J_;

    };
}

#endif // diminishing_rigidity_model_h
