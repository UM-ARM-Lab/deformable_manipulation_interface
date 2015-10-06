#ifndef diminishing_rigidity_model_h
#define diminishing_rigidity_model_h

#include "smmap/deformable_model.h"

namespace smmap
{
    class DiminishingRigidityModel : public DeformableModel
    {
        public:
            DiminishingRigidityModel() {}
            ~DiminishingRigidityModel() {}

        private:
            void doGetPrediction();
            void doPerturbModel();
    };
}

#endif
