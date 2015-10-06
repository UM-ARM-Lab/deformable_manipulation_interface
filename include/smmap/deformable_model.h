#ifndef deformable_model_h
#define deformable_model_h

namespace smmap
{

class DeformableModel
{
    public:
        void getPrediction() { return doGetPrediction(); }

    private:
        virtual void doGetPrediction() = 0;
};

}

#endif
