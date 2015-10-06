#ifndef deformable_model_h
#define deformable_model_h

namespace smmap
{

class DeformableModel
{
    public:
        inline void getPrediction() { return doGetPrediction(); }
        inline void perturbModel() { return doPerturbModel(); }

    protected:
        ~DeformableModel() {}

    private:
        virtual void doGetPrediction() = 0;
        virtual void doPerturbModel() = 0;
};

}

#endif
