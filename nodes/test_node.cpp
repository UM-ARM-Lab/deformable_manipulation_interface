#include "smmap/model_set.h"
#include "smmap/diminishing_rigidity_model.h"

using namespace smmap;
using std::shared_ptr;

void populateInitialModelSet( ModelSet& model_set, const ObjectPointSetPtr& point_set );

int main()
{
    // Simulator sim;
    // ObjectHandle rope = sim.addObject('rope');

    // we have 5 points in our object, all uninitialized
    shared_ptr< ObjectPointSet > object_starting_points(
            new ObjectPointSet( 3, 5 ) );

    ModelSet model_set;

    populateInitialModelSet( model_set, object_starting_points );

    return 0;
}

void populateInitialModelSet( ModelSet& model_set, const ObjectPointSetPtr& point_set )
{
    DeformableModel::Ptr m( new DiminishingRigidityModel( point_set ) );
    model_set.addModel( m );
}
