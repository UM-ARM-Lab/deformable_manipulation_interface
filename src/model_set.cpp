#include "smmap/model_set.h"

#include <chrono>

using namespace smmap;

ModelSet::ModelSet()
    : rnd_generator( new std::mt19937_64( std::chrono::system_clock::now().time_since_epoch().count() ) )
{}

ModelSet::~ModelSet()
{
}

void ModelSet::addModel( const DeformableModel::Ptr& m )
{
    model_list.push_front( m );
}
