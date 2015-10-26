#include "smmap/planner.h"

using namespace smmap;

int main( int argc, char* argv[] )
{
    // Read in all ROS parameters
    ros::init( argc, argv, "custom_scene", ros::init_options::NoSigintHandler );

    ros::NodeHandle nh;

    Planner planner( nh );
    planner.run();

    return 0;
}
