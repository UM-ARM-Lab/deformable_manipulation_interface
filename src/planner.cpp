#include "smmap/planner.h"

#include <boost/thread.hpp>
#include <ros/callback_queue.h>
#include <arc_utilities/pretty_print.hpp>

using namespace smmap;

Planner::Planner( ros::NodeHandle& nh,
        CustomScene::TaskType task,
        std::string cmd_gripper_traj_topic,
        std::string gripper_pose_topic,
        std::string object_configuration_topic,
        std::string get_gripper_names_topic )
    : task_( task )
    , nh_( nh )
{
    // Subscribe to feedback channels
    gripper_pose_sub_ = nh_.subscribe(
            gripper_pose_topic, 5, &Planner::gripperPoseCallback, this );

    object_configuration_sub_ = nh_.subscribe(
            object_configuration_topic, 5, &Planner::objectConfigurationCallback, this );

    // Publish to the desired gripper trajectory channel
    cmd_gripper_traj_pub_ = nh_.advertise< deform_simulator::GripperTrajectoryStamped >(
            cmd_gripper_traj_topic, 1 );

    // Find out the names of grippers in the world
    ros::ServiceClient gripper_names_client =
        nh_.serviceClient< deform_simulator::GetGripperNames >( get_gripper_names_topic );

    gripper_names_client.waitForExistence();

    deform_simulator::GetGripperNames srv_data;
    gripper_names_client.call( srv_data );
    gripper_names_ = srv_data.response.names;

    ROS_INFO( "Gripper names: %s", PrettyPrint::PrettyPrint( gripper_names_ ).c_str() );
}

////////////////////////////////////////////////////////////////////////////////
// Main function that makes things happen
////////////////////////////////////////////////////////////////////////////////

void Planner::run()
{
    ROS_INFO( "Starting feedback spinner" );
    boost::thread spin_thread( boost::bind( &Planner::spin, 1000 ) );

    // Initialize the trajectory command message
    ROS_INFO( "Initializing gripper command message" );
    static deform_simulator::GripperTrajectoryStamped msg;
    msg.gripper_names = gripper_names_;
    msg.trajectories.resize( msg.gripper_names.size() );
    for (auto& traj: msg.trajectories)
    {
        traj.pose.resize(1);
        traj.pose[0].position.x = -0.6;
        traj.pose[0].position.y = 0;
        traj.pose[0].position.z = 0.75;
        traj.pose[0].orientation.x = 0;
        traj.pose[0].orientation.y = 0;
        traj.pose[0].orientation.z = 0;
        traj.pose[0].orientation.w = 1;
    }

    ROS_INFO( "Running our planner" );
    // run our planner at 100 Hz
    ros::Rate loop_rate( 100 );
    while ( ros::ok() )
    {
        msg.header.stamp = ros::Time::now();
        msg.trajectories[0].pose[0].position.y += 0.002;
        cmd_gripper_traj_pub_.publish( msg );
        loop_rate.sleep();
    }

    ROS_INFO( "Terminating" );
    spin_thread.join();
}

////////////////////////////////////////////////////////////////////////////////
// ROS Callbacks
////////////////////////////////////////////////////////////////////////////////

void Planner::gripperPoseCallback(
        const deform_simulator::GripperPoseStamped& gripper_pose )
{
    //TODO: do
}

void Planner::objectConfigurationCallback(
        const deform_simulator::ObjectConfigurationStamped& object_configuration )
{
    //TODO: do
}

////////////////////////////////////////////////////////////////////////////////
// ROS Objects and Helpers
////////////////////////////////////////////////////////////////////////////////

void Planner::spin( double loop_rate )
{
    ros::NodeHandle nh;
    while ( ros::ok() )
    {
        ros::getGlobalCallbackQueue()->callAvailable( ros::WallDuration( loop_rate ) );
    }
}
