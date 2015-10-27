#include "smmap/planner.h"
#include "smmap/trajectory.h"

#include <assert.h>
#include <boost/thread.hpp>
#include <ros/callback_queue.h>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/pretty_print.hpp>

using namespace smmap;

Planner::Planner( ros::NodeHandle& nh,
        CustomScene::TaskType task,
        const std::string& cmd_gripper_traj_topic,
        const std::string& simulator_fbk_topic,
        const std::string& get_gripper_names_topic )
    : task_( task )
    , nh_( nh )
    , fbk_buffer_initialized_( false )
{
    // Subscribe to feedback channels
    simulator_fbk_sub_ = nh_.subscribe(
            simulator_fbk_topic, 20, &Planner::simulatorFbkCallback, this );

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

void Planner::run( double loop_rate )
{
    ROS_INFO( "Starting feedback spinner" );
    // TODO: remove this hardcoded spin rate
    boost::thread spin_thread( boost::bind( &Planner::spin, 1000 ) );

    // Initialize the trajectory command message
    ROS_INFO( "Initializing gripper command message" );
    static deform_simulator::GripperTrajectoryStamped msg;
    msg.gripper_names = gripper_names_;
    msg.trajectories.resize( msg.gripper_names.size() );
    for ( auto& traj: msg.trajectories )
    {
        traj.pose.resize( 5 );
        traj.pose[ traj.pose.size() - 1 ].position.x = -0.6;
        traj.pose[ traj.pose.size() - 1 ].position.y = 0.0;
        traj.pose[ traj.pose.size() - 1 ].position.z = 0.75;
        traj.pose[ traj.pose.size() - 1 ].orientation.x = 0;
        traj.pose[ traj.pose.size() - 1 ].orientation.y = 0;
        traj.pose[ traj.pose.size() - 1 ].orientation.z = 0;
        traj.pose[ traj.pose.size() - 1 ].orientation.w = 1;
    }

    // Run the planner at whatever rate we've been given
    ROS_INFO( "Running our planner" );
    ros::Rate rate( loop_rate );
    while ( ros::ok() )
    {
        // Publish a trajectory to follow
        auto& pose = msg.trajectories[0].pose;
        pose[0] = pose[ pose.size() - 1 ];
        for ( size_t i = 1; i < pose.size(); i++ )
        {
            pose[i].position.y = pose[i-1].position.y + 0.001;
        }

        msg.header.stamp = ros::Time::now();
        cmd_gripper_traj_pub_.publish( msg );
        rate.sleep();

        boost::mutex::scoped_lock lock( input_mtx_ );
        // TODO: determine if it's time to replan
        if ( simulator_fbk_buffer_.size() >= 2 )
        {
            // collect all the data and update the models
            updateModels( lock );

            // now replan?
        }
    }

    ROS_INFO( "Terminating" );
    spin_thread.join();
}

////////////////////////////////////////////////////////////////////////////////
// Input data parsing and model management
////////////////////////////////////////////////////////////////////////////////

void Planner::updateModels( boost::mutex::scoped_lock& lock )
{
    // first read in all the input data, we assume that data is already locked
    // TODO: smart mutex locking to ensure this
    if ( !lock )
    {
        ROS_WARN( "updateModels called without a lock already in place, this code was not designed to handle that situation." );
        lock.lock();
    }

    // convert the data over to Eigen format
    ROS_INFO( "Number of data points: %lu", simulator_fbk_buffer_.size() );

    // TODO: move this to a helper function
    // first we allocate space
    ObjectTrajectory object_trajectory( simulator_fbk_buffer_.size() );
    // TODO: we assume here that the order of the gripper trajectories matches
    // the order in gripper_names_
    std::vector< GripperTrajectory > gripper_trajectories( gripper_names_.size() );
    for ( size_t ind = 0; ind < gripper_names_.size(); ind++ )
    {
        gripper_trajectories[ind].resize( simulator_fbk_buffer_.size() );
    }

    // then we do the actual conversion to Eigen
    for ( size_t ind = 0; ind < simulator_fbk_buffer_.size() ; ind++ )
    {
        object_trajectory[ind] = EigenHelpers::VectorGeometryPointToEigenMatrix3Xd( simulator_fbk_buffer_[ind].object_configuration );

        for ( size_t gripper_ind = 0; gripper_ind < gripper_names_.size(); gripper_ind++ )
        {
            gripper_trajectories[gripper_ind][ind] = EigenHelpers::GeometryPoseToEigenAffine3d( simulator_fbk_buffer_[ind].gripper_poses[gripper_ind] );
        }
    }

    // clear the buffers now that we've consumed the data
    auto last_fbk = simulator_fbk_buffer_[ simulator_fbk_buffer_.size() - 1 ];
    simulator_fbk_buffer_.clear();
    simulator_fbk_buffer_.push_back( last_fbk );

    lock.unlock();

    model_set_.evaluateAccuracy( gripper_trajectories, object_trajectory );
    model_set_.updateModels( gripper_trajectories, object_trajectory );
}

////////////////////////////////////////////////////////////////////////////////
// ROS Callbacks
////////////////////////////////////////////////////////////////////////////////

void Planner::simulatorFbkCallback(
        const deform_simulator::SimulatorFbkStamped& fbk )
{
    boost::mutex::scoped_lock lock( input_mtx_ );

    // If we've already received at least one data element, it's business as usual
    if ( fbk_buffer_initialized_ )
    {
        // if this data arrived out of order, discard it
        // TODO: do something smart instead of discarding
        if ( fbk.header.seq > simulator_fbk_buffer_[ simulator_fbk_buffer_.size() - 1 ].header.seq )
        {
            simulator_fbk_buffer_.push_back( fbk );
        }
        else
        {
            ROS_WARN( "Out of sequence data, dropping message %u", fbk.header.seq );
        }
    }
    // otherwise initialize some data
    else
    {
        fbk_buffer_initialized_ = true;
        simulator_fbk_buffer_.clear();
        simulator_fbk_buffer_.push_back( fbk );
    }
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
