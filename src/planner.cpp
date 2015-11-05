#include "smmap/planner.h"
#include "smmap/trajectory.h"

#include <algorithm>
#include <assert.h>
#include <boost/thread.hpp>
#include <ros/callback_queue.h>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/pretty_print.hpp>
#include <sensor_msgs/image_encodings.h>

using namespace smmap;
using namespace EigenHelpersConversions;

Planner::Planner( ros::NodeHandle& nh,
        CustomScene::TaskType task,
        const std::string& cmd_gripper_traj_topic,
        const std::string& simulator_fbk_topic,
        const std::string& get_gripper_names_topic,
        const std::string& get_gripper_attached_node_indices_topic,
        const std::string& get_object_initial_configuration_topic,
        const std::string& confidence_image_topic )
    : task_( task )
    , nh_( nh )
    , it_( nh_ )
    , fbk_buffer_initialized_( false )
{
    // Subscribe to feedback channels
    simulator_fbk_sub_ = nh_.subscribe(
            simulator_fbk_topic, 20, &Planner::simulatorFbkCallback, this );

    // Publish to the desired gripper trajectory channel
    cmd_gripper_traj_pub_ = nh_.advertise< deform_simulator::GripperTrajectoryStamped >(
            cmd_gripper_traj_topic, 1 );

    // Publish a confidence image
    confidence_image_pub_ = it_.advertise( confidence_image_topic, 1 );

    getGrippersData( get_gripper_names_topic, get_gripper_attached_node_indices_topic );
    getObjectInitialConfiguration( get_object_initial_configuration_topic );

    model_set_ = std::unique_ptr< ModelSet >(
            new ModelSet( gripper_data_, object_initial_configuration_ ) );
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

    msg.trajectories.resize( gripper_data_.size() );
    for ( size_t ind = 0; ind < gripper_data_.size(); ind++ )
    {
        msg.gripper_names.push_back( gripper_data_[ind].name );
        auto& traj = msg.trajectories[ind];
        traj.pose.resize( 20 );
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

// TODO: split this into multiple functions
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
    ROS_INFO( "Number of data points: %zu", simulator_fbk_buffer_.size() );

    // TODO: move this to a helper function
    // first we allocate space
    ObjectTrajectory object_trajectory( simulator_fbk_buffer_.size() );
    // TODO: we assume here that the order of the gripper trajectories matches
    // the order in gripper_names_
    std::vector< GripperTrajectory > gripper_trajectories( gripper_data_.size() );
    for ( size_t ind = 0; ind < gripper_data_.size(); ind++ )
    {
        gripper_trajectories[ind].resize( simulator_fbk_buffer_.size() );
    }

    // then we do the actual conversion to Eigen
    for ( size_t ind = 0; ind < simulator_fbk_buffer_.size() ; ind++ )
    {
        object_trajectory[ind] = VectorGeometryPointToEigenMatrix3Xd( simulator_fbk_buffer_[ind].object_configuration );

        for ( size_t gripper_ind = 0; gripper_ind < gripper_data_.size(); gripper_ind++ )
        {
            gripper_trajectories[gripper_ind][ind] = GeometryPoseToEigenAffine3d( simulator_fbk_buffer_[ind].gripper_poses[gripper_ind] );
        }
    }

    // clear the buffers now that we've consumed the data
    auto last_fbk = simulator_fbk_buffer_[ simulator_fbk_buffer_.size() - 1 ];
    simulator_fbk_buffer_.clear();
    simulator_fbk_buffer_.push_back( last_fbk );

    lock.unlock();

    model_set_->updateModels( gripper_trajectories, object_trajectory );
    const std::vector<double> model_confidence = model_set_->getModelConfidence();

    cv::Mat image( 1, model_confidence.size(), CV_8UC3 );

    const double min_conf = *std::min_element( model_confidence.begin(), model_confidence.end() );
    const double max_conf = *std::max_element( model_confidence.begin(), model_confidence.end() );

    for ( size_t ind = 0; ind < model_confidence.size(); ind++ )
    {
        //image.at< cv::Vec3b >( 0, ind )[0] = (model_confidence[ind] - 0.9) * 10.0 * 255.0;
        image.at< cv::Vec3b >( 0, ind )[0] = (model_confidence[ind] - min_conf) / ( max_conf - min_conf) * 255.0;
        image.at< cv::Vec3b >( 0, ind )[1] = 0;
        image.at< cv::Vec3b >( 0, ind )[2] = 0;
    }

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, image).toImageMsg();
    confidence_image_pub_.publish( msg );
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

void Planner::getGrippersData( const std::string& names_topic, const std::string& indices_topic )
{
    // Get the names of each gripper
    ros::ServiceClient gripper_names_client =
        nh_.serviceClient< deform_simulator::GetGripperNames >( names_topic );
    gripper_names_client.waitForExistence();

    deform_simulator::GetGripperNames srv_data;
    gripper_names_client.call( srv_data );
    std::vector< std::string > gripper_names = srv_data.response.names;

    // Get the attached nodes for each gripper
    ros::ServiceClient gripper_node_indices_client =
        nh_.serviceClient< deform_simulator::GetGripperAttachedNodeIndices >( indices_topic );
    gripper_node_indices_client.waitForExistence();
    for ( size_t ind = 0; ind < gripper_names.size(); ind++ )
    {
        deform_simulator::GetGripperAttachedNodeIndices srv_data;
        srv_data.request.name = gripper_names[ind];
        gripper_node_indices_client.call( srv_data );

        // TODO: get rid of this, currently used to ensure we have some data from the simulator
        boost::mutex::scoped_lock lock( input_mtx_ );
        while ( !fbk_buffer_initialized_ )
        {
            lock.unlock();
            usleep( 1000 );
            ros::spinOnce();
            lock.lock();
        }
        lock.unlock();
        gripper_data_.push_back( GripperData(
                    EigenHelpersConversions::GeometryPoseToEigenAffine3d( simulator_fbk_buffer_[0].gripper_poses[ind] ),
                    srv_data.response.indices, gripper_names[ind] ) );

        ROS_INFO( "Gripper #%zu: %s", ind, PrettyPrint::PrettyPrint( gripper_data_[ind] ).c_str() );
    }
}

void Planner::getObjectInitialConfiguration( const std::string& topic )
{
    // Get the initial configuration of the object
    ros::ServiceClient object_initial_configuration_client =
        nh_.serviceClient< deform_simulator::GetObjectInitialConfiguration >( topic );

    object_initial_configuration_client.waitForExistence();

    deform_simulator::GetObjectInitialConfiguration srv_data;
    object_initial_configuration_client.call( srv_data );
    object_initial_configuration_ =
        VectorGeometryPointToEigenMatrix3Xd( srv_data.response.config );

    ROS_INFO( "Number of points on object: %zu", srv_data.response.config.size() );
}
