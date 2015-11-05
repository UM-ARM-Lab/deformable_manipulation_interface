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

#include "smmap/ConfidenceStamped.h"

using namespace smmap;
using namespace EigenHelpersConversions;

Planner::Planner( ros::NodeHandle& nh,
        CustomScene::TaskType task,
        const std::string& cmd_gripper_traj_topic,
        const std::string& simulator_fbk_topic,
        const std::string& get_gripper_names_topic,
        const std::string& get_gripper_attached_node_indices_topic,
        const std::string& get_object_initial_configuration_topic,
        const std::string& confidence_topic,
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

    // Publish a our confidence values
    confidence_pub_ = nh_.advertise< smmap::ConfidenceStamped >( confidence_topic, 1 );
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

    ros::Rate rate( loop_rate );
    rate.sleep();

    // Run the planner at whatever rate we've been given
    ROS_INFO( "Running our planner" );
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

        std::pair< ObjectTrajectory, AllGrippersTrajectory > fbk = readSimulatorFeedbackBuffer();
        updateModels( fbk.first, fbk.second );
    }

    ROS_INFO( "Terminating" );
    spin_thread.join();
}

////////////////////////////////////////////////////////////////////////////////
// Input data parsing and model management
////////////////////////////////////////////////////////////////////////////////

std::pair< ObjectTrajectory, AllGrippersTrajectory > Planner::readSimulatorFeedbackBuffer()
{
    boost::mutex::scoped_lock lock( input_mtx_ );

    std::pair< ObjectTrajectory, AllGrippersTrajectory > fbk;

    fbk.first = object_trajectory_;
    fbk.second = grippers_trajectory_;

    object_trajectory_.clear();
    object_trajectory_.push_back( fbk.first[0] );

    for ( size_t gripper_ind = 0; gripper_ind < grippers_trajectory_.size(); gripper_ind++ )
    {
        grippers_trajectory_[gripper_ind].clear();
        grippers_trajectory_[gripper_ind].push_back( fbk.second[gripper_ind][0] );
    }

    lock.unlock();

    return fbk;
}

void Planner::updateModels( const ObjectTrajectory& object_trajectory,
                            const AllGrippersTrajectory& grippers_trajectory )
{
    assert( object_trajectory.size() >= 2 );
    assert( grippers_trajectory.size() >= 1 );
    assert( object_trajectory.size() == grippers_trajectory[0].size() );

    model_set_->updateModels( grippers_trajectory, object_trajectory );
    const std::vector<double> model_confidence = model_set_->getModelConfidence();

    cv::Mat image( 1, model_confidence.size(), CV_8UC3 );

    const double min_conf = *std::min_element( model_confidence.begin(), model_confidence.end() );
    const double max_conf = *std::max_element( model_confidence.begin(), model_confidence.end() );

    for ( size_t ind = 0; ind < model_confidence.size(); ind++ )
    {
        image.at< cv::Vec3b >( 0, ind )[0] = (model_confidence[ind] - min_conf) / ( max_conf - min_conf) * 255.0;
        image.at< cv::Vec3b >( 0, ind )[1] = 0;
        image.at< cv::Vec3b >( 0, ind )[2] = 0;
    }

    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, image).toImageMsg();
    img_msg->header.stamp = ros::Time::now();
    confidence_image_pub_.publish( img_msg );

    smmap::ConfidenceStamped double_msg;
    double_msg.confidence = model_confidence;
    double_msg.header.stamp = img_msg->header.stamp;
    confidence_pub_.publish( double_msg );
}

////////////////////////////////////////////////////////////////////////////////
// ROS Callbacks
////////////////////////////////////////////////////////////////////////////////

void Planner::simulatorFbkCallback(
        const deform_simulator::SimulatorFbkStamped& fbk )
{
    boost::mutex::scoped_lock lock( input_mtx_ );

    // If we've already received at least one data element, it's business as usual
    if ( unlikely( !fbk_buffer_initialized_.load() ) )
    {
        fbk_buffer_initialized_.store( true );
        object_trajectory_.clear();
        grippers_trajectory_.clear();
        grippers_trajectory_.resize( fbk.gripper_names.size() );
    }

    // TODO: if this data arrived out of order, do something smart
    object_trajectory_.push_back( VectorGeometryPointToEigenMatrix3Xd( fbk.object_configuration ) );

    for ( size_t gripper_ind = 0; gripper_ind < fbk.gripper_names.size(); gripper_ind++ )
    {
        grippers_trajectory_[gripper_ind].push_back(
                GeometryPoseToEigenAffine3d( fbk.gripper_poses[gripper_ind] ) );
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

    deform_simulator::GetGripperNames names_srv_data;
    gripper_names_client.call( names_srv_data );
    std::vector< std::string > gripper_names = names_srv_data.response.names;

    // Get the attached nodes for each gripper
    ros::ServiceClient gripper_node_indices_client =
        nh_.serviceClient< deform_simulator::GetGripperAttachedNodeIndices >( indices_topic );
    gripper_node_indices_client.waitForExistence();

    for ( size_t gripper_ind = 0; gripper_ind < gripper_names.size(); gripper_ind++ )
    {
        deform_simulator::GetGripperAttachedNodeIndices srv_data;
        srv_data.request.name = gripper_names[gripper_ind];
        gripper_node_indices_client.call( srv_data );

        // Still single threaded, so I don't need to worry about the locking here
        while ( !fbk_buffer_initialized_.load() )
        {
            usleep( 1000 );
            ros::spinOnce();
        }

        gripper_data_.push_back( GripperData( grippers_trajectory_[gripper_ind][0],
                    srv_data.response.indices, gripper_names[gripper_ind] ) );

        ROS_INFO( "Gripper #%zu: %s", gripper_ind, PrettyPrint::PrettyPrint( gripper_data_[gripper_ind] ).c_str() );
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
