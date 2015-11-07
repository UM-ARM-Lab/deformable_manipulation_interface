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
        const std::string& get_cover_points_topic,
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
    getCoverPoints( get_cover_points_topic );

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
        auto& pose = msg.trajectories[ind].pose;
        pose.resize(1);
        pose[0].position.x = -0.6;
        pose[0].position.y = 0.0;
        pose[0].position.z = 0.75;
        pose[0].orientation.x = 0;
        pose[0].orientation.y = 0;
        pose[0].orientation.z = 0;
        pose[0].orientation.w = 1;

        pose.resize( 100/loop_rate, pose[0] );

        for ( size_t i = 1; i < pose.size(); i++ )
        {
            pose[i].position.y = pose[i-1].position.y + 0.001;
        }
    }

    // Send our first stupid trajectory
    msg.header.stamp = ros::Time::now();
    cmd_gripper_traj_pub_.publish( msg );
    ros::Rate rate( loop_rate );
    rate.sleep();

    // Run the planner at whatever rate we've been given
    ROS_INFO( "Running our planner" );
    while ( ros::ok() )
    {
        ROS_INFO( "Updating models" );
        // Update the models with whatever feedback we have
        std::pair< ObjectTrajectory, AllGrippersTrajectory > fbk = readSimulatorFeedbackBuffer();
        updateModels( fbk.first, fbk.second );

        // here we make a better trajectory
        ROS_INFO( "Finding 'best' configuration" );
        ObjectPointSet object_desired_config = findObjectDesiredConfiguration( fbk.first.back() );

        std::vector< std::pair< AllGrippersTrajectory, double > > suggested_trajectories =
                model_set_->getDesiredGrippersTrajectories( fbk.first.back(), object_desired_config, getLastGrippersPose( fbk.second ), 0.001, 10 );

        int min_weighted_cost_ind = -1;
        double min_weighted_cost = std::numeric_limits< double >::infinity();

        ROS_INFO( "Finding 'best' trajectory" );
        for ( size_t traj_ind = 0; traj_ind < suggested_trajectories.size(); traj_ind++ )
        {
            double weighted_cost = suggested_trajectories[traj_ind].second; //*
                    //( suggested_trajectories[traj_ind].first.back() - object_desired_config ).norm();
            if ( weighted_cost < min_weighted_cost )
            {
                min_weighted_cost = weighted_cost;
                min_weighted_cost_ind = traj_ind;
            }
        }

        // copy the best trajectory over
        ROS_INFO( "Sending 'best' trajectory" );
        const AllGrippersTrajectory best_grippers_traj = suggested_trajectories[min_weighted_cost_ind].first;
        msg.trajectories.resize( best_grippers_traj.size() );
        for ( size_t gripper_ind = 0; gripper_ind < msg.trajectories.size(); gripper_ind++ )
        {
            msg.trajectories[gripper_ind].pose = VectorAffine3dToVectorGeometryPose( best_grippers_traj[gripper_ind] );
        }

//        auto& pose = msg.trajectories[0].pose;
//        pose[0] = pose[ pose.size() - 1 ];
//        for ( size_t i = 1; i < pose.size(); i++ )
//        {
//            pose[i].position.y = pose[i-1].position.y + 0.001;
//        }

        msg.header.stamp = ros::Time::now();
        cmd_gripper_traj_pub_.publish( msg );
        rate.sleep();
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
    assert( grippers_trajectory.size() >= 1 );
    assert( object_trajectory.size() == grippers_trajectory[0].size() );

    if ( object_trajectory.size() >= 2 )
    {

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
}

////////////////////////////////////////////////////////////////////////////////
// Task specific functionality
////////////////////////////////////////////////////////////////////////////////

ObjectPointSet Planner::findObjectDesiredConfiguration( const ObjectPointSet& current_configuration )
{
    // point should be the same size
    assert( current_configuration.rows() == cover_points_.rows() );

    ObjectPointSet desired_configuration = current_configuration;
    switch ( task_ )
    {
        case CustomScene::TaskType::COVERAGE:
        {
            // If we have more cover points than object points, align the object
            // to the cover points - multiple cover points per object point
            if ( cover_points_.cols() >= current_configuration.cols() )
            {
                // We'll need to track how many cover points are mapping to a given object point
                // in order to do the averaging.
                std::vector< int > num_mapped( current_configuration.cols(), 0 );

                // for every cover point, find the nearest deformable object point
                for ( int cover_ind = 0; cover_ind < cover_points_.cols(); cover_ind++ )
                {
                    Eigen::Vector3d cover_point = cover_points_.block< 3, 1 >( 0, cover_ind );

                    ObjectPointSet diff = current_configuration - ( cover_point * Eigen::MatrixXd::Ones( 1, current_configuration.cols() ) );

                    Eigen::RowVectorXd dist_sq = diff.array().square().colwise().sum();

                    // find the closest deformable point
                    int min_ind = -1;
                    double min_dist = std::numeric_limits< double >::infinity();
                    for ( int object_ind = 0; object_ind < dist_sq.cols(); object_ind++ )
                    {
                        if ( dist_sq( object_ind ) < min_dist )
                        {
                            min_ind = object_ind;
                            min_dist = dist_sq( object_ind );
                        }
                    }

                    // If this is the first time we've found this as the closest, just use it
                    if ( num_mapped[min_ind] == 0 )
                    {
                        desired_configuration.block< 3, 1 >( 0, min_ind ) = cover_points_.block< 3, 1 >( 0, cover_ind );
                    }
                    // Otherwise average it
                    else
                    {
                        desired_configuration.block< 3, 1 >( 0, min_ind ) = (
                                (double)num_mapped[min_ind] * desired_configuration.block< 3, 1 >( 0, min_ind )
                                + cover_points_.block< 3, 1 >( 0, cover_ind ) ) / (double)( num_mapped[min_ind] + 1 );
                    }
                    num_mapped[min_ind]++;
                }
            }
            // Otherwise align the cover points to the object
            // - multiple object points per cover point?
            else
            {
                throw new std::invalid_argument( "Not implemented" );
            }

            break;
        }

        default:
            throw new std::invalid_argument( "Unknown task type" );
    }

    return desired_configuration;
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
//    std::cout << "Post convert:\n" << PrettyPrint::PrettyPrint( grippers_trajectory_[0][ grippers_trajectory_[0].size() -1 ] ) << std::endl ;
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
        nh_.serviceClient< deform_simulator::GetPointSet >( topic );

    object_initial_configuration_client.waitForExistence();

    deform_simulator::GetPointSet srv_data;
    object_initial_configuration_client.call( srv_data );
    object_initial_configuration_ =
        VectorGeometryPointToEigenMatrix3Xd( srv_data.response.points );

    ROS_INFO( "Number of points on object: %zu", srv_data.response.points.size() );
}

void Planner::getCoverPoints( const std::string& topic )
{
    // Get the initial configuration of the object
    ros::ServiceClient cover_points_client =
        nh_.serviceClient< deform_simulator::GetPointSet >( topic );

    cover_points_client.waitForExistence();

    deform_simulator::GetPointSet srv_data;
    cover_points_client.call( srv_data );
    cover_points_ =
        VectorGeometryPointToEigenMatrix3Xd( srv_data.response.points );

    ROS_INFO( "Number of cover points: %zu", srv_data.response.points.size() );
}
