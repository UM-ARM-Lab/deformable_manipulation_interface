#ifndef smmap_planner_h
#define smmap_planner_h

#include <boost/thread/recursive_mutex.hpp>

#include <ros/ros.h>
#include <arc_utilities/maybe.hpp>
#include <custom_scene/custom_scene.h>

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "smmap/model_set.h"

namespace smmap
{
    class Planner
    {
        public:
            Planner( ros::NodeHandle& nh,
                     CustomScene::TaskType task = CustomScene::TaskType::COVERAGE,
                     const std::string& cmd_gripper_traj_topic = "cmd_gripper_traj",
                     const std::string& simulator_fbk_topic = "simulator_fbk",
                     const std::string& get_gripper_names_topic = "get_gripper_names",
                     const std::string& get_cover_points_topic = "get_cover_points",
                     const std::string& get_gripper_attached_node_indices_topic = "get_gripper_attached_node_indices",
                     const std::string& get_gripper_pose_topic = "get_gripper_pose",
                     const std::string& get_object_initial_configuratoin_topic = "get_object_initial_configuration",
                     const std::string& confidence_image = "confidence",
                     const std::string& confidence_image_topic = "confidence_image",
                     const std::string& set_visualization_marker_topic = "set_visualization_marker");

            ////////////////////////////////////////////////////////////////////
            // Main function that makes things happen
            ////////////////////////////////////////////////////////////////////

            void run( const size_t num_traj_cmds_per_loop = 10 );

        private:
            // TODO: Use this
            CustomScene::TaskType task_;
            std::unique_ptr< ModelSet > model_set_;
            // Stores a "gripper name", {gripper_node_indices} pair for each gripper
            VectorGrippersData gripper_data_;
            ObjectPointSet object_initial_configuration_;
            ObjectPointSet cover_points_;

            ////////////////////////////////////////////////////////////////////
            // Internal helpers for the run() function
            ////////////////////////////////////////////////////////////////////

            AllGrippersTrajectory replan( size_t num_traj_cmds_per_loop );

            std::pair<ObjectTrajectory, AllGrippersTrajectory> readSimulatorFeedbackBuffer();
            void updateModels( const ObjectTrajectory& object_trajectory,
                               const AllGrippersTrajectory& grippers_trajectory );


            ////////////////////////////////////////////////////////////////////
            // Task specific functionality
            ////////////////////////////////////////////////////////////////////

            ObjectPointSet findObjectDesiredConfiguration( const ObjectPointSet& current );
            void visualizeRopeObject( const std::string& marker_name,
                                      const ObjectPointSet& rope,
                                      const std_msgs::ColorRGBA& color );

            ////////////////////////////////////////////////////////////////////
            // ROS Callbacks
            ////////////////////////////////////////////////////////////////////

            // TODO: when moving to a real robot, create a node that deals with
            // synchronization problems, and rename this function
            void simulatorFbkCallback( const deform_simulator::SimulatorFbkStamped& fbk );

            ////////////////////////////////////////////////////////////////////
            // ROS Objects and Helpers
            ////////////////////////////////////////////////////////////////////

            // Our internal version of ros::spin()
            static void spin( double loop_rate );
            void getGrippersData(const std::string& names_topic, const std::string& indices_topic , const std::string& pose_topic );
            void getObjectInitialConfiguration( const std::string& topic );
            void getCoverPoints( const std::string& topic );

            ros::NodeHandle nh_;

            ros::Publisher confidence_pub_;
            image_transport::ImageTransport it_;
            image_transport::Publisher confidence_image_pub_;
            ros::ServiceClient cmd_gripper_traj_client_;
            ros::ServiceClient visualization_client_;

            // global input mutex
            boost::recursive_mutex input_mtx_;

            ros::Subscriber simulator_fbk_sub_;
            ObjectTrajectory object_trajectory_;
            AllGrippersTrajectory grippers_trajectory_;
    };
}

#endif // planner_h
