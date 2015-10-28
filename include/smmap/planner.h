#ifndef smmap_planner_h
#define smmap_planner_h

#include <ros/ros.h>
#include <arc_utilities/maybe.hpp>
#include <custom_scene/custom_scene.h>

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
                    const std::string& get_object_initial_configuratoin_topic = "get_object_initial_configuration" );

            ////////////////////////////////////////////////////////////////////
            // Main function that makes things happen
            ////////////////////////////////////////////////////////////////////

            void run( double loop_rate = 10 );

        private:
            CustomScene::TaskType task_;
            std::unique_ptr< ModelSet > model_set_;
            std::vector< std::string > gripper_names_;
            ObjectPointSet object_initial_configuration_;

            ////////////////////////////////////////////////////////////////////
            // Input data parsing and model management
            ////////////////////////////////////////////////////////////////////

            void updateModels( boost::mutex::scoped_lock& lock );

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
            void getGripperNames( const std::string& topic );
            void getObjectInitialConfiguration( const std::string& topic );

            ros::NodeHandle nh_;

            ros::Publisher cmd_gripper_traj_pub_;

            // global input mutex
            boost::mutex input_mtx_;

            ros::Subscriber simulator_fbk_sub_;
            std::vector< deform_simulator::SimulatorFbkStamped > simulator_fbk_buffer_;
            bool fbk_buffer_initialized_;
    };
}

#endif
