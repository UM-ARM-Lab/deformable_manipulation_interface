#ifndef smmap_planner_h
#define smmap_planner_h

#include <ros/ros.h>
#include <custom_scene/custom_scene.h>

#include "smmap/model_set.h"

namespace smmap
{
    class Planner
    {
        public:
            Planner( ros::NodeHandle& nh,
                    CustomScene::TaskType task = CustomScene::TaskType::COVERAGE,
                    std::string cmd_gripper_traj_topic = "cmd_gripper_traj",
                    std::string gripper_pose_topic = "gripper_pose",
                    std::string object_configuration_topic = "object_configuration",
                    std::string get_gripper_names_topic = "get_gripper_names" );

            ////////////////////////////////////////////////////////////////////
            // Main function that makes things happen
            ////////////////////////////////////////////////////////////////////

            void run();

        private:
            CustomScene::TaskType task_;
            ModelSet models;
            std::vector< std::string > gripper_names_;

            ////////////////////////////////////////////////////////////////////
            // ROS Callbacks
            ////////////////////////////////////////////////////////////////////

            void gripperPoseCallback( const deform_simulator::GripperPoseStamped& gripper_pose );
            void objectConfigurationCallback( const deform_simulator::ObjectConfigurationStamped& object_configuration );

            ////////////////////////////////////////////////////////////////////
            // ROS Objects and Helpers
            ////////////////////////////////////////////////////////////////////

            // Our internal version of ros::spin()
            static void spin( double loop_rate );

            ros::NodeHandle nh_;

            ros::Publisher cmd_gripper_traj_pub_;

            ros::Subscriber gripper_pose_sub_;
            ros::Subscriber object_configuration_sub_;
    };
}

#endif
