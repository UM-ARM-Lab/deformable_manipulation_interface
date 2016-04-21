#ifndef ROS_PARAMS_HPP
#define ROS_PARAMS_HPP

#include <exception>
#include <string>
#include <arc_utilities/ros_helpers.hpp>

#include "smmap_experiment_params/task_enums.h"

namespace smmap
{
    inline float GetClothXSize( ros::NodeHandle &nh );

    ////////////////////////////////////////////////////////////////////////////
    // Task and Deformable Type parameters
    ////////////////////////////////////////////////////////////////////////////

    inline DeformableType GetDeformableType( ros::NodeHandle& nh )
    {
        std::string deformable_type = ROSHelpers::GetParam< std::string >( nh, "deformable_type", "rope" );

        if ( deformable_type.compare( "rope" ) == 0 )
        {
            return DeformableType::ROPE;
        }
        else if ( deformable_type.compare( "cloth" ) == 0 )
        {
            return DeformableType::CLOTH;
        }
        else
        {
            throw new std::invalid_argument( "Unknown deformable type: " + deformable_type );
        }
    }

    inline TaskType GetTaskType( ros::NodeHandle& nh )
    {
        std::string task_type = ROSHelpers::GetParam< std::string >(nh, "task_type", "coverage" );

        if ( task_type.compare( "cylinder_coverage" ) == 0 )
        {
            return TaskType::CYLINDER_COVERAGE;
        }
        else if ( task_type.compare( "table_coverage" ) == 0 )
        {
            return TaskType::TABLE_COVERAGE;
        }
        else if ( task_type.compare( "colab_folding" ) == 0 )
        {
            return TaskType::COLAB_FOLDING;
        }
        else
        {
            throw new std::invalid_argument( "Unknown task type: " + task_type );
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    // Table Size Settings
    ////////////////////////////////////////////////////////////////////////////

    inline float GetTableSurfaceX( ros::NodeHandle& nh )    // METERS
    {
        return ROSHelpers::GetParam( nh, "table_surface_x", 0.0f );
    }

    inline float GetTableSurfaceY( ros::NodeHandle& nh )    // METERS
    {
        return ROSHelpers::GetParam( nh, "table_surface_y", 0.0f );
    }

    inline float GetTableSurfaceZ( ros::NodeHandle& nh )    // METERS
    {
        return ROSHelpers::GetParam( nh, "table_surface_z", 0.7f );
    }

    inline float GetTableSizeX( ros::NodeHandle& nh )       // METERS
    {
        switch ( GetDeformableType( nh ) )
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam( nh, "table_x_size", 3.0f );

            case DeformableType::CLOTH:
                return ROSHelpers::GetParam( nh, "table_x_size", 0.4f );

            default:
                throw new std::invalid_argument( "Unknown table size for deformable type " + GetDeformableType( nh ) );
        }
    }

    inline float GetTableSizeY( ros::NodeHandle& nh )       // METERS
    {
        switch ( GetDeformableType( nh ) )
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam( nh, "table_y_size", 3.0f );

            case DeformableType::CLOTH:
                return ROSHelpers::GetParam( nh, "table_y_size", 0.4f );

            default:
                throw new std::invalid_argument( "Unknown table size for deformable type " + GetDeformableType( nh ) );
        }
    }

    inline float GetTableSizeZ( ros::NodeHandle& nh )       // METERS
    {
        return ROSHelpers::GetParam( nh, "table_z_size", GetTableSurfaceZ( nh ) );
    }

    inline float GetTableLegWidth( ros::NodeHandle& nh )    // METERS
    {
        return ROSHelpers::GetParam( nh, "table_leg_width", 0.05f );
    }

    inline float GetTableThickness( ros::NodeHandle& nh )   // METERS
    {
        return ROSHelpers::GetParam( nh, "table_thickness", 0.05f );
    }

    ////////////////////////////////////////////////////////////////////////////
    // Cylinder Size Settings
    ////////////////////////////////////////////////////////////////////////////

    inline float GetCylinderRadius( ros::NodeHandle& nh )   // METERS
    {
        switch ( GetDeformableType( nh ) )
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam( nh, "rope_cylinder_radius", 0.15f );

            case DeformableType::CLOTH:
                return ROSHelpers::GetParam( nh, "cloth_cylinder_radius", 0.10f );

            default:
                throw new std::invalid_argument( "Unknown cylinder radius for deformable type " + GetDeformableType( nh ) );
        }
    }

    inline float GetCylinderHeight( ros::NodeHandle& nh )   // METERS
    {
        switch ( GetDeformableType( nh ) )
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam( nh, "rope_cylinder_height", 0.3f );

            case DeformableType::CLOTH:
                return ROSHelpers::GetParam( nh, "cloth_cylinder_height", 0.3f );

            default:
                throw new std::invalid_argument( "Unknown cylinder height for deformable type " + GetDeformableType( nh ) );
        }
    }

    inline float GetCylinderCenterOfMassX( ros::NodeHandle& nh )    // METERS
    {
        switch ( GetDeformableType( nh ) )
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam( nh, "rope_cylinder_com_x", GetTableSurfaceX( nh ) );

            case DeformableType::CLOTH:
                return ROSHelpers::GetParam( nh, "cloth_cylinder_com_x", GetTableSurfaceX( nh ) - GetClothXSize( nh ) );

            default:
                throw new std::invalid_argument( "Unknown cylinder com for deformable type " + GetDeformableType( nh ) );
        }
    }

    inline float GetCylinderCenterOfMassY( ros::NodeHandle& nh )    // METERS
    {
        switch ( GetDeformableType( nh ) )
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam( nh, "rope_cylinder_com_y", GetTableSurfaceY( nh ) + GetCylinderRadius( nh ) * 5.0f / 3.0f );

            case DeformableType::CLOTH:
                return ROSHelpers::GetParam( nh, "cloth_cylinder_com_y", GetTableSurfaceY( nh ) );

            default:
                throw new std::invalid_argument( "Unknown cylinder com for deformable type " + GetDeformableType( nh ) );
        }
    }

    // NOTE: this currently has part of the cylinder inside the table
    inline float GetCylinderCenterOfMassZ( ros::NodeHandle& nh )    // METERS
    {
        switch ( GetDeformableType( nh ) )
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam( nh, "rope_cylinder_com_z",
                                             GetTableSurfaceZ( nh )
                                             - GetTableThickness( nh ) / 2.0f
                                             + GetCylinderHeight( nh ) / 2.0f );

            case DeformableType::CLOTH:
                return ROSHelpers::GetParam( nh, "cloth_cylinder_com_z", GetTableSurfaceZ( nh ) );

            default:
                throw new std::invalid_argument( "Unknown cylinder com for deformable type " + GetDeformableType( nh ) );
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    // Rope Settings
    ////////////////////////////////////////////////////////////////////////////

    inline float GetRopeSegmentLength( ros::NodeHandle& nh )    // METERS
    {
        return ROSHelpers::GetParam( nh, "rope_segment_length", 0.025f );
    }

    inline float GetRopeRadius( ros::NodeHandle& nh )           // METERS
    {
        return ROSHelpers::GetParam( nh, "rope_radius", 0.01f );
    }

    inline int GetRopeNumLinks( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam( nh, "rope_num_links", 50 );
    }

    ////////////////////////////////////////////////////////////////////////////
    // Rope BulletPhysics settings
    ////////////////////////////////////////////////////////////////////////////

    // TODO: merge this and the cloth version into a single param?
    inline float GetRopeGripperApperture( ros::NodeHandle& nh ) // METERS
    {
        // TODO: why did Dmitry's code use 0.5f here?
        return ROSHelpers::GetParam( nh, "rope_gripper_apperture", 0.03f );
    }

    ////////////////////////////////////////////////////////////////////////////
    // Rope-Cylinder experiment settings
    ////////////////////////////////////////////////////////////////////////////

    inline float GetRopeCenterOfMassX( ros::NodeHandle& nh )    // METERS
    {
        return ROSHelpers::GetParam( nh, "rope_com_x", GetTableSurfaceX( nh ) + 15.0f * GetRopeSegmentLength( nh ) );
    }

    inline float GetRopeCenterOfMassY( ros::NodeHandle& nh )    // METERS
    {
        return ROSHelpers::GetParam( nh, "rope_com_y", GetTableSurfaceY( nh ) );
    }

    inline float GetRopeCenterOfMassZ( ros::NodeHandle& nh )    // METERS
    {
        return ROSHelpers::GetParam( nh, "rope_com_z", GetTableSurfaceZ( nh ) + 5.0f * GetRopeRadius( nh ) );
    }

    ////////////////////////////////////////////////////////////////////
    // Cloth settings
    ////////////////////////////////////////////////////////////////////

    inline float GetClothXSize( ros::NodeHandle& nh )           // METERS
    {
        return ROSHelpers::GetParam( nh, "cloth_x_size", 0.5f );
    }

    inline float GetClothYSize( ros::NodeHandle& nh )           // METERS
    {
        return ROSHelpers::GetParam( nh, "cloth_y_size", 0.5f );
    }

    inline float GetClothCenterOfMassX( ros::NodeHandle& nh )   // METERS
    {
        return ROSHelpers::GetParam( nh, "cloth_com_x", GetTableSurfaceX( nh ) + GetClothXSize( nh ) / 2.0f );
    }

    inline float GetClothCenterOfMassY( ros::NodeHandle& nh )   // METERS
    {
        return ROSHelpers::GetParam( nh, "cloth_com_y", GetTableSurfaceY( nh ) );
    }

    inline float GetClothCenterOfMassZ( ros::NodeHandle& nh )   // METERS
    {
        return ROSHelpers::GetParam( nh, "cloth_com_z", GetTableSurfaceZ( nh ) + 0.01f );
    }

    ////////////////////////////////////////////////////////////////////////////
    // Cloth BulletPhysics settings
    ////////////////////////////////////////////////////////////////////////////

    inline int GetClothNumDivs( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam( nh, "cloth_num_divs", 45 );
    }

    inline float GetClothGripperApperture( ros::NodeHandle& nh ) // METERS
    {
        return ROSHelpers::GetParam( nh, "cloth_gripper_apperture", 0.1f );
    }

    ////////////////////////////////////////////////////////////////////////////
    // Simulator settings
    ////////////////////////////////////////////////////////////////////////////

    inline double GetFeedbackCovariance( ros::NodeHandle& nh )  // METERS^2
    {
        return ROSHelpers::GetParam( nh, "feedback_covariance", 0.0 );
    }

    ////////////////////////////////////////////////////////////////////////////
    // (Fake) Robot settings
    ////////////////////////////////////////////////////////////////////////////

    inline double GetRobotControlPeriod( ros::NodeHandle& nh ) // SECONDS
    {
        return ROSHelpers::GetParam( nh, "robot_control_rate", 0.01 );
    }

    ////////////////////////////////////////////////////////////////////////////
    // Planner settings
    ////////////////////////////////////////////////////////////////////////////

    inline bool GetUseMultiModel( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam( nh, "use_multi_model", false );
    }

    inline int GetPlanningHorizon( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam( nh, "planning_horizion", 1 );
    }

    ////////////////////////////////////////////////////////////////////////////
    // Logging functionality
    ////////////////////////////////////////////////////////////////////////////

    inline bool GetLoggingEnabled( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam( nh, "logging_enabled", false );
    }

    inline std::string GetLogFolder( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam< std::string >( nh, "log_folder", "/tmp/" );
    }

    ////////////////////////////////////////////////////////////////////////////
    // ROS Topic settings
    ////////////////////////////////////////////////////////////////////////////

    inline std::string GetCommandGripperTrajTopic( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam< std::string >( nh, "command_gripper_traj_topic", "command_gripper_traj" );
    }

    inline std::string GetSimulatorFeedbackTopic( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam< std::string >( nh, "simulator_feedback_topic", "simulator_feedback" );
    }

    inline std::string GetCoverPointsTopic( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam< std::string >( nh, "get_cover_points_topic", "get_cover_points" );
    }

    inline std::string GetMirrorLineTopic( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam< std::string >( nh, "get_mirror_line_topic", "get_mirror_line" );
    }

    inline std::string GetGripperNamesTopic( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam< std::string >( nh, "get_gripper_names_topic", "get_gripper_names" );
    }

    inline std::string GetGripperAttachedNodeIndicesTopic( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam< std::string >( nh, "get_gripper_attached_node_indices", "get_gripper_attached_node_indices" );
    }

    inline std::string GetGripperPoseTopic( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam< std::string >( nh, "get_gripper_pose_topic", "get_gripper_pose" );
    }

    inline std::string GetObjectInitialConfigurationTopic( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam< std::string >( nh, "get_object_initial_configuration_topic", "get_object_initial_configuration" );
    }

    inline std::string GetObjectCurrentConfigurationTopic( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam< std::string >( nh, "get_object_current_configuration_topic", "get_object_current_configuration" );
    }

    inline std::string GetVisualizationMarkerTopic( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam< std::string >( nh, "visualization_marker_topic", "visualization_marker" );
    }

    inline std::string GetVisualizationMarkerArrayTopic( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam< std::string >( nh, "visualization_marker_array_topic", "visualization_marker_array" );
    }

    inline std::string GetConfidenceTopic( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam< std::string >( nh, "confidence_topic", "confidence" );
    }

    inline std::string GetConfidenceImageTopic( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam< std::string >( nh, "confidence_image_topic", "confidence_image" );
    }

    inline std::string GetGripperCollisionCheckTopic( ros::NodeHandle& nh )
    {
        return ROSHelpers::GetParam< std::string >( nh, "get_gripper_collision_check_topic", "get_gripper_collision_check" );
    }

    ////////////////////////////////////////////////////////////////////////////
    // ROS TF Frame name settings
    ////////////////////////////////////////////////////////////////////////////

    inline std::string GetWorldFrameName()
    {
        return "/mocap_world";
    }

    inline std::string GetTableFrameName()
    {
        return "/table_surface";
    }

    inline std::string GetKinectBaseFrameName()
    {
        return "/mocap_Kinect2Block_Kinect2Block";
    }

    inline std::string GetKinectCameraFrameName()
    {
        return "/Kinect2_ir_camera";
    }
}

#endif // ROS_PARAMS_HPP
