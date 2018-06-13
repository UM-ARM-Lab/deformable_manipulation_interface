#ifndef ROS_PARAMS_HPP
#define ROS_PARAMS_HPP

#include <cmath>
#include <string>
#include <chrono>
#include <unordered_map>
#include <vector>
#include <Eigen/Core>
#include <arc_utilities/ros_helpers.hpp>
#include <arc_utilities/arc_exceptions.hpp>
#include <ros/package.h>

#include "deformable_manipulation_experiment_params/task_enums.h"

namespace smmap
{
    inline float GetClothXSize(ros::NodeHandle &nh);

    ////////////////////////////////////////////////////////////////////////////
    // Visualization Settings
    ////////////////////////////////////////////////////////////////////////////

    inline bool GetDisableAllVisualizations(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<bool>(nh, "disable_all_visualizations", __func__);
        return val.GetImmutable();
    }

    inline bool GetVisualizeObjectDesiredMotion(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "visualize_object_desired_motion", true);
    }

    inline bool GetVisualizerGripperMotion(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "visualize_gripper_motion", true);
    }

    inline bool GetVisualizeObjectPredictedMotion(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "visualize_object_predicted_motion", true);
    }

    inline bool GetVisualizePRM(ros::NodeHandle& nh, const bool default_vis = true)
    {
        return ROSHelpers::GetParam(nh, "visualize_prm", default_vis);
    }

    inline bool GetVisualizeFreeSpaceGraph(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "visualize_free_space_graph", true);
    }

    inline bool GetVisualizeCorrespondences(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "visualize_correspondences", true);
    }

    inline int GetViewerWidth(ros::NodeHandle& nh)      // Pixels
    {
        const auto val = ROSHelpers::GetParamDebugLog<int>(nh, "viewer_width", 800);
        assert(val > 0);
        return val;
    }

    inline int GetViewerHeight(ros::NodeHandle& nh)     // Pixels
    {
        const auto val = ROSHelpers::GetParamDebugLog<int>(nh, "viewer_height", 800);
        assert(val > 0);
        return val;
    }

    inline bool VisualizeStrainLines(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "visualize_strain_lines", false);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Task and Deformable Type parameters
    ////////////////////////////////////////////////////////////////////////////

    inline DeformableType GetDeformableType(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequiredDebugLog<std::string>(nh, "deformable_type", __func__);
        const std::string deformable_type = val.GetImmutable();

        if (deformable_type.compare("rope") == 0)
        {
            return DeformableType::ROPE;
        }
        else if (deformable_type.compare("cloth") == 0)
        {
            return DeformableType::CLOTH;
        }
        else
        {
            ROS_FATAL_STREAM_NAMED("params", "Unknown deformable type: " << deformable_type);
            throw_arc_exception(std::invalid_argument, "Unknown deformable type: " + deformable_type);
        }
    }

    /**
     *  Maps the ros param "task_type" into an enum TaskType
     */
    inline TaskType GetTaskType(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequiredDebugLog<std::string>(nh, "task_type", __func__);
        const std::string task_type = val.GetImmutable();

        std::unordered_map<std::string, TaskType> task_map{
            {"rope_cylinder_coverage",              TaskType::ROPE_CYLINDER_COVERAGE},
            {"rope_cylinder_coverage_two_grippers", TaskType::ROPE_CYLINDER_COVERAGE_TWO_GRIPPERS},
            {"cloth_cylinder_coverage",             TaskType::CLOTH_CYLINDER_COVERAGE},
            {"cloth_table_coverage",                TaskType::CLOTH_TABLE_COVERAGE},
            {"cloth_colab_folding",                 TaskType::CLOTH_COLAB_FOLDING},
            {"cloth_wafr",                          TaskType::CLOTH_WAFR},
            {"cloth_single_pole",                   TaskType::CLOTH_SINGLE_POLE},
            {"cloth_wall",                          TaskType::CLOTH_WALL},
            {"cloth_double_slit",                   TaskType::CLOTH_DOUBLE_SLIT},
            {"rope_maze",                           TaskType::ROPE_MAZE},
            {"rope_zig_match",                      TaskType::ROPE_ZIG_MATCH},
            {"rope_table_linear_motion",            TaskType::ROPE_TABLE_LINEAR_MOTION},
            {"cloth_table_linear_motion",           TaskType::CLOTH_TABLE_LINEAR_MOTION},
            {"rope_table_penetration",              TaskType::ROPE_TABLE_PENTRATION},
            {"cloth_placemat_live_robot",           TaskType::CLOTH_PLACEMAT_LIVE_ROBOT}
        };
        
        try
        {
            return task_map.at(task_type);
        }
        catch (std::out_of_range& e)
        {
            ROS_FATAL_STREAM_NAMED("params", "Unknown task type: " << task_type);
            throw_arc_exception(std::invalid_argument, "Unknown task type: " + task_type);
        }
    }

    inline double GetMaxTime(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "task/max_time", __func__);
        return val.GetImmutable();
    }

    inline double GetMaxStretchFactor(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "task/max_stretch_factor", __func__);
        return val.GetImmutable();
    }

    inline double GetMaxBandLength(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "task/max_band_length", __func__);
        return val.GetImmutable();
    }

    inline float GetMaxStrain(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<float>(nh, "max_strain", __func__);
        return val.GetImmutable();
    }

    ////////////////////////////////////////////////////////////////////////////
    // Error calculation settings
    ////////////////////////////////////////////////////////////////////////////

    inline double GetErrorThresholdAlongNormal(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "task/error_threshold_along_normal", __func__);
        return val.GetImmutable();
    }

    inline double GetErrorThresholdDistanceToNormal(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "task/error_threshold_distance_to_normal", __func__);
        return val.GetImmutable();
    }

    inline double GetErrorThresholdTaskDone(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "task/error_threshold_task_done", __func__);
        return val.GetImmutable();
    }

    inline double GetDesiredMotionScalingFactor(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "task/desired_motion_scale_factor", 1.0);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Gripper Size Settings
    ////////////////////////////////////////////////////////////////////////////

    inline float GetGripperApperture(ros::NodeHandle& nh)   // METERS
    {
        switch(GetDeformableType(nh))
        {
            case DeformableType::ROPE:
                // TODO: why did Dmitry's code use 0.5f here?
                return ROSHelpers::GetParamDebugLog(nh, "rope_gripper_apperture", 0.03f);

            case DeformableType::CLOTH:
                // TODO: This number is actually the "closed gap"
                //       The original number was 0.1f
                return ROSHelpers::GetParamDebugLog(nh, "cloth_gripper_apperture", 0.006f);
        }
    }

    // TODO: where is this still used? Is it being used correctly vs ControllerMinDistToObstacles?
    inline double GetRobotGripperRadius()                   // METERS
    {
        return 0.023;
    }

    // Used by the "older" avoidance code, I.e. LeastSquaresControllerWithObjectAvoidance
    inline double GetRobotMinGripperDistanceToObstacles()   // METERS
    {
        return 0.005;
    }

    inline double GetControllerMinDistanceToObstacles(ros::NodeHandle& nh) // METERS
    {
        return ROSHelpers::GetParam(nh, "controller_min_distance_to_obstacles", 0.07);
    }

    inline double GetRRTMinGripperDistanceToObstacles(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "rrt/min_gripper_distance_to_obstacles", __func__);
        assert(val.GetImmutable() >= 0.0);
        return val.GetImmutable();
    }

    inline double GetRRTTargetMinDistanceScaleFactor(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "rrt/target_min_distance_scale_factor", __func__);
        assert(val.GetImmutable() >= 1.0);
        return val.GetImmutable();
    }

    ////////////////////////////////////////////////////////////////////////////
    // Table Size Settings
    ////////////////////////////////////////////////////////////////////////////

    inline float GetTableSurfaceX(ros::NodeHandle& nh)      // METERS
    {
        return ROSHelpers::GetParam(nh, "table_surface_x", 0.0f);
    }

    inline float GetTableSurfaceY(ros::NodeHandle& nh)      // METERS
    {
        return ROSHelpers::GetParam(nh, "table_surface_y", 0.0f);
    }

    inline float GetTableSurfaceZ(ros::NodeHandle& nh)      // METERS
    {
        return ROSHelpers::GetParam(nh, "table_surface_z", 0.7f);
    }

    inline float GetTableHalfExtentsX(ros::NodeHandle& nh)  // METERS
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "table_x_half_extents", __func__);
        return (float)val.GetImmutable();
    }

    inline float GetTableHalfExtentsY(ros::NodeHandle& nh)  // METERS
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "table_y_half_extents", __func__);
        return (float)val.GetImmutable();
    }

    inline float GetTableHeight(ros::NodeHandle& nh)        // METERS
    {
        return ROSHelpers::GetParam(nh, "table_height", GetTableSurfaceZ(nh));
    }

    inline float GetTableLegWidth(ros::NodeHandle& nh)      // METERS
    {
        return ROSHelpers::GetParam(nh, "table_leg_width", 0.05f);
    }

    inline float GetTableThickness(ros::NodeHandle& nh)     // METERS
    {
        return ROSHelpers::GetParam(nh, "table_thickness", 0.05f);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Cylinder Size Settings
    // TODO: Update launch files to contain these defaults
    ////////////////////////////////////////////////////////////////////////////

    inline float GetCylinderRadius(ros::NodeHandle& nh)           // METERS
    {
        switch (GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
            case TaskType::ROPE_CYLINDER_COVERAGE_TWO_GRIPPERS:
                return ROSHelpers::GetParam(nh, "rope_cylinder_radius", 0.15f);

            case TaskType::CLOTH_CYLINDER_COVERAGE:
            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "cloth_cylinder_radius", 0.10f);

            case TaskType::CLOTH_SINGLE_POLE:
            case TaskType::CLOTH_WALL:
                return ROSHelpers::GetParam(nh, "cloth_cylinder_radius", 0.04f);

            default:
                const auto val = ROSHelpers::GetParamRequired<double>(nh, "cylinder_radius", __func__);
                return (float)val.GetImmutable();
        }
    }

    inline float GetCylinderHeight(ros::NodeHandle& nh)           // METERS
    {
        switch (GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
            case TaskType::ROPE_CYLINDER_COVERAGE_TWO_GRIPPERS:
                return ROSHelpers::GetParam(nh, "rope_cylinder_height", 0.3f);

            case TaskType::CLOTH_CYLINDER_COVERAGE:
            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "cloth_cylinder_height", 0.3f);

            case TaskType::CLOTH_SINGLE_POLE:
            case TaskType::CLOTH_WALL:
                return ROSHelpers::GetParam(nh, "cloth_cylinder_height", 1.0f);

            default:
                const auto val = ROSHelpers::GetParamRequired<double>(nh, "cylinder_height", __func__);
                return (float)val.GetImmutable();
        }
    }

    inline float GetCylinderCenterOfMassX(ros::NodeHandle& nh)    // METERS
    {
        switch (GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
            case TaskType::ROPE_CYLINDER_COVERAGE_TWO_GRIPPERS:
                return ROSHelpers::GetParam(nh, "rope_cylinder_com_x", GetTableSurfaceX(nh));

            case TaskType::CLOTH_CYLINDER_COVERAGE:
            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "cloth_cylinder_com_x", GetTableSurfaceX(nh) - GetClothXSize(nh));

            case TaskType::CLOTH_SINGLE_POLE:
            case TaskType::CLOTH_WALL:
                return ROSHelpers::GetParam(nh, "cloth_cylinder_com_x", -0.3f);

            default:
                const auto val = ROSHelpers::GetParamRequired<double>(nh, "cylinder_com_x", __func__);
                return (float)val.GetImmutable();
        }
    }

    inline float GetCylinderCenterOfMassY(ros::NodeHandle& nh)    // METERS
    {
        switch (GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
            case TaskType::ROPE_CYLINDER_COVERAGE_TWO_GRIPPERS:
                return ROSHelpers::GetParam(nh, "rope_cylinder_com_y", GetTableSurfaceY(nh) + GetCylinderRadius(nh) * 5.0f / 3.0f);

            case TaskType::CLOTH_CYLINDER_COVERAGE:
            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "cloth_cylinder_com_y", GetTableSurfaceY(nh));

            case TaskType::CLOTH_SINGLE_POLE:
            case TaskType::CLOTH_WALL:
                return ROSHelpers::GetParam(nh, "cloth_cylinder_com_y", 0.0f);

            default:
                const auto val = ROSHelpers::GetParamRequired<double>(nh, "cylinder_com_y", __func__);
                return (float)val.GetImmutable();
        }
    }

    inline float GetCylinderCenterOfMassZ(ros::NodeHandle& nh)    // METERS
    {
        switch (GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
            case TaskType::ROPE_CYLINDER_COVERAGE_TWO_GRIPPERS:
                return ROSHelpers::GetParam(nh, "rope_cylinder_com_z", GetTableSurfaceZ(nh) + GetCylinderHeight(nh) / 2.0f);

            case TaskType::CLOTH_CYLINDER_COVERAGE:
            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "cloth_cylinder_com_z", GetTableSurfaceZ(nh));

            case TaskType::CLOTH_SINGLE_POLE:
            case TaskType::CLOTH_WALL:
                return ROSHelpers::GetParam(nh, "cloth_cylinder_com_z", 1.0f);

            default:
                const auto val = ROSHelpers::GetParamRequired<double>(nh, "cylinder_com_z", __func__);
                return (float)val.GetImmutable();
        }
    }

    // Cylinder Size settings for WAFR task case

    inline float GetWafrCylinderRadius(ros::NodeHandle& nh)           // METERS
    {
        return ROSHelpers::GetParam(nh, "wafr_second_cylinder_radius", 0.025f);
    }

    inline float GetWafrCylinderHeight(ros::NodeHandle& nh)           // METERS
    {
        return ROSHelpers::GetParam(nh, "wafr_second_cylinder_height", 0.51f);
    }

    inline float GetWafrCylinderRelativeCenterOfMassX(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "wafr_cylinder_relative_com_x", - 0.15f);
    }

    inline float GetWafrCylinderRelativeCenterOfMassY(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "wafr_cylinder_relative_com_y", 0.0f);
    }

    inline float GetWafrCylinderRelativeCenterOfMassZ(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "wafr_cylinder_relative_com_z", 0.2f);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Rope Maze Wall Size and Visibility Settings
    ////////////////////////////////////////////////////////////////////////////

    inline float GetWallHeight(ros::NodeHandle& nh)             // METERS
    {
        const auto val = ROSHelpers::GetParamRequired<float>(nh, "wall_height", __func__);
        return val.GetImmutable();
    }

    inline float GetWallCenterOfMassZ(ros::NodeHandle& nh)      // METERS
    {
        const auto val = ROSHelpers::GetParamRequired<float>(nh, "wall_com_z", __func__);
        return val.GetImmutable();
    }

    inline float GetOuterWallsAlpha(ros::NodeHandle& nh)        // 0.0 thru 1.0 (inclusive)
    {
        const auto val = ROSHelpers::GetParamRequired<float>(nh, "outer_walls_alpha", __func__);
        assert(val.GetImmutable() >= 0.0 && val.GetImmutable() <= 1.0);
        return val.GetImmutable();
    }

    inline float GetFloorDividerAlpha(ros::NodeHandle& nh)      // 0.0 thru 1.0 (inclusive)
    {
        const auto val = ROSHelpers::GetParamRequired<float>(nh, "floor_divider_alpha", __func__);
        assert(val.GetImmutable() >= 0.0 && val.GetImmutable() <= 1.0);
        return val.GetImmutable();
    }

    inline float GetFirstFloorAlpha(ros::NodeHandle& nh)        // 0.0 thru 1.0 (inclusive)
    {
        const auto val = ROSHelpers::GetParamRequired<float>(nh, "first_floor_alpha", __func__);
        assert(val.GetImmutable() >= 0.0 && val.GetImmutable() <= 1.0);
        return val.GetImmutable();
    }

    inline float GetSecondFloorAlpha(ros::NodeHandle& nh)       // 0.0 thru 1.0 (inclusive)
    {
        const auto val = ROSHelpers::GetParamRequired<float>(nh, "second_floor_alpha", __func__);
        assert(val.GetImmutable() >= 0.0 && val.GetImmutable() <= 1.0);
        return val.GetImmutable();
    }

    ////////////////////////////////////////////////////////////////////////////
    // Rope Settings
    ////////////////////////////////////////////////////////////////////////////

    inline float GetRopeSegmentLength(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "rope_segment_length", 0.025f);
    }

    inline float GetRopeRadius(ros::NodeHandle& nh)           // METERS
    {
        return ROSHelpers::GetParam(nh, "rope_radius", 0.01f);
    }

    inline int GetRopeNumLinks(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "rope_num_links", 49);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Rope starting position settings
    ////////////////////////////////////////////////////////////////////////////

    inline float GetRopeCenterOfMassX(ros::NodeHandle& nh)    // METERS
    {
        const auto val = ROSHelpers::GetParamRequired<float>(nh, "rope_com_x", __func__);
        return val.GetImmutable();
    }

    inline float GetRopeCenterOfMassY(ros::NodeHandle& nh)    // METERS
    {
        const auto val = ROSHelpers::GetParamRequired<float>(nh, "rope_com_y", __func__);
        return val.GetImmutable();
    }

    inline float GetRopeCenterOfMassZ(ros::NodeHandle& nh)    // METERS
    {
        const auto val = ROSHelpers::GetParamRequired<float>(nh, "rope_com_z", __func__);
        return val.GetImmutable();
    }

    ////////////////////////////////////////////////////////////////////
    // Cloth settings
    ////////////////////////////////////////////////////////////////////

    inline float GetClothXSize(ros::NodeHandle& nh)           // METERS
    {
        return ROSHelpers::GetParam(nh, "cloth_x_size", 0.5f);
    }

    inline float GetClothYSize(ros::NodeHandle& nh)           // METERS
    {
        return ROSHelpers::GetParam(nh, "cloth_y_size", 0.5f);
    }

    inline float GetClothCenterOfMassX(ros::NodeHandle& nh)   // METERS
    {
        switch(GetTaskType(nh))
        {
            case TaskType::CLOTH_COLAB_FOLDING:
            case TaskType::CLOTH_TABLE_COVERAGE:
                return ROSHelpers::GetParam(nh, "cloth_com_x", GetTableSurfaceX(nh) + GetClothXSize(nh) / 2.0f);

            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "cloth_com_x", GetCylinderCenterOfMassX(nh) + GetCylinderRadius(nh) * 1.5f + GetClothXSize(nh) / 2.0f);

            case TaskType::CLOTH_SINGLE_POLE:
            case TaskType::CLOTH_WALL:
            default:
                const auto val = ROSHelpers::GetParamRequired<double>(nh, "cloth_com_x", __func__);
                return (float)val.GetImmutable();
        }
    }

    inline float GetClothCenterOfMassY(ros::NodeHandle& nh)   // METERS
    {
        switch(GetTaskType(nh))
        {
            case TaskType::CLOTH_COLAB_FOLDING:
            case TaskType::CLOTH_TABLE_COVERAGE:
            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "cloth_com_y", GetTableSurfaceY(nh));

            case TaskType::CLOTH_SINGLE_POLE:
            case TaskType::CLOTH_WALL:
                return ROSHelpers::GetParam(nh, "cloth_com_y", GetCylinderCenterOfMassY(nh));

            default:
                const auto val = ROSHelpers::GetParamRequired<double>(nh, "cloth_com_y", __func__);
                return (float)val.GetImmutable();
        }
    }

    inline float GetClothCenterOfMassZ(ros::NodeHandle& nh)   // METERS
    {
        switch(GetTaskType(nh))
        {
            case TaskType::CLOTH_COLAB_FOLDING:
            case TaskType::CLOTH_TABLE_COVERAGE:
            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "cloth_com_z", GetTableSurfaceZ(nh) + 0.01f);

            case TaskType::CLOTH_SINGLE_POLE:
            case TaskType::CLOTH_WALL:
                return ROSHelpers::GetParam(nh, "cloth_com_z", GetCylinderCenterOfMassZ(nh));

            default:
                const auto val = ROSHelpers::GetParamRequired<double>(nh, "cloth_com_z", __func__);
                return (float)val.GetImmutable();
        }
    }

    inline float GetClothLinearStiffness(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "cloth_linear_stiffness", __func__);
        return (float)val.GetImmutable();
    }

    ////////////////////////////////////////////////////////////////////////////
    // Cloth BulletPhysics settings
    ////////////////////////////////////////////////////////////////////////////

    inline int GetClothNumControlPointsX(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "cloth_num_control_points_x", 45);
    }

    inline int GetClothNumControlPointsY(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "cloth_num_control_points_y", 45);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Generic target patch settings
    ////////////////////////////////////////////////////////////////////////////

    inline float GetCoverRegionXMin(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "cover_region_x_min", 0.0f);
    }

    inline size_t GetCoverRegionXSteps(ros::NodeHandle& nh)
    {
        const int steps = ROSHelpers::GetParam(nh, "cover_region_x_steps", 1);
        assert(steps > 0);
        return (size_t)steps;
    }

    inline float GetCoverRegionXRes(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "cover_region_x_res", 0.01f);
    }

    inline float GetCoverRegionYMin(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "cover_region_y_min", 0.0f);
    }

    inline size_t GetCoverRegionYSteps(ros::NodeHandle& nh)
    {
        const int steps = ROSHelpers::GetParam(nh, "cover_region_y_steps", 1);
        assert(steps > 0);
        return (size_t)steps;
    }

    inline float GetCoverRegionYRes(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "cover_region_y_res", 0.01f);
    }

    inline float GetCoverRegionZMin(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "cover_region_z_min", 0.0f);
    }

    inline size_t GetCoverRegionZSteps(ros::NodeHandle& nh)
    {
        const int steps = ROSHelpers::GetParam(nh, "cover_region_z_steps", 1);
        assert(steps > 0);
        return (size_t)steps;
    }

    inline float GetCoverRegionZRes(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "cover_region_z_res", 0.01f);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Simulator settings
    ////////////////////////////////////////////////////////////////////////////

    inline double GetFeedbackCovariance(ros::NodeHandle& nh)  // METERS^2
    {
        return ROSHelpers::GetParamDebugLog(nh, "feedback_covariance", 0.0);
    }

    inline size_t GetNumSimstepsPerGripperCommand(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "num_simsteps_per_gripper_command", 4);
    }

    inline float GetSettlingTime(ros::NodeHandle& nh, const float default_time = 4.0)
    {
        return ROSHelpers::GetParam(nh, "settle_time", default_time);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Robot settings
    ////////////////////////////////////////////////////////////////////////////

    inline double GetRobotControlPeriod(ros::NodeHandle& nh) // SECONDS
    {
        return ROSHelpers::GetParamDebugLog(nh, "robot_control_rate", 0.01);
    }

    inline double GetMaxGripperVelocityNorm(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog(nh, "max_gripper_velocity", 0.2);
    }

    inline double GetMaxDOFVelocityNorm(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "max_dof_velocity", 1.0);
    }

    ////////////////////////////////////////////////////////////////////////////
    // World size settings for Graph/Dijkstras - DEFINED IN BULLET FRAME, but WORLD SIZES
    ////////////////////////////////////////////////////////////////////////////

    inline double GetWorldXStep(ros::NodeHandle& nh)    // METERS
    {
        switch(GetDeformableType(nh))
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam(nh, "world_x_step", 0.05);

            case DeformableType::CLOTH:
                return ROSHelpers::GetParam(nh, "world_x_step", 0.02);

            default:
                ROS_FATAL_STREAM_NAMED("params", "Unknown deformable type for " << __func__);
                throw_arc_exception(std::invalid_argument, std::string("Unknown deformable type for ") + __func__);
        }
    }

    inline double GetWorldXMinBulletFrame(ros::NodeHandle& nh)     // METERS
    {
        switch(GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
                return ROSHelpers::GetParam(nh, "world_x_min", GetTableSurfaceX(nh) - GetTableHalfExtentsX(nh));

            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "world_x_min", GetClothCenterOfMassX(nh) - 1.4 * GetClothXSize(nh));

            default:
                const auto val = ROSHelpers::GetParamRequired<double>(nh, "world_x_min", __func__);
                return val.GetImmutable();
        }
    }

    inline double GetWorldXMaxBulletFrame(ros::NodeHandle& nh)     // METERS
    {
        switch(GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
                return ROSHelpers::GetParam(nh, "world_x_max", GetTableSurfaceX(nh) + GetTableHalfExtentsX(nh));

            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "world_x_max", GetClothCenterOfMassX(nh) + 0.4 * GetClothXSize(nh));

            default:
                const auto val = ROSHelpers::GetParamRequired<double>(nh, "world_x_max", __func__);
                return val.GetImmutable();
        }
    }

    inline int64_t GetWorldXNumSteps(ros::NodeHandle& nh)
    {
        return std::lround((GetWorldXMaxBulletFrame(nh) - GetWorldXMinBulletFrame(nh))/GetWorldXStep(nh)) + 1;
    }

    inline double GetWorldYStep(ros::NodeHandle& nh)    // METERS
    {
        switch(GetDeformableType(nh))
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam(nh, "world_y_step", 0.05);

            case DeformableType::CLOTH:
                return ROSHelpers::GetParam(nh, "world_y_step", 0.02);

            default:
                ROS_FATAL_STREAM_NAMED("params", "Unknown deformable type for " << __func__);
                throw_arc_exception(std::invalid_argument, std::string("Unknown deformable type for ") + __func__);
        }
    }

    inline double GetWorldYMinBulletFrame(ros::NodeHandle& nh)     // METERS
    {
        switch(GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
            case TaskType::ROPE_CYLINDER_COVERAGE_TWO_GRIPPERS:
                return ROSHelpers::GetParam(nh, "world_y_min", GetTableSurfaceY(nh) - GetTableHalfExtentsY(nh));

            case TaskType::CLOTH_COLAB_FOLDING:
                return -0.05;

            case TaskType::CLOTH_TABLE_COVERAGE:
                return ROSHelpers::GetParam(nh, "world_y_min", GetClothCenterOfMassY(nh) - 0.65 * GetClothYSize(nh));

            case TaskType::CLOTH_CYLINDER_COVERAGE:
            case TaskType::CLOTH_WAFR:
            case TaskType::CLOTH_SINGLE_POLE:
                return ROSHelpers::GetParam(nh, "world_y_min", GetClothCenterOfMassY(nh) - 0.75 * GetClothYSize(nh));

            default:
                const auto val = ROSHelpers::GetParamRequired<double>(nh, "world_y_min", __func__);
                return val.GetImmutable();
        }
    }

    inline double GetWorldYMaxBulletFrame(ros::NodeHandle& nh)     // METERS
    {
        switch(GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
            case TaskType::ROPE_CYLINDER_COVERAGE_TWO_GRIPPERS:
                return ROSHelpers::GetParam(nh, "world_y_max", GetTableSurfaceY(nh) + GetTableHalfExtentsY(nh));

            case TaskType::CLOTH_COLAB_FOLDING:
                return 0.05;

            case TaskType::CLOTH_TABLE_COVERAGE:
                return ROSHelpers::GetParam(nh, "world_y_max", GetClothCenterOfMassY(nh) + 0.65 * GetClothYSize(nh));

            case TaskType::CLOTH_CYLINDER_COVERAGE:
            case TaskType::CLOTH_WAFR:
            case TaskType::CLOTH_SINGLE_POLE:
                return ROSHelpers::GetParam(nh, "world_y_max", GetClothCenterOfMassY(nh) + 0.75 * GetClothYSize(nh));

            default:
                const auto val = ROSHelpers::GetParamRequired<double>(nh, "world_y_max", __func__);
                return val.GetImmutable();
        }
    }

    inline int64_t GetWorldYNumSteps(ros::NodeHandle& nh)
    {
        return std::lround((GetWorldYMaxBulletFrame(nh) - GetWorldYMinBulletFrame(nh))/GetWorldYStep(nh)) + 1;
    }

    inline double GetWorldZStep(ros::NodeHandle& nh)    // METERS
    {
        switch(GetDeformableType(nh))
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam(nh, "world_z_step", 0.05);

            case DeformableType::CLOTH:
                return ROSHelpers::GetParam(nh, "world_z_step", 0.02);

            default:
                ROS_FATAL_STREAM_NAMED("params", "Unknown deformable type for " << __func__);
                throw_arc_exception(std::invalid_argument, std::string("Unknown deformable type for ") + __func__);
        }
    }

    inline double GetWorldZMinBulletFrame(ros::NodeHandle& nh)     // METERS
    {
        switch(GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
            case TaskType::ROPE_CYLINDER_COVERAGE_TWO_GRIPPERS:
                return ROSHelpers::GetParam(nh, "world_z_min", GetTableSurfaceZ(nh));

            case TaskType::CLOTH_COLAB_FOLDING:
                return -0.05;

            case TaskType::CLOTH_TABLE_COVERAGE:
                return ROSHelpers::GetParam(nh, "world_z_min", GetClothCenterOfMassY(nh) - 0.65 * GetClothXSize(nh));

            case TaskType::CLOTH_CYLINDER_COVERAGE:
            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "world_z_min", GetClothCenterOfMassZ(nh) - 1.0 * GetClothXSize(nh));

            default:
                const auto val = ROSHelpers::GetParamRequired<double>(nh, "world_z_min", __func__);
                return val.GetImmutable();
        }
    }

    inline double GetWorldZMaxBulletFrame(ros::NodeHandle& nh)     // METERS
    {
        switch(GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
            case TaskType::ROPE_CYLINDER_COVERAGE_TWO_GRIPPERS:
                return ROSHelpers::GetParam(nh, "world_z_max", GetTableSurfaceZ(nh) + GetCylinderHeight(nh) + GetRopeRadius(nh) * 5.0);

            case TaskType::CLOTH_COLAB_FOLDING:
                return 0.05;

            case TaskType::CLOTH_TABLE_COVERAGE:
                return ROSHelpers::GetParam(nh, "world_z_max", GetClothCenterOfMassY(nh) + 0.1 * GetClothXSize(nh));

            case TaskType::CLOTH_CYLINDER_COVERAGE:
            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "world_z_max", GetClothCenterOfMassZ(nh) + 0.5 * GetClothXSize(nh));

            default:
                const auto val = ROSHelpers::GetParamRequired<double>(nh, "world_z_max", __func__);
                return val.GetImmutable();
        }
    }

    inline int64_t GetWorldZNumSteps(ros::NodeHandle& nh)
    {
        return std::lround((GetWorldZMaxBulletFrame(nh) - GetWorldZMinBulletFrame(nh))/GetWorldZStep(nh)) + 1;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Planner trial type settings
    ////////////////////////////////////////////////////////////////////////////

    inline PlannerTrialType GetPlannerTrialType(ros::NodeHandle& nh)
    {
        const std::string planner_trial_type = ROSHelpers::GetParamRequired<std::string>(nh, "planner_trial_type", __func__).GetImmutable();

        std::unordered_map<std::string, PlannerTrialType> task_map{
            {"diminishing_rigidity_single_model_least_squares_controller",  PlannerTrialType::DIMINISHING_RIGIDITY_SINGLE_MODEL_LEAST_SQUARES_CONTROLLER},
            {"adaptive_jacobian_single_model_least_squares_controller",     PlannerTrialType::ADAPTIVE_JACOBIAN_SINGLE_MODEL_LEAST_SQUARES_CONTROLLER},
            {"constraint_single_model_constraint_controller",               PlannerTrialType::CONSTRAINT_SINGLE_MODEL_CONSTRAINT_CONTROLLER},
            {"diminishing_rigidity_single_model_constraint_controller",     PlannerTrialType::DIMINISHING_RIGIDITY_SINGLE_MODEL_CONSTRAINT_CONTROLLER},
            {"multi_model_bandit_test",                                     PlannerTrialType::MULTI_MODEL_BANDIT_TEST},
            {"multi_model_controller_test",                                 PlannerTrialType::MULTI_MODEL_CONTROLLER_TEST},
            {"multi_model_accuracy_test",                                   PlannerTrialType::MULTI_MODEL_ACCURACY_TEST}
        };

        try
        {
            return task_map.at(planner_trial_type);
        }
        catch (std::out_of_range& e)
        {
            ROS_FATAL_STREAM_NAMED("params", "Unknown planner trial type type: " << planner_trial_type);
            throw_arc_exception(std::invalid_argument, "Unknown planner trial type: " + planner_trial_type);
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    // Diminishing Rigidity Model Parameters
    ////////////////////////////////////////////////////////////////////////////

    inline double GetDefaultDeformability(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "diminishing_rigidity/default_deformability", __func__);
        return val.GetImmutable();
    }

    ////////////////////////////////////////////////////////////////////////////
    // Adaptive Jacobian Model Parameters
    ////////////////////////////////////////////////////////////////////////////

    inline double GetAdaptiveModelLearningRate(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "adaptive_model/adaptive_model_learning_rate", 1e-6);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Constraint Model Parameters
    ////////////////////////////////////////////////////////////////////////////

    inline double GetConstraintTranslationalDir(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "constraint_model/translational_dir_deformability", __func__).GetImmutable();
    }

    inline double GetConstraintTranslationalDis(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "constraint_model/translational_dis_deformability", __func__).GetImmutable();
    }

    inline double GetConstraintRotational(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "constraint_model/rotational_dis_deformability", __func__).GetImmutable();
    }

    inline double GetConstraintTranslationalOldVersion(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamRequired<double>(nh, "constraint_model/translational_old_version_deformability", __func__).GetImmutable();
    }

    ////////////////////////////////////////////////////////////////////////////
    // Bandit Multi-model settings
    ////////////////////////////////////////////////////////////////////////////

    inline bool GetCollectResultsForAllModels(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "multi_model/collect_results_for_all_models", false);
    }

    inline double GetRewardScaleAnnealingFactor(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "multi_model/reward_scale_annealing_factor", __func__);
        const double factor = val.GetImmutable();
        assert(0.0 <= factor && factor < 1.0);
        return factor;
    }

    inline double GetProcessNoiseFactor(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog(nh, "process_noise_factor", 0.1);
    }

    inline double GetObservationNoiseFactor(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog(nh, "observation_noise_factor", 0.01);
    }

    inline double GetCorrelationStrengthFactor(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog(nh, "correlation_strength_factor", 0.9);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Planner settings
    ////////////////////////////////////////////////////////////////////////////

    inline bool GetUseRandomSeed(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "use_random_seed", false);
    }

    inline size_t GetPlannerSeed(ros::NodeHandle& nh)
    {
        size_t seed;
        if (GetUseRandomSeed(nh))
        {
            assert(nh.hasParam("static_seed") == false);
            seed = std::chrono::system_clock::now().time_since_epoch().count();
        }
        else
        {
            std::string seed_as_string = ROSHelpers::GetParam<std::string>(nh, "static_seed", "a8710913d2b5df6c"); // a30cd67f3860ddb3) // MD5 sum of "Dale McConachie"
            std::stringstream ss;
            ss << std::hex << seed_as_string;
            ss >> seed;
        }
        return seed;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Planner - Stuck detection settings
    ////////////////////////////////////////////////////////////////////////////

    inline bool GetEnableStuckDetection(ros::NodeHandle& nh)
    {
        const auto enable_stuck_detection = ROSHelpers::GetParam<bool>(nh, "enable_stuck_detection", false);
        return enable_stuck_detection;
    }

    inline size_t GetNumLookaheadSteps(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<int>(nh, "stuck_detection/num_lookahead_steps", __func__);
        const int steps = val.GetImmutable();
        assert(steps >= 1);
        return (size_t)steps;
    }

    inline double GetRubberBandOverstretchPredictionAnnealingFactor(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "stuck_detection/band_overstretch_prediction_annealing_factor", __func__);
        const double factor = val.GetImmutable();
        assert(0.0 <= factor && factor < 1.0);
        return factor;
    }

    inline size_t GetMaxGrippersPoseHistoryLength(ros::NodeHandle& nh)
    {
        size_t length = ROSHelpers::GetParam(nh, "stuck_detection/max_pose_history_steps", 20);
        assert(length >= 1);
        return length;
    }

    inline double GetErrorDeltaThresholdForProgress(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "stuck_detection/error_delta_threshold_for_progress", __func__);
        return val.GetImmutable();
    }

    inline double GetGrippersDistanceDeltaThresholdForProgress(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "stuck_detection/grippers_distance_delta_threshold_for_progress", __func__);
        return val.GetImmutable();
    }

    ////////////////////////////////////////////////////////////////////////////
    // Planner - RRT settings
    ////////////////////////////////////////////////////////////////////////////

    inline bool GetRRTReuseOldResults(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<bool>(nh, "rrt/reuse_old_results", __func__);
        return val.GetImmutable();
    }

    inline bool GetRRTStoreNewResults(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<bool>(nh, "rrt/store_new_results", __func__);
        return val.GetImmutable();
    }

    inline double GetRRTHomotopyDistancePenalty()
    {
        return 1e3;
    }

    inline double GetRRTBandDistance2ScalingFactor(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "rrt/band_dist2_scaling_factor", __func__);
        return val.GetImmutable();
    }

    inline size_t GetRRTBandMaxPoints(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<int>(nh, "rrt/band_max_points_", __func__);
        return (size_t)val.GetImmutable();
    }

    inline double GetRRTMaxRobotDOFStepSize(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "rrt/max_robot_dof_step_size", __func__);
        return val.GetImmutable();
    }

    inline double GetRRTMinRobotDOFStepSize(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "rrt/min_robot_dof_step_size", __func__);
        return val.GetImmutable();
    }

    inline double GetRRTMaxGripperRotation(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "rrt/max_gripper_rotation", __func__);
        return val.GetImmutable();
    }

    inline double GetRRTGoalBias(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "rrt/goal_bias", __func__);
        return val.GetImmutable();
    }

    inline int64_t GetRRTMaxShortcutIndexDistance(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<int>(nh, "rrt/max_shortcut_index_distance", __func__);
        return (int64_t)val.GetImmutable();
    }

    inline uint32_t GetRRTMaxSmoothingIterations(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<int>(nh, "rrt/max_smoothing_iterations",  __func__);
        return (uint32_t)val.GetImmutable();
    }

    inline uint32_t GetRRTMaxFailedSmoothingIterations(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<int>(nh, "rrt/max_failed_smoothing_iterations", __func__);
        return (uint32_t)val.GetImmutable();
    }

    inline double GetRRTTimeout(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "rrt/timeout", __func__);
        assert(val.GetImmutable() > 0.0);
        return val.GetImmutable();
    }

    inline double GetRRTPlanningXMinBulletFrame(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "rrt/planning_x_min", __func__);
        return val.GetImmutable();
    }

    inline double GetRRTPlanningXMaxBulletFrame(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "rrt/planning_x_max", __func__);
        return val.GetImmutable();
    }

    inline double GetRRTPlanningYMinBulletFrame(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "rrt/planning_y_min", __func__);
        return val.GetImmutable();
    }

    inline double GetRRTPlanningYMaxBulletFrame(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "rrt/planning_y_max", __func__);
        return val.GetImmutable();
    }

    inline double GetRRTPlanningZMinBulletFrame(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "rrt/planning_z_min", __func__);
        return val.GetImmutable();
    }

    inline double GetRRTPlanningZMaxBulletFrame(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "rrt/planning_z_max", __func__);
        return val.GetImmutable();
    }

    inline bool GetUseCBiRRTStyleProjection(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<bool>(nh, "rrt/use_cbirrt_style_projection", __func__);
        return val.GetImmutable();
    }

    inline size_t GetRRTForwardTreeExtendIterations(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<int>(nh, "rrt/forward_tree_extend_iterations", __func__);
        return (size_t)val.GetImmutable();
    }

    inline size_t GetRRTBackwardTreeExtendIterations(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<int>(nh, "rrt/backward_tree_extend_iterations", __func__);
        return (size_t)val.GetImmutable();
    }

    inline bool GetUseBruteForceNN(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<bool>(nh, "rrt/use_brute_force_nn", __func__);
        return val.GetImmutable();
    }

    inline size_t GetKdTreeGrowThreshold(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<int>(nh, "rrt/kd_tree_grow_threshold", __func__);
        return (size_t)val.GetImmutable();
    }

    ////////////////////////////////////////////////////////////////////////////
    // Planner - PRM settings
    ////////////////////////////////////////////////////////////////////////////

    inline size_t GetPRMNumNearest(ros::NodeHandle& nh, const size_t default_k = 5)
    {
        const int retrieved_k = ROSHelpers::GetParam(nh, "prm/num_nearest", (int)default_k);
        assert(retrieved_k > 0);
        return (size_t)retrieved_k;
    }

    inline size_t GetPRMNumSamples(ros::NodeHandle& nh, const size_t default_num_samples = 1000)
    {
        const int retrieved_num_samples = ROSHelpers::GetParam(nh, "prm/num_samples", (int)default_num_samples);
        assert(retrieved_num_samples > 0);
        return (size_t)retrieved_num_samples;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Pure Jacobian based motion controller paramters
    ////////////////////////////////////////////////////////////////////////////

    inline bool GetJacobianControllerOptimizationEnabled(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "jacobian_controller/optimization_enabled", false);
    }

    inline double GetCollisionScalingFactor(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "jacobian_controller/collision_scaling_factor", __func__);
        return val.GetImmutable();
    }

    ////////////////////////////////////////////////////////////////////////////
    // Stretching avoidance controller parameters
    ////////////////////////////////////////////////////////////////////////////

    inline StretchingAvoidanceControllerSolverType GetStretchingAvoidanceControllerSolverType(ros::NodeHandle& nh)
    {
        std::string solver_type = ROSHelpers::GetParam<std::string>(nh, "stretching_avoidance_controller/solver_type", "random_sampling");

        std::unordered_map<std::string, StretchingAvoidanceControllerSolverType> solver_map {
            {"random_sampling",     StretchingAvoidanceControllerSolverType::RANDOM_SAMPLING},
            {"nomad_optimization",  StretchingAvoidanceControllerSolverType::NOMAD_OPTIMIZATION},
            {"gradient_descent",    StretchingAvoidanceControllerSolverType::GRADIENT_DESCENT},
        };

        try
        {
            return solver_map.at(solver_type);
        }
        catch (std::out_of_range& e)
        {
            ROS_FATAL_STREAM_NAMED("params", "Unknown solver type: " << solver_type);
            throw_arc_exception(std::invalid_argument, "Unknown solver type: " + solver_type);
        }
    }

    inline int64_t GetMaxSamplingCounts(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<int>(nh, "stretching_avoidance_controller/max_sampling_counts", __func__);
        return val.GetImmutable();
    }

    inline bool GetUseFixedGripperDeltaSize(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<bool>(nh, "stretching_avoidance_controller/fix_step_size", __func__);
        return val.GetImmutable();
    }

    inline double GetStretchingCosineThreshold(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "stretching_avoidance_controller/stretching_cosine_threshold", __func__);
        return val.GetImmutable();
    }

    ////////////////////////////////////////////////////////////////////////////
    // Straight line motion parameters for testing model accuracy
    // Note: these parameters are gripper velocities *in gripper frame*
    ////////////////////////////////////////////////////////////////////////////

    inline std::pair<std::vector<double>, std::vector<Eigen::Matrix<double, 6, 1>>> GetGripperDeltaTrajectory(ros::NodeHandle& nh, const std::string& gripper_name)
    {
        const std::string base_param_name = "straight_line_motion_controller/" + gripper_name + "_deltas/";

//        const std::vector<double> t = ROSHelpers::GetParamRequired<std::vector<double>>(nh, base_param_name + "t", __func__).GetImmutable();
        std::vector<double> t;
        if (!nh.getParam(base_param_name + "t", t))
        {
            ROS_FATAL_STREAM_NAMED("params", "Cannot find " << nh.getNamespace() << "/" << base_param_name + "t" << " on parameter server for " << __func__ << ": Value must be on paramter sever");
            throw_arc_exception(std::runtime_error, "Unable to find parameter on server");
        }

//        const std::vector<double> vx = ROSHelpers::GetParamRequired<std::vector<double>>(nh, base_param_name + "/vx", __func__).GetImmutable();
//        const std::vector<double> vy = ROSHelpers::GetParamRequired<std::vector<double>>(nh, base_param_name + "/vy", __func__).GetImmutable();
//        const std::vector<double> vz = ROSHelpers::GetParamRequired<std::vector<double>>(nh, base_param_name + "/vz", __func__).GetImmutable();
//        const std::vector<double> wx = ROSHelpers::GetParamRequired<std::vector<double>>(nh, base_param_name + "/wx", __func__).GetImmutable();
//        const std::vector<double> wy = ROSHelpers::GetParamRequired<std::vector<double>>(nh, base_param_name + "/wy", __func__).GetImmutable();
//        const std::vector<double> wz = ROSHelpers::GetParamRequired<std::vector<double>>(nh, base_param_name + "/wz", __func__).GetImmutable();

        std::vector<double> vx;
        if (!nh.getParam(base_param_name + "vx", vx))
        {
            ROS_FATAL_STREAM_NAMED("params", "Cannot find " << nh.getNamespace() << "/" <<base_param_name + "vx" << " on parameter server for " << __func__ << ": Value must be on paramter sever");
            throw_arc_exception(std::runtime_error, "Unable to find parameter on server");
        }

        std::vector<double> vy;
        if (!nh.getParam(base_param_name + "vy", vy))
        {
            ROS_FATAL_STREAM_NAMED("params", "Cannot find " << nh.getNamespace() << "/" <<base_param_name + "vy" << " on parameter server for " << __func__ << ": Value must be on paramter sever");
            throw_arc_exception(std::runtime_error, "Unable to find parameter on server");
        }

        std::vector<double> vz;
        if (!nh.getParam(base_param_name + "vz", vz))
        {
            ROS_FATAL_STREAM_NAMED("params", "Cannot find " << nh.getNamespace() << "/" <<base_param_name + "vz" << " on parameter server for " << __func__ << ": Value must be on paramter sever");
            throw_arc_exception(std::runtime_error, "Unable to find parameter on server");
        }

        std::vector<double> wx;
        if (!nh.getParam(base_param_name + "wx", wx))
        {
            ROS_FATAL_STREAM_NAMED("params", "Cannot find " << nh.getNamespace() << "/" <<base_param_name + "wx" << " on parameter server for " << __func__ << ": Value must be on paramter sever");
            throw_arc_exception(std::runtime_error, "Unable to find parameter on server");
        }

        std::vector<double> wy;
        if (!nh.getParam(base_param_name + "wy", wy))
        {
            ROS_FATAL_STREAM_NAMED("params", "Cannot find " << nh.getNamespace() << "/" <<base_param_name + "wy" << " on parameter server for " << __func__ << ": Value must be on paramter sever");
            throw_arc_exception(std::runtime_error, "Unable to find parameter on server");
        }

        std::vector<double> wz;
        if (!nh.getParam(base_param_name + "wz", wz))
        {
            ROS_FATAL_STREAM_NAMED("params", "Cannot find " << nh.getNamespace() << "/" <<base_param_name + "wz" << " on parameter server for " << __func__ << ": Value must be on paramter sever");
            throw_arc_exception(std::runtime_error, "Unable to find parameter on server");
        }

        assert(t.size() == vx.size());
        assert(t.size() == vy.size());
        assert(t.size() == vz.size());
        assert(t.size() == wx.size());
        assert(t.size() == wy.size());

        std::vector<Eigen::Matrix<double, 6, 1>> deltas(t.size());assert(t.size() == wz.size());
        for (size_t ind = 0; ind < t.size(); ++ind)
        {
            deltas[ind](0) = vx[ind];
            deltas[ind](1) = vy[ind];
            deltas[ind](2) = vz[ind];
            deltas[ind](3) = wx[ind];
            deltas[ind](4) = wy[ind];
            deltas[ind](5) = wz[ind];
        }

        return {t, deltas};
    }

    inline double GetGripperStraightLineMotionTransX(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "straight_line_motion_controller/vx", __func__);
        return val.GetImmutable();
    }

    inline double GetGripperStraightLineMotionTransY(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "straight_line_motion_controller/vy", __func__);
        return val.GetImmutable();
    }

    inline double GetGripperStraightLineMotionTransZ(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "straight_line_motion_controller/vz", __func__);
        return val.GetImmutable();
    }

    inline double GetGripperStraightLineMotionAngularX(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "straight_line_motion_controller/wx", __func__);
        return val.GetImmutable();
    }

    inline double GetGripperStraightLineMotionAngularY(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "straight_line_motion_controller/wy", __func__);
        return val.GetImmutable();
    }

    inline double GetGripperStraightLineMotionAngularZ(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "straight_line_motion_controller/wz", __func__);
        return val.GetImmutable();
    }

    ////////////////////////////////////////////////////////////////////////////
    // Logging functionality
    ////////////////////////////////////////////////////////////////////////////

    inline bool GetPlannerLoggingEnabled(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "planner_logging_enabled", false);
    }

    inline bool GetControllerLoggingEnabled(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "controller_logging_enabled", false);
    }

    inline std::string GetLogFolder(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "log_folder", "/tmp/");
    }

    inline std::string GetDijkstrasStorageLocation(ros::NodeHandle& nh)
    {
        const std::string base_path = ros::package::getPath("smmap");
        const std::string task_name = ROSHelpers::GetParamRequired<std::string>(nh, "task_type", __func__).GetImmutable();
        const std::string default_dijkstras_file_path =
                base_path + "/logs/"
                + task_name + "/"
                + task_name + ".dijkstras_serialized";
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "dijkstras_file_path", default_dijkstras_file_path);
    }

    inline bool GetScreenshotsEnabled(ros::NodeHandle& nh)
    {
        const bool screenshots_enabled = ROSHelpers::GetParam(nh, "screenshots_enabled", false);
        // The viewer must be enabled for screen shots to be enabled
        assert(!screenshots_enabled || ROSHelpers::GetParam(nh, "start_bullet_viewer", true));
        return screenshots_enabled;
    }

    inline std::string GetScreenshotFolder(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "screenshot_folder", GetLogFolder(nh) + "screenshots/");
    }

    ////////////////////////////////////////////////////////////////////////////
    // ROS Topic settings
    ////////////////////////////////////////////////////////////////////////////

    inline std::string GetTestRobotMotionTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "test_robot_motion_topic", "test_robot_motion");
    }

    inline std::string GetExecuteRobotMotionTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "execute_robot_motion_topic", "execute_robot_motion");
    }

    inline std::string GetWorldStateTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "world_state_topic", "world_state");
    }

    inline std::string GetCoverPointsTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "get_cover_points_topic", "get_cover_points");
    }

    inline std::string GetCoverPointNormalsTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "get_cover_point_normals_topic", "get_cover_point_normals");
    }

    inline std::string GetMirrorLineTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "get_mirror_line_topic", "get_mirror_line");
    }

    inline std::string GetFreeSpaceGraphTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "get_free_space_graph_topic", "get_free_space_graph");
    }

    inline std::string GetSignedDistanceFieldTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "get_signed_distance_field_topic", "get_signed_distance_field");
    }

    inline std::string GetGripperNamesTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "get_gripper_names_topic", "get_gripper_names");
    }

    inline std::string GetGripperAttachedNodeIndicesTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "get_gripper_attached_node_indices_topic", "get_gripper_attached_node_indices");
    }

    // Get Stretching Vector information for cloth two grippers
    inline std::string GetGripperStretchingVectorInfoTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "get_gripper_stretching_vector_topic", "get_gripper_stretching_vector");
    }

    inline std::string GetGripperPoseTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "get_gripper_pose_topic", "get_gripper_pose");
    }

    inline std::string GetRobotConfigurationTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "get_robot_configuration_topic", "get_robot_configuration");
    }

    inline std::string GetObjectInitialConfigurationTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "get_object_initial_configuration_topic", "get_object_initial_configuration");
    }

    inline std::string GetObjectCurrentConfigurationTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "get_object_current_configuration_topic", "get_object_current_configuration");
    }

    inline std::string GetVisualizationMarkerTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "visualization_marker_topic", "visualization_marker");
    }

    inline std::string GetVisualizationMarkerArrayTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "visualization_marker_array_topic", "visualization_marker_vector");
    }

    inline std::string GetClearVisualizationsTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "clear_visualizations_topic", "clear_visualizations");
    }

    inline std::string GetConfidenceTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "confidence_topic", "confidence");
    }

    inline std::string GetConfidenceImageTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "confidence_image_topic", "confidence_image");
    }

    inline std::string GetGripperCollisionCheckTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "get_gripper_collision_check_topic", "get_gripper_collision_check");
    }

    inline std::string GetTerminateSimulationTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamDebugLog<std::string>(nh, "terminate_simulation_topic", "terminate_simulation");
    }

    ////////////////////////////////////////////////////////////////////////////
    // Live Robot Settings
    ////////////////////////////////////////////////////////////////////////////

    inline std::string GetDeformableObjectEstimateTopic()
    {
        return "object_estimate";
    }

    inline std::string GetARTagTopic()
    {
        return "ar_pose_marker";
    }

    inline std::string GetApiltagTopic()
    {
        return "apriltag_kinect2/detections";
    }

    inline std::string GetGripper0Name(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam<std::string>(nh, "gripper0_name", "left");
    }

    inline std::string GetGripper1Name(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam<std::string>(nh, "gripper1_name", "right");
    }

    inline std::string GetGripper0TFName(ros::NodeHandle& nh)
    {
        const std::string name = GetGripper0Name(nh);
        assert(name == "left" || name == "right");
        const bool use_victor = ROSHelpers::GetParamRequired<bool>(nh, "use_victor", __func__).GetImmutable();
        const bool use_val = ROSHelpers::GetParamRequired<bool>(nh, "use_val", __func__).GetImmutable();
        assert ((use_victor ^ use_val) && "Only one of Victor or Val can be specified");
        if (use_victor)
        {
            return "victor_" + name + "_gripper";
        }
        else if (use_val)
        {
            return name + "gripper_tip";
        }
        else
        {
            assert(false && "This should not be possible");
        }
    }

    inline std::string GetGripper1TFName(ros::NodeHandle& nh)
    {
        const std::string name = GetGripper1Name(nh);
        assert(name == "left" || name == "right");
        const bool use_victor = ROSHelpers::GetParamRequired<bool>(nh, "use_victor", __func__).GetImmutable();
        const bool use_val = ROSHelpers::GetParamRequired<bool>(nh, "use_val", __func__).GetImmutable();
        assert ((use_victor ^ use_val) && "Only one of Victor or Val can be specified");
        if (use_victor)
        {
            return "victor_" + name + "_gripper";
        }
        else if (use_val)
        {
            return name + "gripper_tip";
        }
        else
        {
            assert(false && "This should not be possible");
        }
    }

    inline size_t GetGripperAttachedIdx(ros::NodeHandle& nh, const std::string& gripper_name)
    {
        const auto val = ROSHelpers::GetParamRequired<int>(nh, gripper_name + "_gripper_attached_node_idx", __func__);
        const int idx = val.GetImmutable();
        assert(idx >= 0);
        return (size_t)idx;
    }

    inline double GetClothFilterMaxDistanceScaleFactor(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "max_distance_scale_factor", __func__);
        const auto scale_factor = val.GetImmutable();
        assert(scale_factor >= 1.0);
        return scale_factor;
    }

    inline double GetClothFilterTimeScaleFactor(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "time_scale_factor", __func__);
        const auto scale_factor = val.GetImmutable();
        assert(scale_factor >= 1.0);
        return scale_factor;
    }

    ////////////////////////////////////////////////////////////////////////////
    // ROS TF Frame name settings
    ////////////////////////////////////////////////////////////////////////////

    inline std::string GetBulletFrameName()
    {
        return "bullet_origin";
    }

    inline std::string GetTaskFrameName()
    {
        return GetBulletFrameName();
    }

    inline std::string GetWorldFrameName()
    {
        return "world_origin";
    }

    inline std::string GetTableFrameName()
    {
        return "table_surface";
    }

    inline std::string GetClothPositionFilterOutputFrame(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam<std::string>(nh, "output_frame", "world_origin");
    }
}

#endif // ROS_PARAMS_HPP
