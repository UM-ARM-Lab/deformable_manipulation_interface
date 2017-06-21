#ifndef ROS_PARAMS_HPP
#define ROS_PARAMS_HPP

#include <cmath>
#include <string>
#include <chrono>
#include <arc_utilities/ros_helpers.hpp>
#include <arc_utilities/arc_exceptions.hpp>

#include "deformable_manipulation_experiment_params/task_enums.h"

namespace smmap
{
    inline float GetClothXSize(ros::NodeHandle &nh);

    ////////////////////////////////////////////////////////////////////////////
    // Task and Deformable Type parameters
    ////////////////////////////////////////////////////////////////////////////

    inline DeformableType GetDeformableType(ros::NodeHandle& nh)
    {
        std::string deformable_type = ROSHelpers::GetParam<std::string>(nh, "deformable_type", "rope");

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
            ROS_FATAL_STREAM("Unknown deformable type: " << deformable_type);
            throw_arc_exception(std::invalid_argument, "Unknown deformable type: " + deformable_type);
        }
    }

    inline TaskType GetTaskType(ros::NodeHandle& nh)
    {
        std::string task_type = ROSHelpers::GetParam<std::string>(nh, "task_type", "coverage");

        if (task_type.compare("rope_cylinder_coverage") == 0)
        {
            return TaskType::ROPE_CYLINDER_COVERAGE;
        }
        else if (task_type.compare("cloth_cylinder_coverage") == 0)
        {
            return TaskType::CLOTH_CYLINDER_COVERAGE;
        }
        else if (task_type.compare("cloth_table_coverage") == 0)
        {
            return TaskType::CLOTH_TABLE_COVERAGE;
        }
        else if (task_type.compare("cloth_colab_folding") == 0)
        {
            return TaskType::CLOTH_COLAB_FOLDING;
        }
        else if (task_type.compare("cloth_wafr") == 0)
        {
            return TaskType::CLOTH_WAFR;
        }
        else if (task_type.compare("cloth_single_pole") == 0)
        {
            return TaskType::CLOTH_SINGLE_POLE;
        }
        else if (task_type.compare("cloth_wall") == 0)
        {
            return TaskType::CLOTH_WALL;
        }
        else if (task_type.compare("cloth_double_slit") == 0)
        {
            return TaskType::CLOTH_DOUBLE_SLIT;
        }
        else if (task_type.compare("rope_maze") == 0)
        {
            return TaskType::ROPE_MAZE;
        }
        else
        {
            ROS_FATAL_STREAM("Unknown task type: " << task_type);
            throw_arc_exception(std::invalid_argument, "Unknown task type: " + task_type);
        }
    }

    inline double GetMaxTime(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "max_time", __func__);
        return val.GetImmutable();
    }

    inline bool GetDisableAllVisualizations(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<bool>(nh, "disable_all_visualizations", __func__);
        return val.GetImmutable();
    }

    ////////////////////////////////////////////////////////////////////////////
    // Error calculation settings
    ////////////////////////////////////////////////////////////////////////////

    inline double GetErrorThresholdAlongNormal(ros::NodeHandle& nh)
    {
        switch(GetDeformableType(nh))
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam(nh, "error_threshold_along_normal", 0.01);

            case DeformableType::CLOTH:
                return ROSHelpers::GetParam(nh, "error_threshold_along_normal", 0.002);
        }
    }

    inline double GetErrorThresholdDistanceToNormal(ros::NodeHandle& nh)
    {
        switch(GetDeformableType(nh))
        {
            case DeformableType::ROPE:
                return ROSHelpers::GetParam(nh, "error_threshold_distance_to_normal", 0.01);

            case DeformableType::CLOTH:
                return ROSHelpers::GetParam(nh, "error_threshold_distance_to_normal", 0.002);
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    // Gripper Size Settings
    ////////////////////////////////////////////////////////////////////////////

    inline float GetGripperApperture(ros::NodeHandle& nh) // METERS
    {
        switch(GetDeformableType(nh))
        {
            case DeformableType::ROPE:
                // TODO: why did Dmitry's code use 0.5f here?
                return ROSHelpers::GetParamNoWarn(nh, "rope_gripper_apperture", 0.03f);

            case DeformableType::CLOTH:
                // TODO: This number is actually the "closed gap"
                //       The original number was 0.1f
                return ROSHelpers::GetParamNoWarn(nh, "cloth_gripper_apperture", 0.006f);
        }
    }

    inline double GetGripperRadius()
    {
        return 0.023;
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
        switch (GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
                return ROSHelpers::GetParam(nh, "table_x_half_extents", 1.5f);

            case TaskType::CLOTH_TABLE_COVERAGE:
                return ROSHelpers::GetParam(nh, "table_x_half_extents", 0.2f);

            default:
                const auto val = ROSHelpers::GetParamRequired<double>(nh, "table_x_half_extents", __func__);
                return (float)val.GetImmutable();
        }
    }

    inline float GetTableHalfExtentsY(ros::NodeHandle& nh)  // METERS
    {
        switch (GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
                return ROSHelpers::GetParam(nh, "table_y_half_extents", 1.5f);

            case TaskType::CLOTH_TABLE_COVERAGE:
                return ROSHelpers::GetParam(nh, "table_y_half_extents", 0.2f);

            default:
                const auto val = ROSHelpers::GetParamRequired<double>(nh, "table_y_half_extents", __func__);
                return (float)val.GetImmutable();
        }
    }

    inline float GetTableHeight(ros::NodeHandle& nh)         // METERS
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
    ////////////////////////////////////////////////////////////////////////////

    inline float GetCylinderRadius(ros::NodeHandle& nh)           // METERS
    {
        switch (GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
                return ROSHelpers::GetParam(nh, "rope_cylinder_radius", 0.15f);

            case TaskType::CLOTH_CYLINDER_COVERAGE:
            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "cloth_cylinder_radius", 0.10f);

            case TaskType::CLOTH_SINGLE_POLE:
            case TaskType::CLOTH_WALL:
                return ROSHelpers::GetParam(nh, "cloth_cylinder_radius", 0.04f);

            default:
                throw_arc_exception(std::invalid_argument, "Unknown cylinder radius for task type " + std::to_string(GetTaskType(nh)));
        }
    }

    inline float GetCylinderHeight(ros::NodeHandle& nh)           // METERS
    {
        switch (GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
                return ROSHelpers::GetParam(nh, "rope_cylinder_height", 0.3f);

            case TaskType::CLOTH_CYLINDER_COVERAGE:
            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "cloth_cylinder_height", 0.3f);

            case TaskType::CLOTH_SINGLE_POLE:
            case TaskType::CLOTH_WALL:
                return ROSHelpers::GetParam(nh, "cloth_cylinder_height", 1.0f);

            default:
                throw_arc_exception(std::invalid_argument, "Unknown cylinder height for task type " + std::to_string(GetTaskType(nh)));
        }
    }

    inline float GetCylinderCenterOfMassX(ros::NodeHandle& nh)    // METERS
    {
        switch (GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
                return ROSHelpers::GetParam(nh, "rope_cylinder_com_x", GetTableSurfaceX(nh));

            case TaskType::CLOTH_CYLINDER_COVERAGE:
            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "cloth_cylinder_com_x", GetTableSurfaceX(nh) - GetClothXSize(nh));

            case TaskType::CLOTH_SINGLE_POLE:
            case TaskType::CLOTH_WALL:
                return ROSHelpers::GetParam(nh, "cloth_cylinder_com_x", -0.3f);

            default:
                throw_arc_exception(std::invalid_argument, "Unknown cylinder com for task type " + std::to_string(GetTaskType(nh)));
        }
    }

    inline float GetCylinderCenterOfMassY(ros::NodeHandle& nh)    // METERS
    {
        switch (GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
                return ROSHelpers::GetParam(nh, "rope_cylinder_com_y", GetTableSurfaceY(nh) + GetCylinderRadius(nh) * 5.0f / 3.0f);

            case TaskType::CLOTH_CYLINDER_COVERAGE:
            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "cloth_cylinder_com_y", GetTableSurfaceY(nh));

            case TaskType::CLOTH_SINGLE_POLE:
            case TaskType::CLOTH_WALL:
                return ROSHelpers::GetParam(nh, "cloth_cylinder_com_y", 0.0f);

            default:
                throw_arc_exception(std::invalid_argument, "Unknown cylinder com for task type " + std::to_string(GetTaskType(nh)));
        }
    }

    inline float GetCylinderCenterOfMassZ(ros::NodeHandle& nh)    // METERS
    {
        switch (GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
                return ROSHelpers::GetParam(nh, "rope_cylinder_com_z", GetTableSurfaceZ(nh) + GetCylinderHeight(nh) / 2.0f);

            case TaskType::CLOTH_CYLINDER_COVERAGE:
            case TaskType::CLOTH_WAFR:
                return ROSHelpers::GetParam(nh, "cloth_cylinder_com_z", GetTableSurfaceZ(nh));

            case TaskType::CLOTH_SINGLE_POLE:
            case TaskType::CLOTH_WALL:
                return ROSHelpers::GetParam(nh, "cloth_cylinder_com_z", 1.0f);

            default:
                throw_arc_exception(std::invalid_argument, "Unknown cylinder com for task type " + std::to_string(GetTaskType(nh)));
        }
    }

    ////////////////////////////////////////////////////////////////////////////
    // Wall Size and Visibility Settings
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
    // Rope-Cylinder experiment settings
    ////////////////////////////////////////////////////////////////////////////

    inline float GetRopeCenterOfMassX(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "rope_com_x", GetTableSurfaceX(nh) + 15.0f * GetRopeSegmentLength(nh));
    }

    inline float GetRopeCenterOfMassY(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "rope_com_y", GetTableSurfaceY(nh));
    }

    inline float GetRopeCenterOfMassZ(ros::NodeHandle& nh)    // METERS
    {
        return ROSHelpers::GetParam(nh, "rope_com_z", GetTableSurfaceZ(nh) + 5.0f * GetRopeRadius(nh));
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
        switch(GetTaskType(nh))
        {
            case TaskType::CLOTH_TABLE_COVERAGE:
                return ROSHelpers::GetParam(nh, "cloth_linear_stiffness", 0.8f);

            case TaskType::CLOTH_COLAB_FOLDING:
            case TaskType::CLOTH_WAFR:
            case TaskType::CLOTH_SINGLE_POLE:
            case TaskType::CLOTH_WALL:
            case TaskType::CLOTH_DOUBLE_SLIT:
                return ROSHelpers::GetParam(nh, "cloth_linear_stiffness", 0.5f);

            default:
                throw_arc_exception(std::invalid_argument, "Unknown cloth linear stiffness for task type " + std::to_string(GetTaskType(nh)));
        }
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

    inline float GetCoverRegionXMin(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "cover_region_x_min", 0.0f);
    }

    inline size_t GetCoverRegionXSteps(ros::NodeHandle& nh)
    {
        const int steps = ROSHelpers::GetParam(nh, "cover_region_x_steps", 1);
        assert(steps > 0);
        return (size_t)steps;
    }

    inline float GetCoverRegionXRes(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "cover_region_x_res", 0.01f);
    }

    inline float GetCoverRegionYMin(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "cover_region_y_min", 0.0f);
    }

    inline size_t GetCoverRegionYSteps(ros::NodeHandle& nh)
    {
        const int steps = ROSHelpers::GetParam(nh, "cover_region_y_steps", 1);
        assert(steps > 0);
        return (size_t)steps;
    }

    inline float GetCoverRegionYRes(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "cover_region_y_res", 0.01f);
    }

    inline float GetCoverRegionZMin(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "cover_region_z_min", 0.0f);
    }

    inline size_t GetCoverRegionZSteps(ros::NodeHandle& nh)
    {
        const int steps = ROSHelpers::GetParam(nh, "cover_region_z_steps", 1);
        assert(steps > 0);
        return (size_t)steps;
    }

    inline float GetCoverRegionZRes(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "cover_region_z_res", 0.01f);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Simulator settings
    ////////////////////////////////////////////////////////////////////////////

    inline double GetFeedbackCovariance(ros::NodeHandle& nh)  // METERS^2
    {
        return ROSHelpers::GetParamNoWarn(nh, "feedback_covariance", 0.0);
    }

    inline size_t GetNumSimstepsPerGripperCommand(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "num_simsteps_per_gripper_command", 4);
    }

    ////////////////////////////////////////////////////////////////////////////
    // (Fake) Robot settings
    ////////////////////////////////////////////////////////////////////////////

    inline double GetRobotControlPeriod(ros::NodeHandle& nh) // SECONDS
    {
        return ROSHelpers::GetParamNoWarn(nh, "robot_control_rate", 0.01);
    }

    inline double GetMaxGripperVelocity(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamNoWarn(nh, "max_gripper_velocity", 0.2);
    }

    ////////////////////////////////////////////////////////////////////////////
    // World size settings for Graph/Dijkstras
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
                ROS_FATAL_STREAM("Unknown deformable type for " << __func__);
                throw_arc_exception(std::invalid_argument, std::string("Unknown deformable type for ") + __func__);
        }
    }

    inline double GetWorldXMin(ros::NodeHandle& nh)     // METERS
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

    inline double GetWorldXMax(ros::NodeHandle& nh)     // METERS
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
        return std::lround((GetWorldXMax(nh) - GetWorldXMin(nh))/GetWorldXStep(nh)) + 1;
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
                ROS_FATAL_STREAM("Unknown deformable type for " << __func__);
                throw_arc_exception(std::invalid_argument, std::string("Unknown deformable type for ") + __func__);
        }
    }

    inline double GetWorldYMin(ros::NodeHandle& nh)     // METERS
    {
        switch(GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
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

    inline double GetWorldYMax(ros::NodeHandle& nh)     // METERS
    {
        switch(GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
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
        return std::lround((GetWorldYMax(nh) - GetWorldYMin(nh))/GetWorldYStep(nh)) + 1;
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
                ROS_FATAL_STREAM("Unknown deformable type for " << __func__);
                throw_arc_exception(std::invalid_argument, std::string("Unknown deformable type for ") + __func__);
        }
    }

    inline double GetWorldZMin(ros::NodeHandle& nh)     // METERS
    {
        switch(GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
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

    inline double GetWorldZMax(ros::NodeHandle& nh)     // METERS
    {
        switch(GetTaskType(nh))
        {
            case TaskType::ROPE_CYLINDER_COVERAGE:
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
        return std::lround((GetWorldZMax(nh) - GetWorldZMin(nh))/GetWorldZStep(nh)) + 1;
    }

    ////////////////////////////////////////////////////////////////////////////
    // Planner settings
    ////////////////////////////////////////////////////////////////////////////

    inline bool GetUseMultiModel(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "use_multi_model", false);
    }

    inline bool GetUseAdaptiveModel(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "use_adaptive_model", false);
    }

    inline bool GetUseConstraintModel(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "use_constraint_model", false);
    }

    inline double GetAdaptiveModelLearningRate(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "adaptive_model_learning_rate", 1e-6);
    }

    inline double GetProcessNoiseFactor(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamNoWarn(nh, "process_noise_factor", 0.1);
    }

    inline double GetObservationNoiseFactor(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamNoWarn(nh, "observation_noise_factor", 0.01);
    }

    inline double GetCorrelationStrengthFactor(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamNoWarn(nh, "correlation_strength_factor", 0.9);
    }

    inline bool GetOptimizationEnabled(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "optimization_enabled", false);
    }

    inline double GetRobotMinGripperDistance()
    {
        return 0.005;
    }

    inline bool GetCalculateRegret(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "calculate_regret", false);
    }

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


    inline size_t GetNumLookaheadSteps(ros::NodeHandle& nh)
    {
        const size_t steps = ROSHelpers::GetParamNoWarn(nh, "num_lookahead_steps", 20);
        assert(steps >= 1);
        return steps;
    }

    inline size_t GetMaxGrippersPoseHistoryLength(ros::NodeHandle& nh)
    {
        size_t length = ROSHelpers::GetParam(nh, "max_pose_history_steps", 20);
        assert(length >= 1);
        return length;
    }

    inline double GetErrorDeltaThresholdForProgress(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "error_delta_threshold_for_progress", __func__);
        return val.GetImmutable();
    }

    inline double GetGrippersDistanceDeltaThresholdForProgress(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "grippers_distance_delta_threshold_for_progress", __func__);
        return val.GetImmutable();
    }

    inline bool GetReuseOldRRTResults(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<bool>(nh, "reuse_old_rrt_results", __func__);
        return val.GetImmutable();
    }

    inline double GetRRTHomotopyDistancePenalty()
    {
        return 1e3;
    }

    inline int64_t GetRRTMaxShortcutIndexDistance(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<int>(nh, "rrt_max_shortcut_index_distance", __func__);
        return (int64_t)val.GetImmutable();
    }

    inline uint32_t GetRRTMaxSmoothingIterations(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<int>(nh, "rrt_max_smoothing_iterations",  __func__);
        return (uint32_t)val.GetImmutable();
    }

    inline uint32_t GetRRTMaxFailedSmoothingIterations(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<int>(nh, "rrt_max_failed_smoothing_iterations", __func__);
        return (uint32_t)val.GetImmutable();
    }

    inline double GetRRTTimeout(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamNoWarn(nh, "rrt_timeout", 3600.0);
    }

    ////////////////////////////////////////////////////////////////////////////
    // Diminishing Rigidity Parameters
    ////////////////////////////////////////////////////////////////////////////

    inline double GetDefaultDeformability(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "default_deformability", __func__);
        return val.GetImmutable();
    }

    inline double GetCollisionScalingFactor(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "collision_scaling_factor", __func__);
        return val.GetImmutable();
    }

    inline double GetStretchingThreshold(ros::NodeHandle& nh)
    {
        const auto val = ROSHelpers::GetParamRequired<double>(nh, "stretching_threshold", __func__);
        return val.GetImmutable();
    }

    ////////////////////////////////////////////////////////////////////////////
    // Logging functionality
    ////////////////////////////////////////////////////////////////////////////

    inline bool GetLoggingEnabled(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParam(nh, "logging_enabled", false);
    }

    inline std::string GetLogFolder(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamNoWarn<std::string>(nh, "log_folder", "/tmp/");
    }

    inline std::string GetDijkstrasStorageLocation(ros::NodeHandle& nh)
    {
        std::string dijkstras_file_path;
        switch (GetTaskType(nh))
        {
            case TaskType::CLOTH_WAFR:
                dijkstras_file_path = "/home/dmcconac/Dropbox/catkin_ws/src/smmap/logs/cloth_wafr.dijkstras_serialized";
                break;

            case TaskType::CLOTH_SINGLE_POLE:
                dijkstras_file_path = "/home/dmcconac/Dropbox/catkin_ws/src/smmap/logs/cloth_single_pole.dijkstras_serialized";
                break;

            case TaskType::CLOTH_WALL:
                dijkstras_file_path = "/home/dmcconac/Dropbox/catkin_ws/src/smmap/logs/cloth_wall.dijkstras_serialized";
                break;

            case TaskType::CLOTH_DOUBLE_SLIT:
                dijkstras_file_path = "/home/dmcconac/Dropbox/catkin_ws/src/smmap/logs/cloth_double_slit.dijkstras_serialized";
                break;

            case TaskType::ROPE_MAZE:
                dijkstras_file_path = "/home/dmcconac/Dropbox/catkin_ws/src/smmap/logs/rope_maze.dijkstras_serialized";
                break;

            default:
                dijkstras_file_path = "/home/dmcconac/Dropbox/catkin_ws/src/smmap/logs/unknown_trial.dijkstras_serialized";
                break;
        }

        return ROSHelpers::GetParamNoWarn<std::string>(nh, "dijkstras_file_path", dijkstras_file_path);
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
        return ROSHelpers::GetParamNoWarn<std::string>(nh, "screenshot_folder", GetLogFolder(nh) + "screenshots/");
    }

    ////////////////////////////////////////////////////////////////////////////
    // ROS Topic settings
    ////////////////////////////////////////////////////////////////////////////

    inline std::string GetTestGrippersPosesTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamNoWarn<std::string>(nh, "test_grippers_poses_topic", "test_grippers_poses");
    }

    inline std::string GetExecuteGrippersMovementTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamNoWarn<std::string>(nh, "execute_grippers_movement_topic", "execute_grippers_movement");
    }

    inline std::string GetSimulatorFeedbackTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamNoWarn<std::string>(nh, "simulator_feedback_topic", "simulator_feedback");
    }

    inline std::string GetCoverPointsTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamNoWarn<std::string>(nh, "get_cover_points_topic", "get_cover_points");
    }

    inline std::string GetCoverPointNormalsTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamNoWarn<std::string>(nh, "get_cover_point_normals_topic", "get_cover_point_normals");
    }

    inline std::string GetMirrorLineTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamNoWarn<std::string>(nh, "get_mirror_line_topic", "get_mirror_line");
    }

    inline std::string GetFreeSpaceGraphTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamNoWarn<std::string>(nh, "get_free_space_graph_topic", "get_free_space_graph");
    }

    inline std::string GetSignedDistanceFieldTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamNoWarn<std::string>(nh, "get_signed_distance_field_topic", "get_signed_distance_field");
    }

    inline std::string GetGripperNamesTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamNoWarn<std::string>(nh, "get_gripper_names_topic", "get_gripper_names");
    }

    inline std::string GetGripperAttachedNodeIndicesTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamNoWarn<std::string>(nh, "get_gripper_attached_node_indices_topic", "get_gripper_attached_node_indices");
    }

    inline std::string GetGripperPoseTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamNoWarn<std::string>(nh, "get_gripper_pose_topic", "get_gripper_pose");
    }

    inline std::string GetObjectInitialConfigurationTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamNoWarn<std::string>(nh, "get_object_initial_configuration_topic", "get_object_initial_configuration");
    }

    inline std::string GetObjectCurrentConfigurationTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamNoWarn<std::string>(nh, "get_object_current_configuration_topic", "get_object_current_configuration");
    }

    inline std::string GetVisualizationMarkerTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamNoWarn<std::string>(nh, "visualization_marker_topic", "visualization_marker");
    }

    inline std::string GetVisualizationMarkerArrayTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamNoWarn<std::string>(nh, "visualization_marker_array_topic", "visualization_marker_vector");
    }

    inline std::string GetConfidenceTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamNoWarn<std::string>(nh, "confidence_topic", "confidence");
    }

    inline std::string GetConfidenceImageTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamNoWarn<std::string>(nh, "confidence_image_topic", "confidence_image");
    }

    inline std::string GetGripperCollisionCheckTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamNoWarn<std::string>(nh, "get_gripper_collision_check_topic", "get_gripper_collision_check");
    }

    inline std::string GetTerminateSimulationTopic(ros::NodeHandle& nh)
    {
        return ROSHelpers::GetParamNoWarn<std::string>(nh, "terminate_simulation_topic", "terminate_simulation");
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
