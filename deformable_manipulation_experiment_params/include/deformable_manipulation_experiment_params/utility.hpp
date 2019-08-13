#pragma once

#include <ros/ros.h>
#include <type_traits>
#include <cstdint>
#include <Eigen/Dense>
#include <arc_utilities/arc_helpers.hpp>
#include <arc_utilities/ros_helpers.hpp>

namespace smmap
{
    // Desiged to work with any integer type, not floats
    template<typename T>
    inline int GetNumberOfDigits(T i)
    {
        // Safety check on the type we've been called with
        static_assert(std::is_same<T, uint8_t>::value
                      || std::is_same<T, uint16_t>::value
                      || std::is_same<T, uint32_t>::value
                      || std::is_same<T, uint64_t>::value
                      || std::is_same<T, int8_t>::value
                      || std::is_same<T, int16_t>::value
                      || std::is_same<T, int32_t>::value
                      || std::is_same<T, int64_t>::value,
                      "Type must be a fixed-size integral type");
        if (i < 0)
        {
            i = -i;
        }
        return i > 0 ? (int)std::log10((double)i) + 1 : 1;
    }

    inline int PressAnyKeyToContinue(const std::string& message = "Press any key to continue ")
    {
        if (!arc_helpers::IsDebuggerPresent())
        {
            std::cout << message << std::flush;
            auto key = arc_helpers::GetChar();
            if (key != '\n')
            {
                std::cout << std::endl;
            }
            return key;
        }
        else
        {
            std::cout << "Process is under debugger, use breakpoints for "
                      << "interactive flow control instead. Message: "
                      << message << std::endl;
            return '\n';
        }
    }

    inline Eigen::Vector3d GetVector3FromParamServer(
            ros::NodeHandle& nh,
            const std::string& base_name)
    {
        using namespace ROSHelpers;
        return Eigen::Vector3d(
                    GetParamRequired<double>(nh, base_name + "_x", __func__).GetImmutable(),
                    GetParamRequired<double>(nh, base_name + "_y", __func__).GetImmutable(),
                    GetParamRequired<double>(nh, base_name + "_z", __func__).GetImmutable());
    }

    inline Eigen::Quaterniond GetQuaternionFromParamServer(
            ros::NodeHandle& nh,
            const std::string& base_name)
    {
        using namespace ROSHelpers;
        return Eigen::Quaterniond(
                    GetParamRequired<double>(nh, base_name + "_w", __func__).GetImmutable(),
                    GetParamRequired<double>(nh, base_name + "_x", __func__).GetImmutable(),
                    GetParamRequired<double>(nh, base_name + "_y", __func__).GetImmutable(),
                    GetParamRequired<double>(nh, base_name + "_z", __func__).GetImmutable());
    }

    inline Eigen::Isometry3d GetPoseFromParamSerer(
            ros::NodeHandle& nh,
            const std::string& base_name,
            const bool rotation_optional = true)
    {
        const Eigen::Translation3d trans(GetVector3FromParamServer(nh, base_name + "_pos"));
        try
        {
            const Eigen::Quaterniond quat = GetQuaternionFromParamServer(nh, base_name + "_quat");
            return trans * quat;
        }
        catch (const std::invalid_argument& ex)
        {
            if (!rotation_optional)
            {
                throw;
            }
            (void)ex;
            return Eigen::Isometry3d(trans);
        }
    }
}
