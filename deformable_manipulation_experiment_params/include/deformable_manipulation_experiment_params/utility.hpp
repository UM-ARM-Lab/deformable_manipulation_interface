#pragma once

#include <type_traits>
#include <cstdint>
#include <arc_utilities/arc_helpers.hpp>

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

    inline int PressKeyToContinue(const std::string& message = "Press any key to continue ")
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
}
