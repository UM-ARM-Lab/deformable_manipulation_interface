#pragma once

#include <type_traits>
#include <cstdint>

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
}
