#pragma once

#include "math.hpp"

// grody hack - this should go once C++11 is supported
// on all platforms.
#include <math.h>

namespace matrix
{

template<typename Type>
Type wrap_pi(Type x)
{
    if (!isfinite(x)) {
        return x;
    }

    while (x >= (Type)M_PI) {
        x -= (Type)(2.0 * M_PI);

    }

    while (x < (Type)(-M_PI)) {
        x += (Type)(2.0 * M_PI);

    }

    return x;
}


};
