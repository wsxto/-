#include "arm_math.h"
#include <math.h>

extern "C" float arm_sin_f32(float value)
{
    return sinf(value);
}

extern "C" float arm_cos_f32(float value)
{
    return cosf(value);
}
