#include "Math/novus_math.h"

float map(float target, float from_min, float from_max, float to_min, float to_max){
    float mult = (float)(to_max - to_min) / (float)(from_max - from_min);
    target = target - from_min;
    return to_min + (target * mult);
}
