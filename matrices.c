#include "matrices.h"

struct vec3 vec3_scale(struct vec3 v, float s) {
    return (struct vec3){ v.x*s, v.y*s, v.z*s };
}

struct vec3 vec3_normalize(struct vec3 v) {
    if (v.x == 0 && v.y == 0 && v.z == 0)
        return v;
    return vec3_scale(v, 1/vec3_length(v));
}

float vec3_length(struct vec3 v) {
    return sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
}
