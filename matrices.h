#include <math.h>

struct vec3 {
    float x, y, z;
};

struct vec3 vec3_scale(struct vec3 v, float s);
struct vec3 vec3_normalize(struct vec3 v);
float vec3_length(struct vec3 v);
