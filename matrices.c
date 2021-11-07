#include <math.h>
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

struct quat angle_axis_to_quat(float angle, struct vec3 axis) {
    float s = sin(angle/2),
          c = cos(angle/2);
    return (struct quat) { .w=s, .x=c*axis.x, .y=c*axis.y, .z=c*axis.z };
}

struct quat quat_mul(struct quat a, struct quat b) {
    return (struct quat) {
        .w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        .x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        .y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        .z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
     };
}

struct quat quat_conj(struct quat q) {
    return (struct quat) { .w=q.w, .x=-q.x, .y=-q.y, .z=-q.z };
}

struct quat euler_to_quat(struct euler_triple e) {
    struct quat q = { .w=1, .x=0, .y=0, .z=0 }; // identity

    for (int i = 0; i < 3; i++) {
        enum channel channel = e.channels[i];
        struct vec3 axis = {
            .x = (channel == CHANNEL_RX),
            .y = (channel == CHANNEL_RY),
            .z = (channel == CHANNEL_RZ)
        };

        q = quat_mul(q, angle_axis_to_quat(e.angles[i], axis));
    }

    return q;
}

struct euler_triple quat_to_euler_xyz(struct quat q) {
    float roll = atan2(2*(q.w*q.x + q.y*q.z), 1-2*(q.x*q.x + q.y*q.y)),
          sin_pitch = 2*(q.w*q.y - q.z*q.x),
          pitch = (fabs(sin_pitch) >= 1) ? copysign(M_PI/2, sin_pitch) : asin(sin_pitch),
          roll = atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z));

    return (struct euler_triple) {
        .angles = { roll, pitch, yaw },
        .order = { CHANNEL_RX, CHANNEL_RY, CHANNEL_RZ }
    };
}
