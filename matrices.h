enum channel {
    CHANNEL_TX,
    CHANNEL_TY,
    CHANNEL_TZ,
    CHANNEL_RX,
    CHANNEL_RY,
    CHANNEL_RZ,
    CHANNEL_L,
    CHANNEL_EMPTY
};

struct vec3 {
    float x, y, z;
};

struct quat {
    float w, x, y, z;
};

struct euler_triple {
    float angles[3];
    enum channel order[3];
};

struct vec3 vec3_scale(struct vec3 v, float s);
struct vec3 vec3_normalize(struct vec3 v);
float vec3_length(struct vec3 v);

struct quat angle_axis_to_quat(float angle, struct vec3 axis);
struct quat quat_mul(struct quat a, struct quat b);
struct quat quat_conj(struct quat q);
struct quat euler_to_quat(struct euler_triple e);
struct euler_triple quat_to_euler_xyz(struct quat q);
