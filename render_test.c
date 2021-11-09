#include <attyr/attyr.h>
#include <attyr/short_names.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#define SCALE 1.5
#define RENDER_WIDTH 140
#define RENDER_HEIGHT 70

struct renderstate {
    mat4 transform;
    unsigned int face;
    float scale;
};

static vec4 vertices[] = {
    { -0.250000, 0.000000, 0.250000, 1.0 },
    { -0.073281, 1.000000, 0.073281, 1.0 },
    { -0.250000, 0.000000, -0.250000, 1.0 },
    { -0.073281, 1.000000, -0.073281, 1.0 },
    { 0.250000, 0.000000, 0.250000, 1.0 },
    { 0.073281, 1.000000, 0.073281, 1.0 },
    { 0.250000, 0.000000, -0.250000, 1.0 },
    { 0.073281, 1.000000, -0.073281, 1.0 },
};

static vec3 normals[] = {
    { -0.9847, 0.1740, 0.0000 },
    { 0.0000, 0.1740, -0.9847 },
    { 0.9847, 0.1740, 0.0000 },
    { 0.0000, 0.1740, 0.9847 },
    { 0.0000, -1.0000, 0.0000 },
    { 0.0000, 1.0000, 0.0000 },
};

static int faces[] = {
    2, 1, 3, 1, 1, 1,
    4, 2, 7, 2, 3, 2,
    8, 3, 5, 3, 7, 3,
    6, 4, 1, 4, 5, 4,
    7, 5, 1, 5, 3, 5,
    4, 6, 6, 6, 8, 6,
    2, 1, 4, 1, 3, 1,
    4, 2, 8, 2, 7, 2,
    8, 3, 6, 3, 5, 3,
    6, 4, 2, 4, 1, 4,
    7, 5, 5, 5, 1, 5,
    4, 6, 2, 6, 6, 6,
};

int vert_shader(vec4 *out1, vec4 *out2, vec4 *out3, void *data)
{
    struct renderstate *state = data;

    if (state->face >= sizeof(faces)/sizeof(faces[0])) {
        return 0;
    } else {
        vec4 *in1 = vertices + faces[state->face] - 1,
             *in2 = vertices + faces[state->face + 2] - 1,
             *in3 = vertices + faces[state->face + 4] - 1;

        mat4 perspective, scale;
        attyr_perspective(M_PI/4, RENDER_WIDTH/RENDER_HEIGHT, 0, &perspective);
        attyr_scale(&(attyr_vec3) { state->scale/2, state->scale, state->scale/2 }, &scale);

        attyr_mult_mat4x4_4x4(&perspective, &state->transform, &perspective);
        attyr_mult_mat4x4_4x4(&perspective, &scale, &perspective);

        attyr_mult_mat4x4_vec4(&perspective, in1, out1);
        attyr_mult_mat4x4_vec4(&perspective, in2, out2);
        attyr_mult_mat4x4_vec4(&perspective, in3, out3);

        state->face += 6;

        return 1;
     }
}

void frag_shader(attyr_vec4 *color, attyr_vec3 *coords, attyr_vec3 *pos, void *data) {
    struct renderstate *state = data;
    vec3 *in1 = normals + faces[state->face - 5] - 1,
         *in2 = normals + faces[state->face - 3] - 1,
         *in3 = normals + faces[state->face - 1] - 1,
         normal1, normal2, normal3, normal,
         light = { .x = 1, .y = 2, .z = 2 };
    attyr_scale_vec3(&light, 1/attyr_len_vec3(&light));
    mat3 m, transform = { state->transform.m11, state->transform.m12, state->transform.m13,
                          state->transform.m21, state->transform.m22, state->transform.m23,
                          state->transform.m31, state->transform.m32, state->transform.m33 };
    attyr_mult_mat3x3_vec3(&transform, in1, &normal1);
    attyr_mult_mat3x3_vec3(&transform, in2, &normal2);
    attyr_mult_mat3x3_vec3(&transform, in3, &normal3);
    attyr_init_mat3x3(&m, &normal1, &normal2, &normal3);
    attyr_mult_mat3x3_vec3(&m, coords, &normal);

    float illum = fmin(0.1 + fmax(attyr_dot_vec3(&normal, &light), 0.0), 0.9);
    attyr_init_vec4(color, illum, illum, illum, 1.0);
}

void render_bone(attyr_framebuffer_t *buffer, struct amc_joint *joint, mat4 *transform) {
    struct renderstate state = { .face = 0, .scale = joint->length };
    attyr_dup_mat4x4(transform, &state.transform);
    attyr_rasterize(buffer, vert_shader, frag_shader, &state);
}

void calculate_animation_transform(mat4 *animation, struct amc_joint *joint, struct amc_sample *sample) {
    float *anim_data = sample->data + joint->motion_index;
    mat4 rotation, translation;
    attyr_diag_mat4x4(1, &rotation);
    attyr_diag_mat4x4(1, &translation);

    // T = T3 * T2 * T1
    // R = R3 * R2 * R1
    for (int i = 0; i < CHANNEL_COUNT; i++) {
        enum channel channel = joint->channels[i];
        mat4 ch;

        if (channel == CHANNEL_EMPTY) {
            break;
        } else if (channel == CHANNEL_RX) {
            attyr_rotate_x(anim_data[i], &ch);
            attyr_mult_mat4x4_4x4(&ch, &rotation, &rotation);
        } else if (channel == CHANNEL_RY) {
            attyr_rotate_y(anim_data[i], &ch);
            attyr_mult_mat4x4_4x4(&ch, &rotation, &rotation);
        } else if (channel == CHANNEL_RZ) {
            attyr_rotate_z(anim_data[i], &ch);
            attyr_mult_mat4x4_4x4(&ch, &rotation, &rotation);
        } else if (channel == CHANNEL_TX) {
            attyr_translate(&(vec3) { anim_data[i], 0, 0 }, &ch);
            attyr_mult_mat4x4_4x4(&ch, &translation, &translation);
        } else if (channel == CHANNEL_TY) {
            attyr_translate(&(vec3) { 0, anim_data[i], 0 }, &ch);
            attyr_mult_mat4x4_4x4(&ch, &translation, &translation);
        } else if (channel == CHANNEL_TZ) {
           attyr_translate(&(vec3) { 0, 0, anim_data[i] }, &ch);
           attyr_mult_mat4x4_4x4(&ch, &translation, &translation);
       }
    }

    // A = T*R
    attyr_mult_mat4x4_4x4(&translation, &rotation, animation);
}

void calculate_axis_transform(mat4 *transform, mat4 *inv_transform, struct amc_joint *joint) {
    struct euler_triple e = quat_to_euler_xyz(joint->rotation);

    attyr_diag_mat4x4(1, transform);
    attyr_diag_mat4x4(1, inv_transform);

    // local joint space matrices
    // J = C3 * C2 * C1
    // J^ = C1^ * C2^ * C3^
    for (int i = 0; i < 3; i++) {
        mat4 ch, inv;

        if (e.order[i] == CHANNEL_RX) {
            attyr_rotate_x(e.angles[i], &ch);
            attyr_rotate_x(-e.angles[i], &inv);
        } else if (e.order[i] == CHANNEL_RY) {
            attyr_rotate_y(e.angles[i], &ch);
            attyr_rotate_y(-e.angles[i], &inv);
        } else if (e.order[i] == CHANNEL_RZ) {
            attyr_rotate_z(e.angles[i], &ch);
            attyr_rotate_z(-e.angles[i], &inv);
        } else {
            continue;
        }

        attyr_mult_mat4x4_4x4(&ch, transform, transform);
        attyr_mult_mat4x4_4x4(inv_transform, &inv, inv_transform);
    }
}

void render_bones(attyr_framebuffer_t *buffer, struct amc_joint *joint, struct amc_sample *sample, mat4 *inherited) {
    // A significant portion of this could be precalculated. But performance is pretty good anyway,
    // and that would complicate the code.
    vec3 dir = { joint->direction.x, joint->direction.y, joint->direction.z };
    attyr_scale_vec3(&dir, joint->length);
    mat4 animation, joint_animation, local, transform, translation, joint_space, inv_joint_space;
    calculate_animation_transform(&animation, joint, sample);
    calculate_axis_transform(&joint_space, &inv_joint_space, joint);

    // convert into bone space
    // Ja = J * A * J^
    attyr_mult_mat4x4_4x4(&joint_space, &animation, &joint_animation);
    attyr_mult_mat4x4_4x4(&joint_animation, &inv_joint_space, &joint_animation);

    // calculate bone transform (inherited by children)
    // L = Lparent * Ja * B
    attyr_translate(&dir, &translation);
    attyr_mult_mat4x4_4x4(inherited, &joint_animation, &transform);
    attyr_mult_mat4x4_4x4(&transform, &translation, &transform);

    // calculate bone rendering transform
    // R = Lparent * Ja * D
    attyr_mult_mat4x4_4x4(inherited, &joint_animation, &local);
    if (attyr_len_vec3(&dir) > 0) {
        // point the bone in the correct direction
        vec3 axis, i = { 0, 1, 0 };
        attyr_normalize_vec3(&dir);
        attyr_cross_vec3(&i, &dir, &axis);
        attyr_normalize_vec3(&axis);
        mat4 rotation;
        attyr_rotate(&axis, acos(attyr_dot_vec3(&dir, &i)), &rotation);
        attyr_mult_mat4x4_4x4(&local, &rotation, &local);
    }

    render_bone(buffer, joint, &local);

    for (unsigned i = 0; i < joint->child_count; i++) {
        struct amc_joint *child = joint->children[i];
        render_bones(buffer, child, sample, &transform);
    }
}

static volatile int is_alive = 1;

void handle_sigint() {
    is_alive = 0;
}

void render_test(struct amc_skeleton *skeleton, struct amc_motion *motion) {
    struct sigaction sa;
    memset(&sa, 0, sizeof(sa));
    sa.sa_handler = handle_sigint;
    sigaction(SIGINT, &sa, NULL);

    attyr_framebuffer_t *framebuffer = attyr_init_framebuffer((int) (RENDER_WIDTH*SCALE), (int) (RENDER_HEIGHT*SCALE));
    float time = 0;
    struct amc_sample *sample = motion->samples;
    printf("\x1b[?25l");
    while (is_alive) {
        printf("\x1b[H");
        attyr_reset_framebuffer(framebuffer);

        mat4 rotateY, translate, transform;
        attyr_rotate_y(1.57, &rotateY);
        attyr_translate(&(attyr_vec3) { 0, -15, -40 }, &translate);
        attyr_mult_mat4x4_4x4(&translate, &rotateY, &transform);
        render_bones(framebuffer, skeleton->root, sample, &transform);
        attyr_render_truecolor(framebuffer);

        sample = sample->next ? sample->next : motion->samples;
        time += 0.01;
    }
    printf("\x1b[?25h\n\n");
    attyr_free_framebuffer(framebuffer);
}
