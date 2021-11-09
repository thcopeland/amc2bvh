#include <string.h>
#include <ctype.h>
#include <assert.h>
#include <math.h>
#include "amc2bvh.h"

enum parsing_mode {
    MODE_NONE,
    MODE_UNIT,
    MODE_DOC,
    MODE_ROOT,
    MODE_BONES,
    MODE_BONE,
    MODE_TREE,
    MODE_MOTION
};

struct amc_skeleton *parse_asf_skeleton(FILE *asf, unsigned char max_child_count, bool verbose) {
    struct amc_skeleton *skeleton = amc_skeleton_new(max_child_count);
    struct amc_joint *current_joint = NULL;
    bool unit_degrees = true;

    // parsing data
    char *buffer = xmalloc(BUFFSIZE);
    int line_num = 0, modes_encountered = 0;
    enum parsing_mode mode = MODE_NONE;

    while (readline(buffer, BUFFSIZE, asf)) {
        char *trimmed = trim(buffer);
        line_num++;
        modes_encountered |= (1 << mode);

        if (starts_with(buffer, "#")) continue; // comment
        if (strlen(trimmed) == 0) continue; // blank line
        if (starts_with(buffer, ":") && mode != MODE_BONE && mode != MODE_TREE) { // keyword, switch mode
            mode = 0;

            if (starts_with(buffer, ":version")) {
                if (verbose) printf("ASF version: %s\n", trim(buffer+8));
            } else if (starts_with(buffer, ":name")) {
                if (verbose) printf("Name: %s\n", trim(buffer+5));
            } else if (starts_with(buffer, ":units")) mode = MODE_UNIT;
            else if (starts_with(buffer, ":documentation")) mode = MODE_DOC;
            else if (starts_with(buffer, ":root")) mode = MODE_ROOT;
            else if (starts_with(buffer, ":bonedata")) mode = MODE_BONES;
            else if (starts_with(buffer, ":hierarchy")) mode = MODE_TREE;
            else FAIL("Encountered unknown keyword `%s'\n", trim(buffer));
        } else if (starts_with(buffer, " ") || starts_with(buffer, "\t")) {
            if (mode == MODE_BONES) {
                // Handle the 'bonedata' segment in the ASF file. This section contains a series
                // of subsections, one for every bone in the rig, so the real work is done in the
                // MODE_BONE mode.
                if (streq(trimmed, "begin")) {
                    current_joint = amc_joint_new(max_child_count);
                    mode = MODE_BONE;
                }
                else FAIL("Unexpected token %s on line %i\n", trimmed, line_num);
            } else if (mode == MODE_BONE) {
                // Handle a single subsection in the 'bonedata' section. It describes a single
                // bone, but contains no hierarchical information.
                char *prop = trimmed,
                     *val = bifurcate(prop, ' ');

                assert(current_joint); // shouldn't happen, but this prevents warnings
                if (streq(prop, "name")) {
                    current_joint->name = xmalloc(strlen(val)+1);
                    strcpy(current_joint->name, val);
                } else if (streq(prop, "direction")) {
                    current_joint->direction = parse_vec3(val, line_num);
                } else if (streq(prop, "length")) {
                    sscanf(val, "%f", &current_joint->length);
                } else if (streq(prop, "axis")) {
                    current_joint->rotation = parse_joint_rotation(val, unit_degrees, current_joint, line_num);
                } else if (streq(prop, "dof")) {
                    parse_channel_order(current_joint->channels, val, line_num);
                } else if (streq(prop, "end")) {
                    mode = MODE_BONES;
                    if (current_joint->name) {
                        jointmap_set(skeleton->map, current_joint->name, current_joint);
                        if (verbose) printf("Initialized bone `%s'\n", current_joint->name);
                        current_joint = NULL;
                    } else {
                        FAIL("Bone ending on line %i is missing a name\n", line_num);
                    }
                } else if (streq(prop, "id") || streq(prop, "axis") || streq(prop, "limits") || starts_with(prop, "(")) {
                    continue; // ignore id, axis, constraint, and constraint details
                } else {
                    FAIL("Unexpected bone property `%s' on line %i\n", prop, line_num);
                }
            } else if (mode == MODE_TREE) {
                // Handle the hierarchical information contained in the 'hierarchy' segment. This
                // section describes a single bone tree.
                if (streq(trimmed, "begin")) {
                     continue; // ignore begin since only one tree is permitted
                } else if (streq(trimmed, "end")) {
                    mode = MODE_NONE;
                } else {
                    char *parent_name = trimmed,
                         *children = bifurcate(parent_name, ' ');
                    struct amc_joint *parent = jointmap_get(skeleton->map, parent_name);
                    if (!parent) FAIL("Unrecognized bone `%s' referenced on line %i\n", parent_name, line_num);

                    while (children) {
                        char *child_name = children;
                        children = bifurcate(children, ' ');

                        struct amc_joint *child = jointmap_get(skeleton->map, child_name);
                        if (!child) FAIL("Unrecognized bone `%s' referenced on line %i\n", child_name, line_num);

                        parent->children[parent->child_count++] = child;

                        if (parent->child_count > max_child_count) {
                            FAIL("Bone `%s' has %i children, max permitted is %i\n", parent_name, parent->child_count, max_child_count);
                        }
                    }
                }
            } else if (mode == MODE_ROOT) {
                // Handle the 'root' segment, which describes the root joint. We ignore most
                // of the data here.
                char *prop = trim(buffer),
                     *val = bifurcate(prop, ' ');

                if (streq(prop, "order")) {
                    parse_channel_order(skeleton->root->channels, val, line_num);
                } else if (streq(prop, "position")) {
                    skeleton->root_position = parse_vec3(val, line_num);
                }
            } else if (mode == MODE_DOC && verbose) {
                // Handle a 'documentation' section.
                printf("Documentation: %s\n", trim(buffer));
            } else if (mode == MODE_UNIT) {
                // Handle a 'units' section. We ignore everything except angle
                // units.
                char *unit = trim(buffer),
                     *val = bifurcate(unit, ' ');
                if (streq(unit, "angle")) unit_degrees = streq(val, "deg");
                if (verbose) printf("Unit %s: %s %s\n", unit, val, streq(unit, "angle") ? "" : "(ignored)");
            }
        } else {
            FAIL("Encountered unexpected character `%c' on line %i\n", buffer[0], line_num);
        }
    }

    if (!(modes_encountered & (1 << MODE_ROOT))) FAIL("Missing root bone data\n");
    if (!(modes_encountered & (1 << MODE_BONES))) FAIL("Missing bone data\n");
    if (!(modes_encountered & (1 << MODE_TREE))) FAIL("Missing bone hierarchy data\n");
    if (verbose) printf("Finished constructing skeleton\n");

    free(buffer);

    return skeleton;
}

struct amc_motion *parse_amc_motion(FILE *amc, struct amc_skeleton *skeleton, bool verbose) {
    unsigned total_channels = compute_amc_joint_indices(skeleton->root, 0);
    if (verbose) printf("Computed joint motion indices\n");

    struct amc_motion *motion = amc_motion_new(total_channels);
    struct amc_sample *current_sample = NULL;
    float angle_conversion = 1;
    bool is_fully_specified = false;

    // parsing data
    char *buffer = xmalloc(BUFFSIZE);
    int line_num = 0;
    enum parsing_mode mode = MODE_NONE;

    while (readline(buffer, BUFFSIZE, amc)) {
        char *trimmed = trim(buffer);
        line_num++;

        if (starts_with(trimmed, "#")) continue; // comment
        else if (strlen(trimmed) == 0) continue; // blank line

        if (mode == MODE_NONE) {
            if (starts_with(trimmed, ":")) { // various flags of uncertain meaning
                if (streq(trimmed, ":RADIANS")) {
                    angle_conversion = 180/M_PI;
                    if (verbose) printf("AMC uses radians (conversion factor = %f)\n", angle_conversion);
                } else if (streq(trimmed, ":DEGREES")) {
                    angle_conversion = 1.0;
                    if (verbose) printf("AMC uses degrees (conversion factor = %f)\n", angle_conversion);
                } else if (streq(trimmed, ":FULLY-SPECIFIED")) {
                    is_fully_specified = true;
                } else if (verbose) {
                    printf("Warning: unrecognized AMC flag `%s'\n", trimmed);
                }
            } else if (isdigit(trimmed[0])) {
                mode = MODE_MOTION;
                motion->sample_count++;
                motion->samples = amc_sample_new(total_channels);
                current_sample = motion->samples;
                if (verbose) printf("Starting to parse frames\n");
            } else {
                bifurcate(trimmed, ' ');
                FAIL("Unexpected token `%s' on line %i\n", trimmed, line_num);
            }
        } else if (mode == MODE_MOTION) {
            if (isdigit(trimmed[0])) {
                motion->sample_count++;
                current_sample->next = amc_sample_new(total_channels);
                current_sample = current_sample->next;
            } else {
                char *joint_name = trimmed,
                     *channel_data = bifurcate(trimmed, ' ');
                struct amc_joint *joint = jointmap_get(skeleton->map, joint_name);
                if (!joint) FAIL("Unrecognized bone `%s' referenced on line %i\n", joint_name, line_num);
                parse_amc_joint_animation_channels(joint, current_sample, channel_data, line_num);
            }
        }
    }

    if (verbose) {
        printf("Parsed %i frames\n", motion->sample_count);
        if (!is_fully_specified) printf("Warning: this file may not be fully-specified (alternative formats may be unsupported)\n");
    }
    free(buffer);

    return motion;
}

void write_bvh_skeleton(FILE *bvh, struct amc_skeleton *skeleton) {
    fprintf(bvh, "HIERARCHY\n");
    write_bvh_joint(bvh, skeleton, skeleton->root, skeleton->root_position, 0);
}

void write_bvh_joint(FILE *bvh,
                     struct amc_skeleton *skeleton,
                     struct amc_joint *joint,
                     struct vec3 offset,
                     int depth) {
    fprintf_indent(depth, bvh, "%s %s\n", depth ? "JOINT" : "ROOT", joint->name);
    fprintf_indent(depth, bvh, "{\n");
    fprintf_indent(depth+1, bvh, "OFFSET\t%f\t%f\t%f\n", offset.x, offset.y, offset.z);

    // It doesn't seem to be an official part of the BVH spec, but the Blender
    // BVH parser expects translation and rotation channels to be either all or
    // none present, and rotations to be always present.
    bool has_translations = amc_joint_has_translation(joint);
    fprintf_indent(depth+1, bvh, "CHANNELS %i", has_translations ? 6 : 3);
    if (has_translations) fprintf(bvh, " Xposition Yposition Zposition");

    // although the rotations are to be applied in XYZ order, they are written
    // in the reverse order.
    fprintf(bvh, " Zrotation Yrotation Xrotation\n");

    struct vec3 child_offset = vec3_scale(vec3_normalize(joint->direction), joint->length);
    if (joint->child_count > 0) {
        for (unsigned i = 0; i < joint->child_count; i++) {
            write_bvh_joint(bvh, skeleton, joint->children[i], child_offset, depth+1);
        }
    } else {
        fprintf_indent(depth+1, bvh, "End Site\n");
        fprintf_indent(depth+1, bvh, "{\n");
        fprintf_indent(depth+2, bvh, "OFFSET\t%f\t%f\t%f\n", child_offset.x, child_offset.y, child_offset.z);
        fprintf_indent(depth+1, bvh, "}\n");
    }

    fprintf_indent(depth, bvh, "}\n");
}

void write_bvh_motion(FILE *bvh, struct amc_motion *motion, struct amc_skeleton *skeleton, float fps) {
    fprintf(bvh, "MOTION\n");
    fprintf(bvh, "Frames:\t%u\n", motion->sample_count);
    fprintf(bvh, "Frame Time:\t%f\n", 1/fps);
    struct amc_sample *sample = motion->samples;
    while (sample) {
        write_bvh_joint_sample(bvh, skeleton->root, sample);
        fprintf(bvh, "\n");
        sample = sample->next;
    }
}

void write_bvh_joint_sample(FILE *bvh, struct amc_joint *joint, struct amc_sample *sample) {
    if (amc_joint_has_translation(joint)) {
        // read the translation data
        float tx = 0, ty = 0, tz = 0;
        for (int i = 0; i < CHANNEL_COUNT; i++) {
            enum channel channel = joint->channels[i];
            float val = sample->data[joint->motion_index+i];

            if (channel == CHANNEL_TX) {
                tx = val;
            } else if (channel == CHANNEL_TY) {
                ty = val;
            } else if (channel == CHANNEL_TZ) {
                tz = val;
            }
        }

        fprintf(bvh, "\t%f\t%f\t%f", tx, ty, tz);
    }

    float deg2rad = M_PI/180,
          rad2deg = 1/deg2rad;

    // read the rotation data
    struct euler_triple sample_rotation;
    for (int i = 0, j = 0; i < CHANNEL_COUNT; i++) {
        enum channel channel = joint->channels[i];

        if (channel == CHANNEL_RX || channel == CHANNEL_RY || channel == CHANNEL_RZ) {
            sample_rotation.angles[j] = sample->data[joint->motion_index+i] * deg2rad;
            sample_rotation.order[j++] = channel;
        }
    }

    // apply the joint space to the animation rotation
    struct quat local = joint->rotation,
                local_inv = quat_inv(joint->rotation),
                motion = euler_to_quat(sample_rotation);
    struct euler_triple combined_rotation = quat_to_euler_xyz(quat_mul(local, quat_mul(motion, local_inv)));

    fprintf(bvh, "\t%f\t%f\t%f", combined_rotation.angles[2]*rad2deg, combined_rotation.angles[1]*rad2deg, combined_rotation.angles[0]*rad2deg);

    for (unsigned i = 0; i < joint->child_count; i++) {
        write_bvh_joint_sample(bvh, joint->children[i], sample);
    }
}

#include "render_test.c"

int main(int argc, char **argv) {
    if (argc > 3) {
        FILE *asf = fopen(argv[1], "r"),
             *amc = fopen(argv[2], "r"),
             *bvh = fopen(argv[3], "w");
        printf("Starting to parse %s\n", argv[1]);
        struct amc_skeleton *skeleton = parse_asf_skeleton(asf, 4, true);
        printf("Starting to parse %s\n", argv[2]);
        struct amc_motion *motion = parse_amc_motion(amc, skeleton, true);
        write_bvh_skeleton(bvh, skeleton);
        write_bvh_motion(bvh, motion, skeleton, 120);
        fclose(asf);
        fclose(amc);
        fclose(bvh);
        amc_skeleton_free(skeleton);
        amc_motion_free(motion);
    }
}

/*
  HELPER METHODS
*/

struct quat parse_joint_rotation(char *str, bool degrees, struct amc_joint *joint, int line_num) {
    struct euler_triple e;
    char order[4];
    if (sscanf(str, "%f %f %f %3s", e.angles, e.angles+1, e.angles+2, order) != 4) {
        FAIL("Expected rotation axis data on line %i to have three components and an order\n", line_num);
    }

    float conversion = degrees ? M_PI/180 : 1;
    e.angles[0] *= conversion;
    e.angles[1] *= conversion;
    e.angles[2] *= conversion;

    for (int i = 0; i < 3; i++) {
        if (tolower(order[i]) == 'x') e.order[i] = CHANNEL_RX;
        else if (tolower(order[i]) == 'y') e.order[i] = CHANNEL_RY;
        else if (tolower(order[i]) == 'z') e.order[i] = CHANNEL_RZ;
        else FAIL("Unexpected axis `%c' on line %i (expected X, Y, or Z)\n", order[i], line_num);
    }

    joint->rotation2 = e;

    return euler_to_quat(e);
}

void parse_amc_joint_animation_channels(struct amc_joint *joint, struct amc_sample *sample, char *str, int line_num) {
    for (int i = 0; i < CHANNEL_COUNT; i++) {
        enum channel channel = joint->channels[i];

        if (channel == CHANNEL_EMPTY || !str) {
            if (str || channel != CHANNEL_EMPTY) {
                int exp = 0;
                while (exp < CHANNEL_COUNT && joint->channels[exp] != CHANNEL_EMPTY) exp++;
                FAIL("Bone `%s' given an incorrect number of animation channels on line %i (expected %i)\n", joint->name, line_num, exp);
            } else {
                break; // finished parsing bone channels
            }
        } else {
            // parse an animation channel
            char *val = str;
            str = bifurcate(str, ' ');
            sample->data[joint->motion_index+i] = (float) atof(val);
        }
    }
}

void parse_channel_order(enum channel *channels, char *str, int line_num) {
    for (int i = 0; str; i++) {
        if      (starts_with(str, "TX") || starts_with(str, "tx")) channels[i] = CHANNEL_TX;
		else if (starts_with(str, "TY") || starts_with(str, "ty")) channels[i] = CHANNEL_TY;
		else if (starts_with(str, "TZ") || starts_with(str, "tz")) channels[i] = CHANNEL_TZ;
		else if (starts_with(str, "RX") || starts_with(str, "rx")) channels[i] = CHANNEL_RX;
		else if (starts_with(str, "RY") || starts_with(str, "ry")) channels[i] = CHANNEL_RY;
		else if (starts_with(str, "RZ") || starts_with(str, "rz")) channels[i] = CHANNEL_RZ;
		else if (starts_with(str, "L")  || starts_with(str, "l"))  channels[i] = CHANNEL_L;
        else FAIL("Unable to parse channel `%.2s' on line %i\n", str, line_num);
        str = bifurcate(str, ' ');
    }
}

struct vec3 parse_vec3(char *str, int line_num) {
    struct vec3 vec;
    if (sscanf(str, "%f %f %f", &vec.x, &vec.y, &vec.z) != 3) {
        FAIL("Expected vector on line %i to contain 3 components\n", line_num);
    }
    return vec;
}

struct amc_skeleton *amc_skeleton_new(unsigned char max_child_count) {
    struct amc_skeleton *skeleton = xmalloc(sizeof(*skeleton));
    skeleton->root = amc_joint_new(max_child_count);
    skeleton->root->name = xmalloc(5);
    strcpy(skeleton->root->name, "root");
    skeleton->map = jointmap_new();
    jointmap_set(skeleton->map, "root", skeleton->root);
    skeleton->root_position = (struct vec3){ .x=0, .y=0, .z=0 };
    return skeleton;
}

void amc_skeleton_free(struct amc_skeleton *skeleton) {
    amc_joint_free(skeleton->root, true);
    hashmap_free(skeleton->map);
    free(skeleton);
}

struct amc_joint *amc_joint_new(unsigned char max_child_count) {
    struct amc_joint *joint = xmalloc(sizeof(*joint));
    joint->name = NULL;
    joint->direction = (struct vec3) { .x=0, .y=0, .z=0 };
    joint->rotation = (struct quat) { .w=1, .x=0, .y=0, .z=0 };
    // joint->rotation2 = (struct euler_triple) { .alpha=0, .beta=0, .gamma=0, .first=CHANNEL_RX, .second=CHANNEL_RY, .third=CHANNEL_RZ };
    joint->children = xmalloc(sizeof(*joint->children)*max_child_count);
    joint->child_count = 0;
    joint->length = 0;
    joint->motion_index = 0;
    for (int i = 0; i < CHANNEL_COUNT; i++) {
        joint->channels[i] = CHANNEL_EMPTY;
    }
    return joint;
}

bool amc_joint_has_translation(struct amc_joint *joint) {
    for (int i = 0; i < CHANNEL_COUNT && joint->channels[i] != CHANNEL_EMPTY; i++) {
        enum channel channel = joint->channels[i];
        if (channel == CHANNEL_TX || channel == CHANNEL_TY || channel == CHANNEL_TZ) {
            return true;
        }
    }
    return false;
}

void amc_joint_free(struct amc_joint *joint, bool deep) {
    for (unsigned i = 0; (deep && i < joint->child_count); i++) {
        amc_joint_free(joint->children[i], deep);
    }

    free(joint->children);
    free(joint->name);
    free(joint);
}

struct amc_motion *amc_motion_new(unsigned total_channels) {
    struct amc_motion *motion = xmalloc(sizeof(*motion));
    motion->total_channels = total_channels;
    motion->sample_count = 0;
    motion->samples = NULL;
    return motion;
}

void amc_motion_free(struct amc_motion *motion) {
    struct amc_sample *sample = motion->samples;
    while (sample) {
        struct amc_sample *next = sample->next;
        free(sample->data);
        free(sample);
        sample = next;
    }
    free(motion);
}

struct amc_sample *amc_sample_new(unsigned total_channels) {
    struct amc_sample *sample = xmalloc(sizeof(*sample));
    sample->data = xcalloc(total_channels, sizeof(*sample->data));
    sample->next = NULL;
    return sample;
}

unsigned compute_amc_joint_indices(struct amc_joint *joint, unsigned offset) {
    // recursively assign an array index based on position in the tree
    joint->motion_index = offset;
    offset += CHANNEL_COUNT;
    for (unsigned i = 0; i < joint->child_count; i++) {
        offset = compute_amc_joint_indices(joint->children[i], offset);
    }
    return offset;
}

void *xmalloc(size_t size) {
    void *mem = malloc(size);
    if (mem) return mem;
    FAIL("Unable to allocate sufficient memory\n");
}

void *xcalloc(size_t num, size_t size) {
    void *mem = calloc(num, size);
    if (mem) return mem;
    FAIL("Unable to allocate sufficient memory\n");
}

void *xrealloc(void *mem, size_t size) {
    void *new_mem = realloc(mem, size);
    if (new_mem) return new_mem;
    FAIL("Unable to allocate sufficient memory\n");
}

char *readline(char *str, int n, FILE *f) {
    char *res = fgets(str, n, f);

    if (res) {
        int len = strlen(str);
        if (len == n-1 && str[len-1] != '\n') FAIL("Line length exceeds internal buffer\n");
    }

    return res;
}

char *trim(char *str) {
    while (isspace(*str)) str++;
    for (int i = strlen(str)-1; isspace(str[i]); i--) str[i] = '\0';
    return str;
}

char *bifurcate(char *str, char delim) {
    while (*str) {
        if (*str == delim) {
            *str = '\0';
            return str+1;
        }
        str++;
    }
    return NULL;
}

bool streq(char *str, char *str2) {
    return strcmp(str, str2) == 0;
}

bool starts_with(char *str, char *pref) {
    return strncmp(str, pref, strlen(pref)) == 0;
}

struct hashmap *jointmap_new(void) {
     return hashmap_new_with_allocator(xmalloc, xrealloc, free,
                                      sizeof(struct jointmap_item), 16,
                                      0, 0,
                                      jointmap_hash,
                                      jointmap_cmp,
                                      NULL,
                                      NULL);
}

void jointmap_free(struct hashmap *map) {
    hashmap_free(map);
}

void *jointmap_get(struct hashmap *map, char *name) {
    struct jointmap_item search = { .name = name },
                         *result = hashmap_get(map, &search);
    return result ? result->joint : NULL;
}

void jointmap_set(struct hashmap *map, char *name, void *joint) {
    hashmap_set(map, &(struct jointmap_item ){ .name = name, .joint = joint });
}

int jointmap_cmp(const void *a, const void *b, void *data) {
    const struct jointmap_item *ja = a;
    const struct jointmap_item *jb = b;
    return strcmp(ja->name, jb->name);
}

uint64_t jointmap_hash(const void *item, uint64_t seed0, uint64_t seed1) {
    const struct jointmap_item *ji = item;
    return hashmap_sip(ji->name, strlen(ji->name), seed0, seed1);
}

/*
    MATH HELPERS
*/

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
    return (struct quat) { .w=c, .x=s*axis.x, .y=s*axis.y, .z=s*axis.z };
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

struct quat quat_inv(struct quat q) {
    float len = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    return (struct quat) { .w=q.w/len, .x=-q.x/len, .y=-q.y/len, .z=-q.z/len };
}

struct quat euler_to_quat(struct euler_triple e) {
    struct quat q = { .w=1, .x=0, .y=0, .z=0 }; // identity

    // q = q3 * q2 * q1
    for (int i = 0; i < 3; i++) {
        struct vec3 axis = {
            .x = (e.order[i] == CHANNEL_RX),
            .y = (e.order[i] == CHANNEL_RY),
            .z = (e.order[i] == CHANNEL_RZ)
        };
        q = quat_mul(angle_axis_to_quat(e.angles[i], axis), q);
    }

    return q;
}

struct euler_triple quat_to_euler_xyz(struct quat q) {
    float roll = atan2(2*(q.w*q.x + q.y*q.z), 1-2*(q.x*q.x + q.y*q.y)),
          pitch = asin(fmax(fmin(2*(q.w*q.y - q.z*q.x), 0.9999), -0.9999)),
          yaw = atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z));

    return (struct euler_triple) {
        .angles = { roll, pitch, yaw },
        .order = { CHANNEL_RX, CHANNEL_RY, CHANNEL_RZ }
    };
}
